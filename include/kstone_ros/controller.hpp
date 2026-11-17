#pragma once

#include <algorithm>
#include <functional>
#include <kstone/runtime/snapshot.hpp>
#include <kstone_ros/handoff.hpp>
#include <thread>
#include <kstone/module/builtins.hpp>
#include <kstone/runtime/config.hpp>
#include <kstone/runtime/debug_server.hpp>
#include <map>
#include <set>
#include <string_view>

namespace kstone_ros {
using Node = kstone::ConfigNode;
/// Serialize a config node as compact JSON.
std::string json(const Node &value);
/// Parse JSON text into a config node; throws on invalid JSON.
Node parse_json(const std::string &text);
/// Throw std::runtime_error with the status message when `status` is an error.
void require(kstone::Status status);
/// Throw std::invalid_argument unless `value` fits `info`: a rigid pose, or finite numbers of the port size.
void validate_port(const kstone::PortInfo &info,
                   const kstone::PortValue &value);
/// True when a number/vector value lies inside the port bounds; other port types always pass.
/// Out-of-bounds values are accepted; the owning module limits them. Fixed axes (lower == upper) are not checked.
bool within_bounds(const kstone::PortInfo &info, const kstone::PortValue &value);
/// Return the value of `result`, or throw std::runtime_error with its error message.
template <class T> T take(kstone::Result<T> result) {
  if (!result)
    throw std::runtime_error(result.error().message);
  return std::move(result).value();
}
/// Encode a pose as {"position": [x, y, z], "orientation": [w, x, y, z]}.
Node pose_node(const Eigen::Isometry3d &pose);
/// Build a pose from xyz and a wxyz quaternion (normalized); throws on wrong sizes, non-finite or zero values.
Eigen::Isometry3d make_pose(const std::vector<double> &translation,
                            const std::vector<double> &wxyz);
/// Controller construction options; the ROS node fills them from its parameters.
struct Options {
  /// Expected chassis module ID; when set it must name the root-owning chassis module.
  std::string chassis_module;
  /// Variables substituted into the kstone YAML configuration.
  std::map<std::string, std::string> variables;
  /// Maximum age (s) of a motion command at application, and of joint/base feedback; both > 0.
  double command_timeout_s{0.5}, feedback_timeout_s{0.25};
  /// Engine run time (s); 0 runs until stop().
  double duration_s{0};
  /// Overrides the configuration's `realtime` setting when set.
  std::optional<bool> realtime;
  /// Clock for the engine and logs; null uses the kstone default clock.
  std::shared_ptr<const kstone::Clock> clock;
  /// Driver injection for embedding/tests; not a ROS parameter. Null uses ROS joint/base feedback.
  std::shared_ptr<kstone::RobotInterface> robot;
  /// Starts the kstone debug server when set.
  std::optional<kstone::DebugServerOptions> debug;
};
/// Joint names with matching positions and (optionally) velocities.
struct JointValues {
  std::vector<std::string> names;
  std::vector<double> positions, velocities;
};
/// True for module types whose ports the ROS node exposes as topics and services.
inline bool supported_ros_module(std::string_view type) {
  return type=="frame_target" || type=="chassis" || type=="joint_target";
}
/// Immutable discovery metadata; no Module pointers escape the engine owner.
struct ModuleInfo {
  std::string module_id, module_type;
  bool can_home{}, chassis_omni{};
  std::string chassis_frame;
  std::vector<kstone::PortInfo> inputs;
  std::vector<kstone::OutputInfo> outputs;
  const std::string &id() const { return module_id; }
  const std::string &type() const { return module_type; }
  bool homeable() const { return can_home; }
  const auto &ports() const { return inputs; }
};
/// Result of one request, identified by the sequence number its submitting call returned.
struct Outcome {
  std::uint64_t sequence{};
  bool applied{};
  Text<192> error;  ///< Rejection reason; empty when applied.
};
/// State the owner thread publishes after each cycle; read it with Controller::snapshot() after poll().
struct Snapshot {
  kstone::EngineSnapshot engine;
  Eigen::VectorXd q;
  std::vector<std::vector<kstone::OutputValue>> outputs;
  std::vector<kstone::PortValue> accepted;
  std::vector<std::vector<Eigen::Isometry3d>> poses;
  std::vector<unsigned char> expired;
  std::vector<Text<64>> home_states;
  std::array<Outcome, 32> outcomes{};
  std::size_t outcome_count{};
  double stamp{}, dt{};
  std::uint64_t generation{};
  bool safe{}, joints_attached{}, base_attached{};
  Text<192> fault;
  kstone::Twist2D base_velocity;
  Eigen::Isometry3d chassis_pose{Eigen::Isometry3d::Identity()};
};
/// Runs a kstone Engine on its own owner thread and exchanges requests, feedback and snapshots with one
/// ROS thread through lock-free queues. Request methods return a sequence number whose result appears in
/// Snapshot::outcomes; they throw std::invalid_argument for bad input and std::runtime_error after the owner
/// has stopped or failed, or when the request queue is full.
class Controller {
public:
  /// Load the kstone YAML `file` and build the engine; throws on an invalid configuration or options.
  Controller(const std::string &file, Options options = {});
  /// Stops the owner thread.
  ~Controller();
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;
  /// Start the owner thread; throws std::logic_error when called twice or after stop().
  void start();
  /// Stop the owner thread and the debug server; safe to call repeatedly.
  void stop() noexcept;
  /// Take the newest snapshot and refresh the public status fields; returns `written`.
  /// Single serialized ROS producer/consumer. Never calls Engine::step.
  bool poll();
  const auto &model() const { return model_; }
  /// Metadata of every configured module, in configuration order.
  const auto &modules() const { return metadata_; }
  /// Input port `name` of module `id`; throws std::invalid_argument when unknown.
  const kstone::PortInfo &port(const std::string &, const std::string &) const;
  /// Last value the owner accepted on input port `name` of module `id`.
  kstone::PortValue read(const std::string &, const std::string &) const;
  /// Submit a value for an input port. Newer writes to the same port replace pending ones.
  std::uint64_t write(const std::string &, const std::string &, const kstone::PortValue &);
  /// Enable or disable module `id`. Enabling a module that still needs a new command arms it instead: it starts
  /// on its next accepted target, so a pre-fault target is never resumed.
  std::uint64_t enable(const std::string &, bool);
  /// Start homing module `id`; throws std::invalid_argument when it cannot home.
  std::uint64_t home(const std::string &);
  /// Set the `target` port of module `id` to its current frame pose.
  std::uint64_t reset_target(const std::string &);
  /// Home every enabled homeable module (including ones stopped by a fault) and zero the chassis velocity.
  std::uint64_t reset_home();
  /// Clear a latched fault. Motion modules are stopped and requests submitted before it are discarded; a module
  /// stopped this way resumes on its next accepted target.
  std::uint64_t recover();
  /// Submit joint targets routed to the joint_target modules owning each joint.
  /// Names outside every joint_target chain are skipped and, if given, appended to ignored.
  /// Requests submitted within one owner cycle are applied in order, each atomically with its own outcome;
  /// a later request only supersedes the joints it names.
  std::uint64_t joint_targets(const std::vector<std::string> &, const std::vector<double> &, std::vector<std::string> *ignored = nullptr);
  /// Submit measured joint positions sampled `age_s` seconds ago (at most feedback_timeout_s).
  std::uint64_t feedback_joints(const std::vector<std::string> &, const std::vector<double> &, double age_s = 0);
  /// Submit the measured root pose sampled `age_s` seconds ago; needs a free root.
  std::uint64_t feedback_base(const Eigen::Isometry3d &, double age_s = 0);
  /// Chassis control-center pose (center_in_root applied), as chassis drivers report odometry.
  std::uint64_t feedback_center(const Eigen::Isometry3d &, double age_s = 0);
  /// Engine cycle period (s) of the newest snapshot.
  double dt() const { return snapshot().dt; }
  /// Current time (s) of the controller clock.
  double now() const;
  /// ID of the chassis module, empty without one.
  const auto &chassis() const { return chassis_id_; }
  /// Bound debug server port, or 0 when the debug server is off.
  int debug_port() const { return debug_ ? debug_->port() : 0; }
  /// True when the model has a free-floating root joint.
  bool has_root() const { return root_ != nullptr; }
  bool has_joints() const { return !joints_.empty(); }
  /// Model configuration of the newest snapshot.
  const auto &q() const { return snapshots_.read().q; }
  /// Measured configuration of the newest snapshot.
  const auto &measured() const { return snapshots_.read().engine.state.q; }
  /// Commanded configuration and velocity of the newest snapshot.
  const auto &command_q() const { return snapshots_.read().engine.command.q; }
  const auto &velocity() const { return snapshots_.read().engine.command.v; }
  /// Current pose of the frame behind pose input `name` of module `id`, in the port's reference frame.
  Eigen::Isometry3d pose(const std::string &, const std::string &) const;
  /// Configured home pose of pose input `name` of module `id`; throws std::out_of_range when none.
  Eigen::Isometry3d home_pose(const std::string &, const std::string &) const;
  /// Root pose in the newest snapshot configuration.
  Eigen::Isometry3d base_pose() const;
  /// Chassis pose reported by the chassis module.
  Eigen::Isometry3d chassis_pose() const { return snapshot().chassis_pose; }
  /// Chassis velocity command; zero unless the last poll() returned true.
  kstone::Twist2D base_velocity() const;
  /// Positions (and velocities when `v` is given) of all unlocked joints in configuration `q`.
  JointValues joint_state(const Eigen::VectorXd &, const Eigen::VectorXd * = nullptr) const;
  /// Last command of the joints whose feedback group is attached; a detached group (e.g. an unplugged hand)
  /// gets no output while the other groups keep running.
  JointValues joint_command() const;
  /// Module and port description published on the `discover` topic.
  Node discovery() const;
  /// Status document published on the `state` topic (modules, groups, outcomes, counters).
  Node state() const;
  /// Newest snapshot taken by poll().
  const Snapshot &snapshot() const { return snapshots_.read(); }
  /// Engine and driver log records, drained by the ROS thread.
  const auto &logs() const { return logs_; }
  /// Set by poll(): `written` = the newest snapshot holds a safe, recent command; `fresh` = engine feedback is fresh.
  bool written{}, fresh{};
  /// Driver output gates: fresh feedback has been synced into the model.
  bool joints_attached{}, base_attached{};
  /// Set by poll(): latched fault reason (empty when healthy) and solve quality.
  std::string fault, quality{"uninitialized"};
private:
  enum class Operation { enable, home, reset_target, reset_home, recover };
  struct Command {
    Operation op{};
    std::size_t module{};
    bool enabled{};
    std::uint64_t sequence{}, generation{};
    double receipt{};
  };
  struct Input {
    std::uint64_t sequence{}, generation{};
    double receipt{};
    kstone::PortValue value;
  };
  // Per input and port label: the newest position not yet known to be consumed, the request that set it
  // (0 = none) and that request's receipt. Unconsumed requests merge, so coalescing drops no joint.
  struct JointBatch {
    std::uint64_t sequence{}, generation{};
    std::vector<Eigen::VectorXd> values;
    std::vector<std::vector<std::uint64_t>> sequences;
    std::vector<std::vector<double>> receipts;
  };
  struct Feedback {
    std::uint64_t sequence{}, generation{};
    Eigen::VectorXd q;
    std::vector<double> stamps;
  };
  class FeedbackRobot;
  class Observer;
  Options options_;
  kstone::Context context_;
  kstone::ModelPtr model_;
  std::unique_ptr<kstone::Engine> engine_;
  std::shared_ptr<kstone::DebugServer> debug_;
  std::shared_ptr<FeedbackRobot> feedback_;
  std::unique_ptr<kstone::Workspace> workspace_;
  std::vector<std::shared_ptr<const ModuleInfo>> metadata_;
  std::vector<kstone::JointInfo> joints_;  // model order
  const kstone::JointInfo *root_{};
  std::string chassis_id_;
  std::shared_ptr<kstone::ChassisModule> chassis_;
  Eigen::Isometry3d center_in_root_{Eigen::Isometry3d::Identity()};
  Eigen::VectorXd q_, measured_;
  std::vector<double> stamps_;
  std::vector<unsigned char> expired_, needs_command_;
  std::vector<kstone::PortValue> accepted_values_;
  std::map<std::pair<std::string, std::string>, Eigen::Isometry3d> homes_;
  std::vector<std::unique_ptr<Latest<Input>>> inputs_;
  std::vector<std::pair<std::size_t, std::size_t>> addresses_;
  // One coalesced aggregate request, bounded by immutable configured ports.
  Latest<JointBatch> joint_targets_;
  JointBatch joint_pending_;  // Producer-side merge of every request not yet consumed.
  std::atomic<std::uint64_t> joint_consumed_{0};  // Newest batch sequence the owner has processed.
  std::uint64_t processed_joints_{0};  // Owner copy of joint_consumed_.
  std::vector<std::uint64_t> joint_requests_;  // Owner scratch: pending request sequences.
  std::vector<Eigen::Index> joint_elements_;  // Owner scratch: elements written by one request.
  Eigen::VectorXd joint_values_;  // Owner scratch: values for joint_elements_.
  std::map<std::string, std::pair<std::size_t, std::size_t>> joint_routes_;
  std::set<std::string> ambiguous_joints_;
  std::vector<std::uint64_t> accepted_sequences_;
  // Partial joint targets whose omitted joints the module fills on resume; re-read after the step.
  std::vector<unsigned char> refresh_;
  Latest<Feedback> joint_input_;
  Feedback joint_candidate_;
  Latest<Input> base_input_;
  Latest<Snapshot> snapshots_;
  Fifo<Command, 64> commands_;
  std::shared_ptr<BufferedLog> logs_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  // 0 initialized, 1 running, 2 fatal/latched between runs, 3 stopped.
  std::atomic<int> lifecycle_{0};
  std::atomic<std::uint64_t> generation_{0}, rejected_{0}, coalesced_{0};
  std::uint64_t next_sequence_{0}, owner_generation_{0}, seen_failed_{0};
  std::uint64_t applied_joints_{0}, applied_base_{0};
  std::array<Outcome, 32> outcomes_{};
  std::size_t outcome_count_{};
  double owner_dt_{}, owner_limit_{}, owner_elapsed_{}, last_owner_time_{-1};
  // One tolerance for every cycle-gap check: three periods, never below 100 ms.
  static double scheduler_limit(double dt) { return std::max(0.1,3*dt); }
  bool latched_{false}, base_fresh_{false}, base_pending_{false};
  std::vector<unsigned char> required_;  // per joints_: needed for joint attach
  // Engine feedback groups: per joints_, the root's (npos without one), and which hold a joints_ entry.
  std::vector<std::size_t> joint_group_;
  std::size_t base_group_{std::size_t(-1)};
  std::vector<unsigned char> joint_groups_;
  std::vector<double> group_oldest_;
  std::uint64_t seen_attaches_{0};
  Eigen::Isometry3d base_feedback_pose_{Eigen::Isometry3d::Identity()};
  double base_receipt_{-1};
  Text<192> owner_fault_;
  double base_candidate_stamp_{-1};
  std::size_t module_index(const std::string &) const;
  std::size_t input_index(const std::string &, const std::string &) const;
  std::uint64_t enqueue(Operation, std::size_t = 0, bool = false);
  std::uint64_t submit(std::size_t, const kstone::PortValue &, std::uint64_t);
  void owner_run() noexcept;
  void before(kstone::Engine &);
  void after(const kstone::Engine &);
  void capture(bool safe);
  void prepare(Snapshot &);
  void apply(const Command &);
  void outcome(std::uint64_t, bool, std::string_view = {});
  void clear_motion();
  bool joints_synced() const;
  bool base_synced() const;
  void write_port(std::size_t, std::size_t, const kstone::PortValue &, double, double application_time = -1);
  void apply_joint_targets(const JointBatch &);
  void apply_joint_request(const JointBatch &, std::uint64_t sequence);
  void start_home(std::size_t, double);
  Eigen::Isometry3d port_pose(std::size_t, std::size_t);
};
} // namespace kstone_ros
