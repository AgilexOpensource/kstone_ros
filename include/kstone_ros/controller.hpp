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
std::string json(const Node &value);
Node parse_json(const std::string &text);
void require(kstone::Status status);
void validate_port(const kstone::PortInfo &info,
                   const kstone::PortValue &value);
// Out-of-bounds values are accepted; the owning module limits them. Fixed axes (lower == upper) are not checked.
bool within_bounds(const kstone::PortInfo &info, const kstone::PortValue &value);
template <class T> T take(kstone::Result<T> result) {
  if (!result)
    throw std::runtime_error(result.error().message);
  return std::move(result).value();
}
Node pose_node(const Eigen::Isometry3d &pose);
Eigen::Isometry3d make_pose(const std::vector<double> &translation,
                            const std::vector<double> &wxyz);
struct Options {
  std::string chassis_module;
  std::map<std::string, std::string> variables;
  double command_timeout_s{0.5}, feedback_timeout_s{0.25};
  double duration_s{0};
  std::optional<bool> realtime;
  std::shared_ptr<const kstone::Clock> clock;
  // Driver injection for embedding/tests; not a ROS parameter.
  std::shared_ptr<kstone::RobotInterface> robot;
  std::optional<kstone::DebugServerOptions> debug;
};
struct JointValues {
  std::vector<std::string> names;
  std::vector<double> positions, velocities;
};
inline bool supported_ros_module(std::string_view type) {
  return type=="frame_target" || type=="chassis" || type=="joint_target";
}
// Immutable discovery metadata; no Module pointers escape the engine owner.
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
struct Outcome {
  std::uint64_t sequence{};
  bool applied{};
  Text<192> error;
};
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
class Controller {
public:
  Controller(const std::string &file, Options options = {});
  ~Controller();
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;
  void start();
  void stop() noexcept;
  // Single serialized ROS producer/consumer. Never calls Engine::step.
  bool poll();
  const auto &model() const { return model_; }
  const auto &modules() const { return metadata_; }
  const kstone::PortInfo &port(const std::string &, const std::string &) const;
  kstone::PortValue read(const std::string &, const std::string &) const;
  std::uint64_t write(const std::string &, const std::string &, const kstone::PortValue &);
  std::uint64_t enable(const std::string &, bool);
  std::uint64_t home(const std::string &);
  std::uint64_t reset_target(const std::string &);
  std::uint64_t reset_home();
  std::uint64_t recover();
  // Names outside every joint_target chain are skipped and, if given, appended to ignored.
  std::uint64_t joint_targets(const std::vector<std::string> &, const std::vector<double> &, std::vector<std::string> *ignored = nullptr);
  std::uint64_t feedback_joints(const std::vector<std::string> &, const std::vector<double> &, double age_s = 0);
  std::uint64_t feedback_base(const Eigen::Isometry3d &, double age_s = 0);
  // Chassis control-center pose (center_in_root applied), as chassis drivers report odometry.
  std::uint64_t feedback_center(const Eigen::Isometry3d &, double age_s = 0);
  double dt() const { return snapshot().dt; }
  double now() const;
  const auto &chassis() const { return chassis_id_; }
  int debug_port() const { return debug_ ? debug_->port() : 0; }
  bool has_root() const { return root_ != nullptr; }
  bool has_joints() const { return !joints_.empty(); }
  const auto &q() const { return snapshots_.read().q; }
  const auto &measured() const { return snapshots_.read().engine.state.q; }
  const auto &command_q() const { return snapshots_.read().engine.command.q; }
  const auto &velocity() const { return snapshots_.read().engine.command.v; }
  Eigen::Isometry3d pose(const std::string &, const std::string &) const;
  Eigen::Isometry3d home_pose(const std::string &, const std::string &) const;
  Eigen::Isometry3d base_pose() const;
  Eigen::Isometry3d chassis_pose() const { return snapshot().chassis_pose; }
  kstone::Twist2D base_velocity() const;
  JointValues joint_state(const Eigen::VectorXd &, const Eigen::VectorXd * = nullptr) const;
  // Last command of the joints whose feedback group is attached; a detached group (e.g. an unplugged hand)
  // gets no output while the other groups keep running.
  JointValues joint_command() const;
  Node discovery() const;
  Node state() const;
  const Snapshot &snapshot() const { return snapshots_.read(); }
  const auto &logs() const { return logs_; }
  bool written{}, fresh{};
  // Driver output gates: fresh feedback has been synced into the model.
  bool joints_attached{}, base_attached{};
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
  struct JointBatch {
    std::uint64_t sequence{}, generation{};
    double receipt{};
    std::vector<kstone::PortValue> values;  // Per input: positions of `names`, a subset of the port labels.
    std::vector<std::vector<std::string>> names;
    std::vector<unsigned char> touched;
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
  void start_home(std::size_t, double);
  Eigen::Isometry3d port_pose(std::size_t, std::size_t);
};
} // namespace kstone_ros
