#include "kstone_ros/controller.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <regex>

namespace kstone_ros {
namespace {
void positive(double value, const char *name) {
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
}
Node numbers(const Eigen::VectorXd &v) {
  auto n = Node::list();
  for (double x : v)
    n.append(x);
  return n;
}
Node strings(const std::vector<std::string> &v) {
  auto n = Node::list();
  for (const auto &x : v)
    n.append(x);
  return n;
}
Node port_node(const kstone::PortInfo &i, const kstone::PortValue &v) {
  switch (i.type) {
  case kstone::PortType::pose:
    return pose_node(v.pose);
  case kstone::PortType::number:
    return v.number;
  case kstone::PortType::flag:
    return v.flag;
  case kstone::PortType::vector:
    return numbers(v.vector);
  }
  return {};
}
Node port_info(const kstone::PortInfo &p) {
  auto n = Node::map({{"name", p.name}, {"type", kstone::to_string(p.type)}});
  if (p.type == kstone::PortType::vector)
    n.set("labels", strings(p.labels));
  if (p.lower.size())
    n.set("lower", numbers(p.lower));
  if (p.upper.size())
    n.set("upper", numbers(p.upper));
  if (!p.frame.empty())
    n.set("frame", p.frame);
  if (!p.reference.empty())
    n.set("reference", p.reference);
  if (p.timeout > 0)
    n.set("timeout", p.timeout);
  return n;
}
void check_joint_input(const std::vector<std::string> &names,
                       const std::vector<double> &positions) {
  if (names.empty() || names.size() != positions.size() ||
      std::set<std::string>(names.begin(), names.end()).size() != names.size())
    throw std::invalid_argument(
        "joint names must be nonempty, unique and match positions");
  for (double p : positions)
    if (!std::isfinite(p))
      throw std::invalid_argument("non-finite joint position");
}
bool motion(const kstone::Module &m) {
  const std::string t = m.type();
  return t == "chassis" || t == "frame_target" || t == "joint_target" ||
         t == "gaze";
}
Eigen::Isometry3d config_pose(const Node &node) {
  std::vector<double> t{0, 0, 0}, r{1, 0, 0, 0};
  auto list = [](const Node *n, std::vector<double> &out) {
    if (!n)
      return;
    if (n->kind() != Node::Kind::list || n->size() != out.size())
      throw std::invalid_argument("invalid pose config");
    for (size_t i = 0; i < out.size(); ++i) {
      if (n->items()[i].kind() != Node::Kind::number)
        throw std::invalid_argument("invalid pose config");
      out[i] = n->items()[i].as_number();
    }
  };
  list(node.find("position"), t);
  list(node.find("quaternion"), r);
  auto p = make_pose(t, r);
  if (auto angles = node.find("rpy")) {
    std::vector<double> a(3);
    list(angles, a);
    for (double x : a)
      if (!std::isfinite(x))
        throw std::invalid_argument("non-finite rpy");
    p.linear() = (Eigen::AngleAxisd(a[2], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(a[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(a[0], Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
  }
  return p;
}
} // namespace
class Controller::FeedbackRobot : public kstone::RobotInterface {
public:
  Eigen::VectorXd measured, velocity;
  std::int64_t stamp{-1};
  std::vector<std::int64_t> groups;  // per engine feedback group
  const kstone::JointInfo *root{nullptr};
  bool chassis{false};
  kstone::Status read(kstone::RobotState &state) override {
    state.q = measured; state.v = velocity; state.stamp_ns = stamp;
    std::copy_n(groups.begin(), std::min(groups.size(), state.group_stamps_ns.size()), state.group_stamps_ns.begin());
    return {};
  }
  kstone::Status write(const kstone::RobotCommand &command) override {
    if (root && !chassis && command.v.segment(root->idx_v, 6).cwiseAbs().maxCoeff() > 1e-8)
      return kstone::make_error(kstone::ErrorCode::unsupported, "floating-root actuation requires a chassis or kstone driver");
    return {};
  }
};
class Controller::Observer final : public kstone::EngineObserver {
  Controller &owner_;
public:
  explicit Observer(Controller &owner) : owner_(owner) {}
  void before_step(kstone::Engine &e) override { owner_.before(e); }
  void after_step(const kstone::Engine &e) override { owner_.after(e); }
};
Controller::Controller(const std::string &file, Options options)
    : options_(std::move(options)), context_(kstone::Context::defaults()) {
  positive(options_.command_timeout_s, "command_timeout_s");
  positive(options_.feedback_timeout_s, "feedback_timeout_s");
  if (options_.clock)
    context_.clock = options_.clock;
  logs_ = std::make_shared<BufferedLog>();
  context_.log = kstone::Logger(logs_, kstone::LogLevel::info, context_.clock);
  if (!std::isfinite(options_.duration_s) || options_.duration_s < 0)
    throw std::invalid_argument("duration must be finite and nonnegative");
  auto yaml = kstone::YamlOptions::files();
  yaml.vars = options_.variables;
  yaml.resources = &context_.resources;
  const auto path = std::filesystem::canonical(file);
  auto config = take(kstone::load_yaml(path, yaml));
  if (!options_.robot) {
    if (!config.find("model"))
      throw std::invalid_argument("missing model");
    // Bootstrap the model and initial posture once using a loopback driver, not a plugin.
    auto bootstrap = config;
    auto robot = Node::map({{"type", "loopback"}});
    if (auto configured = config.find("robot"))
      if (auto initial = configured->find("initial"))
        robot.set("initial", *initial);
    bootstrap.set("robot", robot);
    bootstrap.set("modules", Node::map());
    auto initial_engine =
        take(kstone::build_engine(bootstrap, path.parent_path(), {}, context_));
    feedback_ = std::make_shared<FeedbackRobot>();
    feedback_->measured = initial_engine.state().q;
    feedback_->velocity =
        Eigen::VectorXd::Zero(initial_engine.modules().model()->nv());
    // The free root attaches on odometry: its own feedback group unless the configuration places it.
    const auto &bm = *initial_engine.modules().model();
    for (const auto &j : bm.joints()) {
      if (j.nq != 7 || j.nv != 6) continue;
      const auto layout = take(kstone::resolve_feedback_groups(bm, initial_engine.options().feedback_groups));
      if (layout.names[layout.joint_group[j.id]] == "default") {
        auto groups = config.find("feedback_groups") ? *config.find("feedback_groups") : kstone::ConfigNode::list();
        groups.append(kstone::ConfigNode::map({{"name", "base"}, {"joints", kstone::ConfigNode::list({kstone::ConfigNode(j.name)})}}));
        config.set("feedback_groups", std::move(groups));
      }
      break;
    }
    config.set("feedback_timeout_s", options_.feedback_timeout_s);
    // Feedback safety cannot be disabled by a solver configuration.
    auto safety = config.find("safety") ? *config.find("safety") : Node::map();
    safety.set("mode", "reject");
    config.set("safety", std::move(safety));
  }
  if (options_.realtime) config.set("realtime", *options_.realtime);
  kstone::EngineLoadOptions load;
  load.robot = feedback_ ? feedback_ : options_.robot;
  engine_ = std::make_unique<kstone::Engine>(
      take(kstone::build_engine(config, path.parent_path(), load, context_)));
  model_ = engine_->modules().model();
  q_ = engine_->state().q;
  measured_ = q_;
  for (const auto &j : model()->joints()) {
    if (j.nv == 1 && (j.nq == 1 || j.nq == 2))
      joints_.push_back(j);
    else if (j.nq == 7 && j.nv == 6) {
      if (root_)
        throw std::invalid_argument("multiple free roots are unsupported");
      root_ = &j;
    } else if (feedback_ && j.nv > 0)
      throw std::invalid_argument("feedback cannot represent joint " + j.name);
  }
  const auto &locked = engine_->modules().solver().options().locked;
  std::set<std::string> followers;
  for (const auto &m : model()->mimics()) followers.insert(m.follower);
  // Locked joints are held and mimic followers derived, so neither gates attach.
  for (const auto &j : joints_) required_.push_back(!locked.test(std::size_t(j.idx_v)) && !followers.count(j.name));
  workspace_ = std::make_unique<kstone::Workspace>(model());
  require(workspace_->update(q_));
  std::regex token("^[A-Za-z_][A-Za-z0-9_]*$");
  std::regex port_path("^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$");
  for (const auto &m : engine_->modules().modules()) {
    if (supported_ros_module(m->type()) && !std::regex_match(m->id(), token))
      throw std::invalid_argument("module id is not a ROS topic token: " +
                                  m->id());
    if (std::string(m->type()) == "chassis") {
      if (chassis_)
        throw std::invalid_argument(
            "multiple chassis modules have ambiguous root ownership");
      chassis_ = std::dynamic_pointer_cast<kstone::ChassisModule>(m);
      chassis_id_ = m->id();
      if (!chassis_)
        throw std::invalid_argument("unsupported chassis implementation");
      if (auto ms = config.find("modules"))
        if (auto mc = ms->find(m->id()))
          if (auto center = mc->find("center_in_root"))
            center_in_root_ = config_pose(*center);
    }
    for (const auto &p : m->ports()) {
      if (!supported_ros_module(m->type())) continue;
      if (!std::regex_match(p.name, port_path))
        throw std::invalid_argument("invalid port name: " + p.name);
      if (p.type == kstone::PortType::pose && !p.frame.empty()) {
        Eigen::Isometry3d anchor, ref;
        require(workspace_->frame_pose(take(model()->frame_id(p.frame)), anchor));
        if (!p.reference.empty()) {
          require(workspace_->frame_pose(take(model()->frame_id(p.reference)), ref));
          anchor = ref.inverse() * anchor;
        }
        if (!p.reference.empty())
          if (auto ms = config.find("modules"))
            if (auto mc = ms->find(m->id()))
              if (auto h = mc->find("home"))
                anchor = config_pose(*h);
        homes_[{m->id(), p.name}] = anchor;
      }
    }
  }
  if (chassis_ && !root_)
    throw std::invalid_argument("chassis requires a free root");
  if (!options_.chassis_module.empty() &&
      options_.chassis_module != chassis_id_)
    throw std::invalid_argument(
        "chassis_module must name the root-owning chassis module");
  if (feedback_) {
    feedback_->root = root_;
    feedback_->chassis = bool(chassis_);
    const auto layout = take(kstone::resolve_feedback_groups(*model(), engine_->options().feedback_groups));
    feedback_->groups.assign(layout.names.size(), -1);
    group_oldest_.assign(layout.names.size(), -1);
    joint_groups_.assign(layout.names.size(), 0);
    for (const auto &j : joints_) { joint_group_.push_back(layout.joint_group[j.id]); joint_groups_[joint_group_.back()] = 1; }
    if (root_) base_group_ = layout.joint_group[root_->id];
  }
  for (const auto &m : engine_->modules().modules()) {
    auto info = std::make_shared<ModuleInfo>();
    info->module_id = m->id(); info->module_type = m->type();
    info->can_home = m->homeable(); info->inputs = m->ports(); info->outputs = m->outputs();
    if (m->type()==std::string_view("chassis")) {
      const auto &cc=chassis_->controller()->config();
      info->chassis_omni=cc.kinematics==kstone::ChassisKinematics::omni;
      info->chassis_frame=cc.base_frame;
    }
    metadata_.push_back(info);
    for (std::size_t pi = 0; pi < info->inputs.size(); ++pi) {
      auto input = std::make_unique<Latest<Input>>();
      for (auto &slot : input->slots) require(m->read_port(pi, slot.value));
      addresses_.push_back({metadata_.size()-1, pi});
      accepted_values_.push_back(input->slots[0].value);
      inputs_.push_back(std::move(input));
    }
  }
  accepted_sequences_.assign(inputs_.size(),0); refresh_.assign(inputs_.size(),0);
  for(std::size_t i=0;i<addresses_.size();++i) {
    const auto [mi,pi]=addresses_[i];
    if(metadata_[mi]->type()!="joint_target" || metadata_[mi]->inputs[pi].name!="target") continue;
    const auto &labels=metadata_[mi]->inputs[pi].labels;
    for(std::size_t j=0;j<labels.size();++j)
      if(!joint_routes_.emplace(labels[j],std::make_pair(i,j)).second) ambiguous_joints_.insert(labels[j]);
  }
  for(auto &slot:joint_targets_.slots) {
    slot.values=accepted_values_; slot.names.resize(inputs_.size()); slot.touched.assign(inputs_.size(),0);
  }
  expired_.assign(metadata_.size(), 0); needs_command_.assign(metadata_.size(),0);
  stamps_.assign(joints_.size()+1, -1);
  joint_candidate_.q = measured_; joint_candidate_.stamps = stamps_;
  for (auto &slot : joint_input_.slots) slot = joint_candidate_;
  for (auto &slot : snapshots_.slots) prepare(slot);
  engine_->add_observer(std::make_shared<Observer>(*this));
  if (options_.debug) debug_ = take(kstone::DebugServer::start(*engine_, *options_.debug));
  capture(false); snapshots_.acquire();
  fresh = true;
  joints_attached = !feedback_ || joints_synced();
  base_attached = feedback_ && base_synced();
}
void validate_port(const kstone::PortInfo &i, const kstone::PortValue &v) {
  if (i.type == kstone::PortType::pose) {
    if (!v.pose.matrix().allFinite() ||
        !v.pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1),
                                         1e-6) ||
        !v.pose.linear().transpose().isApprox(v.pose.linear().inverse(),
                                              1e-6) ||
        std::abs(v.pose.linear().determinant() - 1) > 1e-6)
      throw std::invalid_argument("invalid rigid pose");
  } else if (i.type == kstone::PortType::number ||
             i.type == kstone::PortType::vector) {
    Eigen::VectorXd a = i.type == kstone::PortType::number
                            ? Eigen::VectorXd::Constant(1, v.number)
                            : v.vector;
    if (a.size() != i.size() || !a.allFinite())
      throw std::invalid_argument("port requires matching finite numbers");
  }
}
bool within_bounds(const kstone::PortInfo &i, const kstone::PortValue &v) {
  if (i.type != kstone::PortType::number && i.type != kstone::PortType::vector) return true;
  Eigen::VectorXd a = i.type == kstone::PortType::number ? Eigen::VectorXd::Constant(1, v.number) : v.vector;
  if (a.size() != i.size()) return false;
  for (Eigen::Index k = 0; k < a.size(); ++k) {
    if (i.lower.size() && i.upper.size() && i.lower[k] == i.upper[k]) continue;
    if ((i.lower.size() && a[k] < i.lower[k]) || (i.upper.size() && a[k] > i.upper[k])) return false;
  }
  return true;
}

Controller::~Controller() { stop(); }
void Controller::start() {
  int expected = 0;
  if (!lifecycle_.compare_exchange_strong(expected, 1)) throw std::logic_error("controller already started/stopped");
  if (engine_->options().realtime) {
    auto raised = kstone::elevate_thread_limits(engine_->options().thread, context_.log);
    if (!raised) context_.log.log(kstone::LogLevel::warn, "kstone_ros", raised.error().message + "; continuing");
  }
  try { thread_ = std::thread([this] { owner_run(); }); }
  catch (...) { lifecycle_ = 3; throw; }
}
void Controller::stop() noexcept {
  stop_.store(true);
  if (thread_.joinable()) thread_.join();
  if (debug_) debug_->stop();
  lifecycle_.store(3);
}
double Controller::now() const { return double(context_.clock->now_ns()) * 1e-9; }
std::size_t Controller::module_index(const std::string &id) const {
  for (std::size_t i=0; i<metadata_.size(); ++i) if (metadata_[i]->id()==id) return i;
  throw std::invalid_argument("unknown module: " + id);
}
std::size_t Controller::input_index(const std::string &id, const std::string &name) const {
  const auto mi=module_index(id);
  for (std::size_t i=0; i<addresses_.size(); ++i)
    if (addresses_[i].first==mi && metadata_[mi]->inputs[addresses_[i].second].name==name) return i;
  throw std::invalid_argument("unknown port: " + name);
}
const kstone::PortInfo &Controller::port(const std::string &id,const std::string &name) const {
  const auto a=addresses_[input_index(id,name)]; return metadata_[a.first]->inputs[a.second];
}
kstone::PortValue Controller::read(const std::string &id,const std::string &name) const {
  return snapshot().accepted[input_index(id,name)];
}
std::uint64_t Controller::write(const std::string &id,const std::string &name,const kstone::PortValue &value) {
  const auto i=input_index(id,name); validate_port(port(id,name),value);
  return submit(i,value,generation_.load());
}
std::uint64_t Controller::submit(std::size_t i,const kstone::PortValue &value,std::uint64_t generation) {
  if (lifecycle_!=1 && lifecycle_!=0) { ++rejected_; throw std::runtime_error("owner not running"); }
  auto &slot=inputs_[i]->write();
  slot.value=value; slot.sequence=++next_sequence_; slot.generation=generation; slot.receipt=now();
  inputs_[i]->publish(); ++coalesced_; return next_sequence_;
}
std::uint64_t Controller::enqueue(Operation op,std::size_t mi,bool enabled) {
  if (lifecycle_!=1 && lifecycle_!=0) { ++rejected_; throw std::runtime_error("owner not running"); }
  Command c{op,mi,enabled,++next_sequence_,generation_.load(),now()};
  if (!commands_.push(c)) { ++rejected_; throw std::runtime_error("request queue full (64)"); }
  return c.sequence;
}
std::uint64_t Controller::enable(const std::string &id,bool value) { return enqueue(Operation::enable,module_index(id),value); }
std::uint64_t Controller::home(const std::string &id) {
  auto i=module_index(id); if (!metadata_[i]->homeable()) throw std::invalid_argument("module is not homeable");
  return enqueue(Operation::home,i);
}
std::uint64_t Controller::reset_target(const std::string &id) {
  input_index(id,"target"); return enqueue(Operation::reset_target,module_index(id));
}
std::uint64_t Controller::reset_home() { return enqueue(Operation::reset_home); }
std::uint64_t Controller::recover() { return enqueue(Operation::recover); }
void Controller::outcome(std::uint64_t id,bool applied,std::string_view error) {
  auto &o=outcomes_[outcome_count_++ % outcomes_.size()]; o.sequence=id; o.applied=applied; o.error.set(error);
}
void Controller::clear_motion() {
  for (std::size_t i=0;i<metadata_.size();++i) {
    auto &m=*engine_->modules().modules()[i];
    if (!motion(m)) continue;
    if (m.enabled() || expired_[i]) expired_[i]=1;
    m.set_enabled(false); needs_command_[i]=1;
  }
  if (chassis_) {
    kstone::PortValue zero; zero.vector=Eigen::Vector3d::Zero();
    require(chassis_->write_port(chassis_->port_index("velocity"),zero));
    chassis_->controller()->clear_velocity();
  }
  engine_->modules().resynchronize(true);
}
void Controller::write_port(std::size_t mi,std::size_t pi,const kstone::PortValue &value,double receipt,double application_time) {
  auto &m=*engine_->modules().modules()[mi];
  const auto timeout=metadata_[mi]->inputs[pi].timeout;
  const double limit=timeout>0?std::min(timeout,options_.command_timeout_s):options_.command_timeout_s;
  if (motion(m) && (application_time<0?now():application_time)-receipt>limit) throw std::runtime_error("command expired before application");
  if (!m.enabled() && !expired_[mi]) throw std::runtime_error("module manually disabled");
  require(m.write_port(pi,value));
  const auto ai=input_index(metadata_[mi]->id(),metadata_[mi]->inputs[pi].name);
  accepted_values_[ai]=value; refresh_[ai]=0;
  needs_command_[mi]=0;
  if (expired_[mi]) { m.set_enabled(true); expired_[mi]=0; }
}
void Controller::apply_joint_targets(const JointBatch &batch) {
  try {
    if(latched_ || batch.generation!=owner_generation_)
      throw std::runtime_error("discarded across fault/recovery");
    const double t=now(); bool pending=false;
    // Check every non-superseded module before writing anything or renewing leases.
    // joint_target set_target gates are immutable validation, lock and home.
    for(std::size_t i=0;i<batch.touched.size();++i) {
      if(!batch.touched[i] || batch.sequence<=accepted_sequences_[i]) continue;
      pending=true; const auto [mi,pi]=addresses_[i];
      const auto &m=*engine_->modules().modules()[mi];
      const auto &port=metadata_[mi]->inputs[pi];
      const double limit=port.timeout>0?std::min(port.timeout,options_.command_timeout_s):options_.command_timeout_s;
      if(t-batch.receipt>limit) throw std::runtime_error("aggregate expired before application");
      if(!m.enabled() && !expired_[mi]) throw std::runtime_error("aggregate module manually disabled");
      const std::string_view home=m.home_state()?m.home_state():"";
      if(m.locked_by() || home=="waiting" || home=="executing")
        throw std::runtime_error("aggregate module locked or homing");
    }
    if(!pending) throw std::runtime_error("superseded aggregate");
    // Owner state cannot change between preflight and these built-in writes.
    // Use one application time; no mid-batch expiry and no ROS synchronization.
    for(std::size_t i=0;i<batch.touched.size();++i) {
      if(!batch.touched[i] || batch.sequence<=accepted_sequences_[i]) continue;
      const auto [mi,pi]=addresses_[i]; auto &m=*engine_->modules().modules()[mi];
      require(m.write_port(pi,batch.names[i],batch.values[i].vector));
      require(m.read_port(pi,accepted_values_[i]));
      refresh_[i]=!m.enabled() || !m.running();
      needs_command_[mi]=0;
      if (expired_[mi]) { m.set_enabled(true); expired_[mi]=0; }
      accepted_sequences_[i]=batch.sequence;
    }
    outcome(batch.sequence,true);
  } catch(const std::exception &err) { outcome(batch.sequence,false,err.what()); }
}
void Controller::start_home(std::size_t mi,double) {
  auto &m=*engine_->modules().modules()[mi]; require(m.request_home());
  m.set_enabled(true); expired_[mi]=0; needs_command_[mi]=0;
}
void Controller::apply(const Command &c) {
  try {
    if (c.op==Operation::recover) {
      clear_motion(); owner_generation_=generation_.fetch_add(1)+1;
      latched_=false; owner_fault_.set(""); last_owner_time_=-1;
      seen_failed_=engine_->stats().failed_cycles;
      engine_->set_paused(true);
    } else {
      if ((c.generation!=owner_generation_ || latched_) && !(c.op==Operation::enable && !c.enabled)) throw std::runtime_error("discarded across fault/recovery");
      switch (c.op) {
      case Operation::enable: {
        auto &m=*engine_->modules().modules()[c.module];
        if(c.enabled && needs_command_[c.module]) {
          // Authorize a fresh command without enabling a pre-fault target. This
          // also permits enable after a fail-safe manual disable during a fault.
          m.set_enabled(false); expired_[c.module]=1;
        } else { m.set_enabled(c.enabled); expired_[c.module]=0; }
        break;
      }
      case Operation::home: start_home(c.module,c.receipt); break;
      case Operation::reset_target: {
        auto &m=*engine_->modules().modules()[c.module];
        auto pi=m.port_index("target"); kstone::PortValue v; v.pose=port_pose(c.module,pi);
        write_port(c.module,pi,v,c.receipt); break;
      }
      case Operation::reset_home:
      {
        auto &set=engine_->modules(); const auto &mods=set.modules();
        const auto homing=[&](std::size_t i) { const char *h=mods[i]->home_state(); return h && std::strcmp(h,"idle")!=0; };
        std::vector<char> before(mods.size());
        // Fault-cleared modules count as enabled so home_all can home them; untouched ones stay disabled.
        for (std::size_t i=0;i<mods.size();++i) { before[i]=homing(i); if(expired_[i]) mods[i]->set_enabled(true); }
        const auto s=set.home_all();
        for (std::size_t i=0;i<mods.size();++i) {
          if(!before[i] && homing(i)) { expired_[i]=0; needs_command_[i]=0; }
          else if(expired_[i]) mods[i]->set_enabled(false);
        }
        require(s);
      }
        if (chassis_) { kstone::PortValue z; z.vector=Eigen::Vector3d::Zero();
          write_port(module_index(chassis_id_),chassis_->port_index("velocity"),z,c.receipt); }
        break;
      default: break;
      }
    }
    outcome(c.sequence,true);
  } catch (const std::exception &e) { outcome(c.sequence,false,e.what()); }
}
static bool recent(double stamp,double t,double timeout) { return stamp>=0 && t>=stamp && t-stamp<=timeout; }
void Controller::before(kstone::Engine &e) {
  const double t=now();
  // The gap since the last cycle was slept with the previous period; a runtime rate change must not look like a stall.
  owner_limit_=scheduler_limit(std::max(owner_dt_,e.dt())); owner_dt_=e.dt(); owner_elapsed_+=owner_dt_;
  if (!latched_) {
    char reason[96]{};
    if (e.stats().failed_cycles!=seen_failed_) std::snprintf(reason,sizeof reason,"cycle_error: engine failed");
    else if (last_owner_time_>=0 && t<last_owner_time_) std::snprintf(reason,sizeof reason,"scheduler_discontinuity: clock went backwards");
    else if (last_owner_time_>=0 && t-last_owner_time_>owner_limit_)
      std::snprintf(reason,sizeof reason,"scheduler_discontinuity: gap %.3f s > %.3f s",t-last_owner_time_,owner_limit_);
    if (reason[0]) { latched_=true; owner_fault_.set(reason); clear_motion(); owner_generation_=generation_.fetch_add(1)+1; }
  }
  seen_failed_=e.stats().failed_cycles; last_owner_time_=t;
  // Fixed work per cycle, independent of a producer flooding requests.
  Command c; for (std::size_t i=0;i<64 && commands_.pop(c);++i) apply(c);
  if (joint_input_.acquire()) {
    const auto &f=joint_input_.read();
    if (f.sequence!=applied_joints_) {
      applied_joints_=f.sequence;
      // Feedback is sensing, not motion; keep full accumulated per-joint stamps.
      if (root_) { auto root = measured_.segment(root_->idx_q,7).eval(); measured_=f.q; measured_.segment(root_->idx_q,7)=root; } else measured_=f.q;
      std::copy_n(f.stamps.begin(),joints_.size(),stamps_.begin()); outcome(f.sequence,true);
    }
  }
  if (base_input_.acquire()) {
    const auto &b=base_input_.read();
    if (b.sequence!=applied_base_) {
      applied_base_=b.sequence;
      try {
        require(model_->set_root_pose(measured_,b.value.pose));
        if (chassis_) {
          base_feedback_pose_=b.value.pose*center_in_root_;
          base_receipt_=b.receipt; base_pending_=true;
        }
        stamps_.back()=b.receipt; outcome(b.sequence,true);
      } catch (const std::exception &err) { outcome(b.sequence,false,err.what()); }
    }
  }
  if(joint_targets_.acquire()) apply_joint_targets(joint_targets_.read());
  for (std::size_t i=0;i<inputs_.size();++i) if(inputs_[i]->acquire()) {
    const auto &input=inputs_[i]->read();
    try {
      if (latched_ || input.generation!=owner_generation_) throw std::runtime_error("discarded across fault/recovery");
      if(input.sequence<=accepted_sequences_[i]) throw std::runtime_error("superseded input");
      write_port(addresses_[i].first,addresses_[i].second,input.value,input.receipt);
      accepted_sequences_[i]=input.sequence;
      outcome(input.sequence,true);
    } catch(const std::exception &err) { outcome(input.sequence,false,err.what()); }
  }
  if (feedback_) {
    // Per group, the oldest required stamp, so the engine's own timeout decides each group's attach.
    std::fill(group_oldest_.begin(),group_oldest_.end(),t);
    for (std::size_t i=0;i<joints_.size();++i) if (required_[i]) group_oldest_[joint_group_[i]]=std::min(group_oldest_[joint_group_[i]],stamps_[i]);
    if (root_) group_oldest_[base_group_]=std::min(group_oldest_[base_group_],stamps_.back());
    for (std::size_t g=0;g<group_oldest_.size();++g)
      feedback_->groups[g]=group_oldest_[g]<0?-1:std::int64_t(std::llround(group_oldest_[g]*1e9));
    const bool bf=root_ && recent(group_oldest_[base_group_],t,options_.feedback_timeout_s);
    if (base_pending_ && bf && !latched_) {
      // Exact last chassis timestamp: a reconstructed time can step past the next cycle.
      const double module_time=chassis_->output().timestamp;
      const auto &p=base_feedback_pose_;
      const bool accepted=take(chassis_->controller()->submit_odometry(
        {p.translation().x(),p.translation().y(),std::atan2(p.linear()(1,0),p.linear()(0,0))},
        module_time-(t-base_receipt_),module_time));
      if (accepted || t-base_receipt_>options_.feedback_timeout_s) base_pending_=false;
    }
    auto &m=feedback_->measured; m=e.ideal();
    double oldest=t;
    for (std::size_t i=0;i<joints_.size();++i) if (required_[i]) oldest=std::min(oldest,stamps_[i]);
    // Stale groups report the open-loop state, not frozen measurements.
    for (std::size_t i=0;i<joints_.size();++i)
      if (required_[i] && recent(group_oldest_[joint_group_[i]],t,options_.feedback_timeout_s))
        m.segment(joints_[i].idx_q,joints_[i].nq)=measured_.segment(joints_[i].idx_q,joints_[i].nq);
    if (bf) m.segment(root_->idx_q,7)=measured_.segment(root_->idx_q,7);
    feedback_->stamp=oldest<0?-1:std::int64_t(std::llround(oldest*1e9));
    // The engine resyncs the chassis on reattach; a velocity accepted before the dropout must not resume.
    if (base_fresh_ && !bf && chassis_) {
      kstone::PortValue zero; zero.vector=Eigen::Vector3d::Zero();
      require(chassis_->write_port(chassis_->port_index("velocity"),zero));
      chassis_->controller()->clear_velocity();
    }
    base_fresh_=bf;
  }
  e.set_paused(latched_);
  if (!latched_) owner_fault_.set("");
  // Failed cycles have no after_step: publish suppression before the next read.
  if (latched_) capture(false);
}
void Controller::after(const kstone::Engine &e) {
  q_=e.state().q;
  if (feedback_ && chassis_ && !e.paused()) q_.segment(root_->idx_q,7)=e.ideal().segment(root_->idx_q,7);
  for(std::size_t i=0;i<refresh_.size();++i) {
    if(!refresh_[i]) continue;
    const auto [mi,pi]=addresses_[i]; const auto &m=*e.modules().modules()[mi];
    if(m.running()) { (void)m.read_port(pi,accepted_values_[i]); refresh_[i]=0; }
  }
  const double t=now();
  // A driver/solve can overrun after before_step. Never export its result as safe.
  if (!latched_ && last_owner_time_>=0 && t-last_owner_time_>owner_limit_) {
    char reason[96]; std::snprintf(reason,sizeof reason,"scheduler_stall: cycle %.3f s > %.3f s",t-last_owner_time_,owner_limit_);
    latched_=true; owner_fault_.set(reason);
    clear_motion(); owner_generation_=generation_.fetch_add(1)+1; engine_->set_paused(true);
  } else if (!latched_ && !e.fresh() && !feedback_) {
    latched_=true; owner_fault_.set("driver_stale_feedback");
    clear_motion(); owner_generation_=generation_.fetch_add(1)+1; engine_->set_paused(true);
  }
  // A feedback group attached this cycle: earlier producer requests are void.
  if (e.stats().attaches!=seen_attaches_) owner_generation_=generation_.fetch_add(1)+1;
  seen_attaches_=e.stats().attaches;
  bool safe=!latched_ && !e.paused() && e.written() && !e.result().held && e.command().q.allFinite() && e.command().v.allFinite();
  if (!latched_ && owner_fault_.bytes[0]==0 && !safe && !e.paused()) owner_fault_.set("held_or_stale");
  capture(safe);
}
void Controller::owner_run() noexcept {
  try {
    // Sum of attempted cycle periods, so failed cycles count and rate changes are exact.
    const double start=owner_elapsed_;
    while(!stop_) {
      double remaining=options_.duration_s>0?options_.duration_s-(owner_elapsed_-start):0;
      if(options_.duration_s>0 && remaining<0.5*engine_->dt()) break;
      auto status=engine_->run(remaining,&stop_);
      if(status) break;
      latched_=true; owner_fault_.set(status.error().message); clear_motion();
      owner_generation_=generation_.fetch_add(1)+1; seen_failed_=engine_->stats().failed_cycles;
      engine_->set_paused(true); capture(false);
    }
    clear_motion(); engine_->set_paused(true);
    owner_fault_.set(stop_?"stopped":"duration_complete"); capture(false); lifecycle_=3;
  } catch(const std::exception &e) {
    owner_fault_.set(e.what()); latched_=true; engine_->set_paused(true);
    try { clear_motion(); capture(false); } catch (...) {}
    lifecycle_=2;
  } catch (...) { lifecycle_=2; }
}
void Controller::prepare(Snapshot &s) {
  kstone::take_snapshot(*engine_,s.engine); s.q=q_; s.accepted=accepted_values_;
  s.outputs.resize(metadata_.size()); s.poses.resize(metadata_.size());
  s.expired=expired_; s.home_states.resize(metadata_.size());
  for(std::size_t i=0;i<metadata_.size();++i) {
    s.outputs[i].resize(metadata_[i]->outputs.size()); s.poses[i].resize(metadata_[i]->inputs.size(),Eigen::Isometry3d::Identity());
  }
}
Eigen::Isometry3d Controller::port_pose(std::size_t mi,std::size_t pi) {
  const auto &p=metadata_[mi]->inputs[pi];
  if(p.type!=kstone::PortType::pose || p.frame.empty()) throw std::invalid_argument("pose target requires a frame");
  require(workspace_->update(q_)); Eigen::Isometry3d result,ref;
  require(workspace_->frame_pose(take(model_->frame_id(p.frame)),result));
  if(!p.reference.empty()) { require(workspace_->frame_pose(take(model_->frame_id(p.reference)),ref)); result=ref.inverse()*result; }
  return result;
}
void Controller::capture(bool safe) {
  auto &s=snapshots_.write(); kstone::take_snapshot(*engine_,s.engine);
  s.q=q_; s.accepted=accepted_values_; s.expired=expired_; s.safe=safe; s.joints_attached=!feedback_ || joints_synced(); s.base_attached=!feedback_ || base_synced(); s.stamp=now(); s.dt=engine_->dt(); s.generation=owner_generation_; s.fault=owner_fault_;
  s.outcomes=outcomes_; s.outcome_count=outcome_count_; s.base_velocity={};
  if(chassis_) s.chassis_pose=kstone::chassis_root_pose(chassis_->output().pose);
  s.engine.written=safe;
  if(!safe) s.engine.command.v.setZero();
  if(safe && chassis_ && chassis_->enabled() && chassis_->output().state!=kstone::ChassisState::fault)
    s.base_velocity=chassis_->output().velocity;
  require(workspace_->update(q_));
  for(std::size_t i=0;i<metadata_.size();++i) {
    auto &m=*engine_->modules().modules()[i];
    s.home_states[i].set(m.home_state()?m.home_state():"");
    // Do not export the engine's borrowed home_state pointer to another thread.
    s.engine.modules[i].home_state=nullptr;
    require(m.update_outputs(*workspace_));
    for(std::size_t j=0;j<s.outputs[i].size();++j) require(m.read_output(j,s.outputs[i][j]));
    for(std::size_t j=0;j<metadata_[i]->inputs.size();++j) {
      const auto &p=metadata_[i]->inputs[j]; if(supported_ros_module(metadata_[i]->type()) && p.type==kstone::PortType::pose && !p.frame.empty()) s.poses[i][j]=port_pose(i,j);
    }
  }
  snapshots_.publish();
}
bool Controller::poll() {
  snapshots_.acquire(); const auto &s=snapshot();
  fresh=s.engine.fresh; fault=s.fault.c_str(); joints_attached=s.joints_attached; base_attached=s.base_attached && has_root(); quality=kstone::to_string(s.engine.quality);
  written=s.safe && lifecycle_==1 && now()-s.stamp<=scheduler_limit(s.dt) && now()>=s.stamp;
  return written;
}
std::uint64_t Controller::joint_targets(const std::vector<std::string> &names,const std::vector<double> &positions,std::vector<std::string> *ignored) {
  const double receipt=now(); const auto generation=generation_.load();
  check_joint_input(names,positions);
  std::map<std::size_t,std::pair<std::vector<std::string>,std::vector<double>>> groups;
  for(std::size_t j=0;j<names.size();++j) {
    const auto route=joint_routes_.find(names[j]);
    if(route==joint_routes_.end()) { if(ignored) ignored->push_back(names[j]); continue; }
    if(ambiguous_joints_.count(names[j])) throw std::invalid_argument("ambiguous aggregate joint ownership");
    auto &group=groups[route->second.first]; group.first.push_back(names[j]); group.second.push_back(positions[j]);
  }
  if(lifecycle_!=1 && lifecycle_!=0) { ++rejected_; throw std::runtime_error("owner not running"); }
  if(groups.empty()) return next_sequence_;
  auto &slot=joint_targets_.write(); std::fill(slot.touched.begin(),slot.touched.end(),0);
  for(const auto &[i,group]:groups) {
    slot.names[i]=group.first;
    slot.values[i].vector=Eigen::Map<const Eigen::VectorXd>(group.second.data(),Eigen::Index(group.second.size()));
    slot.touched[i]=1;
  }
  slot.sequence=++next_sequence_; slot.generation=generation; slot.receipt=receipt;
  joint_targets_.publish(); ++coalesced_; return next_sequence_;
}
std::uint64_t Controller::feedback_joints(const std::vector<std::string> &names,const std::vector<double> &positions,double age) {
  if(!feedback_) throw std::invalid_argument("joint feedback is unavailable with an injected robot");
  check_joint_input(names,positions);
  if(!std::isfinite(age) || age<0 || age>options_.feedback_timeout_s) throw std::invalid_argument("stale or future feedback");
  auto candidate=joint_candidate_; double stamp=now()-age; bool known=false; std::size_t idx=0;
  for(const auto &joint:joints_) {
    auto it=std::find(names.begin(),names.end(),joint.name);
    if(it!=names.end()) {
      known=true; if(candidate.stamps[idx]>=0 && stamp<candidate.stamps[idx]) throw std::invalid_argument("out-of-order joint feedback");
      require(model_->set_joint_position(candidate.q,joint.id,positions[it-names.begin()])); candidate.stamps[idx]=stamp;
    }
    ++idx;
  }
  if(!known) throw std::invalid_argument("feedback contains no supported model joints");
  candidate.sequence=++next_sequence_;
  joint_candidate_=candidate; joint_input_.write()=candidate; joint_input_.publish(); return candidate.sequence;
}
std::uint64_t Controller::feedback_base(const Eigen::Isometry3d &p,double age) {
  if(!feedback_ || !root_) throw std::invalid_argument("base feedback requires a free root");
  if(!std::isfinite(age) || age<0 || age>options_.feedback_timeout_s) throw std::invalid_argument("stale or future odometry");
  kstone::PortInfo info;info.type=kstone::PortType::pose; kstone::PortValue v;v.pose=p;validate_port(info,v);
  double stamp=now()-age;
  if(base_candidate_stamp_>=0 && stamp<=base_candidate_stamp_) throw std::invalid_argument("odometry must be newer than last accepted stamp");
  if(chassis_) {
    auto c=p*center_in_root_; auto r=c.linear();
    if(std::abs(c.translation().z())>1e-6 || std::abs(r(2,0))>1e-6 || std::abs(r(2,1))>1e-6 || r(2,2)<1-1e-6)
      throw std::invalid_argument("chassis odometry control center must be planar");
  }
  base_candidate_stamp_=stamp; auto &s=base_input_.write(); s.value=v;s.receipt=stamp;s.sequence=++next_sequence_; base_input_.publish();return next_sequence_;
}
std::uint64_t Controller::feedback_center(const Eigen::Isometry3d &c,double age) {
  if(!chassis_) throw std::invalid_argument("center odometry requires a chassis module");
  return feedback_base(c*center_in_root_.inverse(),age);
}
bool Controller::joints_synced() const {
  for (std::size_t g=0;g<joint_groups_.size();++g) if (joint_groups_[g] && !engine_->group_attached(g)) return false;
  return true;
}
bool Controller::base_synced() const { return root_ && engine_->group_attached(base_group_); }
Eigen::Isometry3d Controller::pose(const std::string &id,const std::string &name) const {
  const auto a=addresses_[input_index(id,name)]; const auto &p=port(id,name);
  if(p.type!=kstone::PortType::pose || p.frame.empty()) throw std::invalid_argument("pose target requires a frame");
  return snapshot().poses[a.first][a.second];
}
Eigen::Isometry3d Controller::home_pose(const std::string &id,const std::string &name) const { return homes_.at({id,name}); }
Eigen::Isometry3d Controller::base_pose() const { return take(model_->root_pose(q())); }
kstone::Twist2D Controller::base_velocity() const { return written?snapshot().base_velocity:kstone::Twist2D{}; }
JointValues Controller::joint_state(const Eigen::VectorXd &q,const Eigen::VectorXd *v) const {
  const auto &locked=engine_->modules().solver().options().locked;
  JointValues out; for(const auto &j:joints_) { if(locked.test(std::size_t(j.idx_v))) continue; out.names.push_back(j.name);out.positions.push_back(take(model_->joint_position(q,j.id)));if(v)out.velocities.push_back((*v)[j.idx_v]); }return out;
}
Node Controller::discovery() const {
  auto ms=Node::list(); for(const auto &m:metadata_) { auto ps=Node::list();for(const auto &p:m->ports())ps.append(port_info(p));
    ms.append(Node::map({{"id",m->id()},{"type",m->type()},{"homeable",m->homeable()},{"ports",ps}})); }
  return Node::map({{"schema_version",1},{"rate_hz",1/snapshot().dt},{"modules",ms}});
}
Node Controller::state() const {
  const auto &s=snapshot();auto ms=Node::map();auto expired=Node::list();
  for(std::size_t i=0;i<metadata_.size();++i) {
    const auto &m=*metadata_[i];const auto &sm=s.engine.modules[i];auto ps=Node::map();auto outputs=Node::map();
    for(std::size_t j=0;j<m.inputs.size();++j)ps.set(m.inputs[j].name,port_node(m.inputs[j],read(m.id(),m.inputs[j].name)));
    for(std::size_t j=0;j<m.outputs.size();++j) {
      const auto &v=s.outputs[i][j];Node result;
      switch(m.outputs[j].type) {
      case kstone::OutputType::number: result=v.number;break;
      case kstone::OutputType::flag: result=v.flag;break;
      case kstone::OutputType::text: result=v.text;break;
      case kstone::OutputType::visuals: result=Node::list();for(int x:v.visuals)result.append(x);break;
      default: result=Node::list();for(Eigen::Index c=0;c<v.points.cols();++c)result.append(numbers(v.points.col(c)));
        if(m.outputs[j].type==kstone::OutputType::point && result.size()) {auto point=result.items()[0];result=std::move(point);}break;
      } outputs.set(m.outputs[j].name,result);
    }
    ms.set(m.id(),Node::map({{"enabled",sm.enabled},{"running",sm.running},{"held_by",sm.held_by<0?Node():Node(metadata_[sm.held_by]->id())},{"home_state",s.home_states[i].bytes[0]?Node(s.home_states[i].c_str()):Node()},{"ports",ps},{"outputs",outputs}}));
    if(s.expired[i])expired.append(m.id());
  }
  auto outcomes=Node::list(); const auto count=std::min(s.outcome_count,s.outcomes.size());
  for(std::size_t i=0;i<count;++i) {const auto &o=s.outcomes[(s.outcome_count-count+i)%s.outcomes.size()];
    outcomes.append(Node::map({{"request_id",double(o.sequence)},{"applied",o.applied},{"error",o.error.c_str()}}));}
  return Node::map({{"schema_version",1},{"fresh",fresh},{"joints_attached",joints_attached},{"base_attached",base_attached},{"written",written},{"quality",quality},{"fault",fault},{"expired_modules",expired},{"modules",ms},{"outcomes",outcomes},{"cycles",double(s.engine.stats.cycles)},{"failed_cycles",double(s.engine.stats.failed_cycles)},{"engine_time",s.engine.stats.time},{"snapshot_stamp",s.stamp},{"generation",double(s.generation)},{"lifecycle",lifecycle_.load()},{"rejected_requests",double(rejected_.load())},{"submitted_inputs",double(coalesced_.load())},{"dropped_logs",double(logs_->dropped.load())},{"truncated_logs",double(logs_->truncated.load())}});
}
} // namespace kstone_ros
