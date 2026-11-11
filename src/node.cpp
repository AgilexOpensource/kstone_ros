#include "kstone_ros/controller.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace kstone_ros {
using Pose = geometry_msgs::msg::PoseStamped;
using Twist = geometry_msgs::msg::Twist;
using Joints = sensor_msgs::msg::JointState;
using String = std_msgs::msg::String;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;
class KstoneNode : public rclcpp::Node {
public:
  KstoneNode() : rclcpp::Node("kstone") {
    const auto config = parameter<std::string>("config_file", "");
    const auto vars = parameter<std::string>("config_variables", "{}");
    Options options;
    prefix_ = parameter<std::string>("topic_prefix", "~");
    options.duration_s = parameter<double>("duration", 0.0);
    rcl_interfaces::msg::ParameterDescriptor realtime_descriptor; realtime_descriptor.read_only = true;
    declare_parameter("realtime", rclcpp::ParameterType::PARAMETER_BOOL, realtime_descriptor);
    rclcpp::Parameter realtime;
    if (get_parameter("realtime", realtime) && realtime.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      options.realtime = realtime.as_bool();
    while (!prefix_.empty() && prefix_.back() == '/')
      prefix_.pop_back();
    world_ = parameter<std::string>("world_frame", "world");
    base_ = parameter<std::string>("base_frame", "base_link");
    if (prefix_.empty() || world_.empty() || base_.empty())
      throw std::invalid_argument(
          "topic_prefix, world_frame and base_frame must be nonempty");
    options.command_timeout_s = parameter<double>("command_timeout_s", 0.5);
    options.feedback_timeout_s = parameter<double>("feedback_timeout_s", 0.25);
    state_rate_ = parameter<double>("state_rate_hz", 30.0);
    options.chassis_module = parameter<std::string>("chassis_module", "");
    const auto feedback_joint =
        parameter<std::string>("feedback_joint_topic", topic("joint_states"));
    const auto feedback_base =
        parameter<std::string>("feedback_base_topic", "");
    const auto joint_command =
        parameter<std::string>("joint_command_topic", topic("feedback/joint_states"));
    const auto base_command =
        parameter<std::string>("base_command_topic", topic("feedback/cmd_vel"));
    const bool publish_tf = parameter<bool>("publish_tf", false);
    if (!std::isfinite(state_rate_) || state_rate_ <= 0)
      throw std::invalid_argument("state_rate_hz must be finite and positive");
    command_timeout_ = options.command_timeout_s;
    feedback_timeout_ = options.feedback_timeout_s;
    const auto variables = parse_json(vars);
    if (variables.kind() != kstone::ConfigNode::Kind::map)
      throw std::invalid_argument(
          "config_variables must be a JSON object of string values");
    for (const auto &v : variables.entries()) {
      if (v.second.kind() != kstone::ConfigNode::Kind::string)
        throw std::invalid_argument(
            "config_variables must contain string values");
      options.variables[v.first] = v.second.as_string();
    }
    const auto debug_port = parameter<int>("debug_port", 0);
    const auto debug_host = parameter<std::string>("debug_host", "127.0.0.1");
    auto debug_token = parameter<std::string>("debug_token", "");
    if (debug_port < 0 || debug_port > 65535)
      throw std::invalid_argument("debug_port must be in [0, 65535]");
    if (debug_port > 0) {
      kstone::DebugServerOptions debug;
      debug.host = debug_host;
      debug.port = static_cast<std::uint16_t>(debug_port);
      if (const char *env = std::getenv("KSTONE_TOKEN"); debug_token.empty() && env) debug_token = env;
      debug.token = std::move(debug_token);
      options.debug = std::move(debug);
    }
    core_ = std::make_unique<Controller>(config, options);
    chassis_frame_ = base_;
    for (const auto &m : core_->modules())
      if (m->id() == core_->chassis()) chassis_frame_ = m->chassis_frame;
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    state_pub_ = publisher<String>(topic("state"), latched);
    discovery_pub_ = publisher<String>(topic("discover"), latched);
    canonical_joint_command_pub_ = publisher<Joints>(topic("feedback/joint_states"));
    canonical_base_command_pub_ = publisher<Twist>(topic("feedback/cmd_vel"), rclcpp::QoS(10), chassis_frame_);
    if (!joint_command.empty() && get_node_topics_interface()->resolve_topic_name(joint_command) != canonical_joint_command_pub_->get_topic_name())
      joint_command_pub_ = publisher<Joints>(joint_command);
    if (!base_command.empty() && get_node_topics_interface()->resolve_topic_name(base_command) != canonical_base_command_pub_->get_topic_name())
      base_command_pub_ = publisher<Twist>(base_command, rclcpp::QoS(10), chassis_frame_);
    // Driver output stays off until real feedback has been synced into the model.
    {
      if (feedback_joint.empty() || get_node_topics_interface()->resolve_topic_name(feedback_joint) != get_node_topics_interface()->resolve_topic_name(topic("joint_states")))
        subscribe<Joints>(topic("joint_states"), [this](const Joints &m) {
          core_->feedback_joints(m.name,m.position,feedback_age(m.header));
        }, rclcpp::SensorDataQoS());
      if (!feedback_joint.empty())
        subscribe<Joints>(
            feedback_joint,
            [this](const Joints &m) {
              core_->feedback_joints(m.name, m.position,
                                     feedback_age(m.header));
            },
            rclcpp::SensorDataQoS());
      if (core_->has_root())
        subscribe<nav_msgs::msg::Odometry>(topic("odom"), [this](const nav_msgs::msg::Odometry &msg) {
          if (!msg.child_frame_id.empty() && msg.child_frame_id != base_) throw std::invalid_argument("unexpected odometry child frame");
          Pose p; p.header=msg.header; p.pose=msg.pose.pose;
          core_->feedback_base(pose_input(p, world_, false), feedback_age(p.header));
        }, rclcpp::SensorDataQoS());
      if (!feedback_base.empty() && core_->has_root())
        subscribe<Pose>(
            feedback_base,
            [this](const Pose &m) {
              core_->feedback_base(pose_input(m, world_, false),
                                   feedback_age(m.header));
            },
            rclcpp::SensorDataQoS());
    }
    for (const auto &m : core_->modules()) {
      if (!supported_ros_module(m->type())) continue;
      const std::string id = m->id(), root = topic(id);
      if (m->homeable())
        service<Trigger>(root + "/home", [this, id](auto, auto response) {
          accepted(response, [&] { return core_->home(id); });
        });
      service<SetBool>(root + "/enable", [this, id](auto request, auto response) {
        accepted(response, [&] { return core_->enable(id, request->data); });
      });
      if (m->type() == "frame_target") {
        const auto endpoint=topic(id+"/control/pose");
        subscribe<Pose>(endpoint,[this,id](const Pose &msg) {
          kstone::PortValue value; value.pose=pose_input(msg,port_frame(id,"target"));
          core_->write(id,"target",value);
        });
        module_interface(id,endpoint,"subscribe","command","target",port_frame(id,"target"));
        auto pub=publisher<Pose>(topic(id+"/feedback/pose"));
        module_interface(id,topic(id+"/feedback/pose"),"publish","fk","",port_frame(id,"target"));
        feedback_publish_.push_back([this,id,pub] {
          pub->publish(pose_message(core_->pose(id,"target"),port_frame(id,"target")));
        });
      } else if (m->type() == "chassis") {
        const auto endpoint=topic(id+"/control/cmd_vel");
        subscribe<Twist>(endpoint,[this,id](const Twist &msg) { twist_input(id,msg); }, rclcpp::QoS(10),
                         kstone_ros::Node::map({{"frame",m->chassis_frame}}));
        module_interface(id,endpoint,"subscribe","command","velocity",m->chassis_frame);
        if(m->chassis_omni) {
          const auto pose_endpoint=topic(id+"/control/pose");
          subscribe<Pose>(pose_endpoint,[this,id](const Pose &msg) {
            const auto p=pose_input(msg,world_); const auto &r=p.linear();
            if(std::abs(p.translation().z())>1e-9 || std::abs(r(2,0))>1e-9 ||
               std::abs(r(2,1))>1e-9 || std::abs(r(0,2))>1e-9 ||
               std::abs(r(1,2))>1e-9 || std::abs(r(2,2)-1)>1e-9)
              throw std::invalid_argument("chassis pose must have zero z and yaw-only rotation");
            kstone::PortValue value;
            value.vector=Eigen::Vector3d(p.translation().x(),p.translation().y(),std::atan2(r(1,0),r(0,0)));
            core_->write(id,"pose_target",value);
          });
          module_interface(id,pose_endpoint,"subscribe","command","pose_target",world_);
        }
        auto pub=publisher<Pose>(topic(id+"/feedback/pose"));
        module_interface(id,topic(id+"/feedback/pose"),"publish","odometry","",world_);
        feedback_publish_.push_back([this,pub] { pub->publish(pose_message(core_->chassis_pose(),world_)); });
      }
    }
    // Immutable configured routing, advertised with the resolved subscription.
    auto routes=kstone_ros::Node::list();
    for(const auto &m:core_->modules()) if(m->type()=="joint_target") {
      auto names=kstone_ros::Node::list();
      for(const auto &name:core_->port(m->id(),"target").labels) names.append(name);
      routes.append(kstone_ros::Node::map({{"module",m->id()},{"port","target"},{"joint_names",names}}));
    }
    subscribe<Joints>(topic("control/joint"),[this](const Joints &m) {
      joint_target_header(m.header);
      std::vector<std::string> ignored;
      core_->joint_targets(m.name,m.position,&ignored);
      if(!ignored.empty()) {
        std::string list; for(const auto &n:ignored) list+=(list.empty()?"":", ")+n;
        warn(topic("control/joint")+" ignored","joints outside every joint_target chain: "+list);
      }
    },rclcpp::QoS(10),kstone_ros::Node::map({
      {"semantics","target"},
      {"routing","by_joint_name"},{"module_routes",routes}}));
    for(const auto &m:core_->modules()) if(m->type()=="joint_target")
      module_interface(m->id(),topic("control/joint"),"subscribe","command","target");
    service<Trigger>(topic("home"), [this](auto, auto response) {
      accepted(response, [&] { return core_->reset_home(); });
    });

    if (publish_tf) {
      if (!core_->has_root() || world_ == base_)
        throw std::invalid_argument(
            "publish_tf requires a free root and distinct frames");
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    // Resolve after all endpoints exist; includes ROS remapping and dynamic
    // typed module topics.
    std::set<std::string> outputs;
    for (const auto &p : publishers_)
      if (!outputs.insert(p->get_topic_name()).second)
        throw std::invalid_argument(
            "output topics must not alias other bridge publications");
    for (const auto &s : subscriptions_)
      if (outputs.count(s->get_topic_name()))
        throw std::invalid_argument(
            "input topics must not alias bridge publications");
    // Presentation only: never change endpoint registration or owner request priority.
    std::map<std::string,int> display_ranks;
    auto display_rank=[&](const std::string &name,int rank) {
      const auto found=resolved_.find(name);
      if(found!=resolved_.end()) display_ranks[found->second]=rank;
    };
    display_rank(topic("joint_states"),0); display_rank(feedback_joint,0);
    display_rank(topic("odom"),1); display_rank(feedback_base,1);
    display_rank(topic("feedback/joint_states"),2); display_rank(joint_command,2);
    display_rank(topic("feedback/cmd_vel"),3); display_rank(base_command,3);
    const std::vector<std::string> globals={"state","discover","control/joint"};
    for(std::size_t i=0;i<globals.size();++i) display_rank(topic(globals[i]),4+int(i));
    const std::vector<std::string> module_names={"state","feedback/pose","control/pose","control/cmd_vel","enable","home"};
    for(const auto &m:core_->modules()) for(std::size_t i=0;i<module_names.size();++i)
      display_rank(topic(m->id()+"/"+module_names[i]),i<4?11+int(i):13+int(i));
    display_rank(topic("home"),16);
    auto ordered_interfaces=[&](const kstone_ros::Node &records) {
      auto rows=records.items();
      auto rank=[&](const kstone_ros::Node &record) {
        const bool service=record.find("direction")->as_string()=="service";
        const auto *endpoint=record.find(service?"service":"topic");
        const auto found=endpoint?display_ranks.find(endpoint->as_string()):display_ranks.end();
        return found!=display_ranks.end()?found->second:(service?19:10);
      };
      std::stable_sort(rows.begin(),rows.end(),[&](const auto &a,const auto &b){return rank(a)<rank(b);});
      auto result=kstone_ros::Node::list();for(const auto &row:rows) result.append(row);
      return result;
    };
    interfaces_=ordered_interfaces(interfaces_);
    for(auto &[id,records]:module_interfaces_) records=ordered_interfaces(records);
    discovery_ = core_->discovery();
    auto discovered_modules = kstone_ros::Node::list();
    const auto engine_discovery=core_->discovery();
    for(const auto &original : engine_discovery.find("modules")->items()) {
      auto module=original; auto ports=kstone_ros::Node::list();
      const auto id=module.find("id")->as_string();
      const bool supported=supported_ros_module(module.find("type")->as_string());
      module.set("ros_topics_supported",supported);
      for(const auto &original_port : module.find("ports")->items()) {
        auto p=original_port; const auto name=p.find("name")->as_string();
        const auto input=module_controls_.find({id,name});
        p.set("ros_exposed",input!=module_controls_.end());
        if(input!=module_controls_.end()) {
          const auto &description=input->second;
          p.set("control_topic",*description.find("topic"));
          p.set("control_type",*description.find("type"));
          if(const auto *frame=description.find("frame")) p.set("ros_frame",*frame);
        }
        if(supported && module.find("type")->as_string()=="frame_target" && name=="target") {
          p.set("home_pose",pose_node(core_->home_pose(id,name)));
          p.set("home_frame",port_frame(id,name));
        }
        ports.append(p);
      }
      module.set("ports",ports);
      const auto descriptions=module_interfaces_.find(id);
      module.set("ros_interfaces",descriptions==module_interfaces_.end()?kstone_ros::Node::list():descriptions->second);
      for(const auto &info:core_->modules()) if(info->id()==id && info->type()=="chassis") {
        module.set("chassis_kinematics",info->chassis_omni?"omni":"differential");
        module.set("chassis_frame",info->chassis_frame);
      }
      discovered_modules.append(module);
    }
    discovery_.set("modules",discovered_modules);
    discovery_.set("interfaces", interfaces_);
    discovery_.set("world_frame",world_); discovery_.set("base_frame",base_);

    String discovery; discovery.data=json(discovery_);
    discovery_pub_->publish(discovery);
    core_->start();
    schedule(core_->dt());
    base_hold_ = options.command_timeout_s;
    RCLCPP_INFO(get_logger(), "Loaded %zu modules; driver output waits for feedback",
                core_->modules().size());
    if (core_->debug_port())
      RCLCPP_INFO(get_logger(), "Debug server on %s:%d (%s)", options.debug->host.c_str(),
                  core_->debug_port(), options.debug->token.empty() ? "no authentication" : "token required");
  }
  ~KstoneNode() override { stop(); }
  void stop() {
    if (core_) core_->stop();
    if (timer_)
      timer_->cancel();
    if (rclcpp::ok() && core_ && canonical_base_command_pub_) base_output(false, {});
  }

private:
  template <class T> T parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  template <class Msg>
  typename rclcpp::Publisher<Msg>::SharedPtr
  publisher(const std::string &name, const rclcpp::QoS &qos = rclcpp::QoS(10),
            const std::string &frame = "") {
    auto p = create_publisher<Msg>(name, qos);
    publishers_.push_back(p);
    resolved_[name] = p->get_topic_name();
    auto description = kstone_ros::Node::map({{"direction","publish"},{"topic",p->get_topic_name()},{"type",rosidl_generator_traits::name<Msg>()},{"depth",double(qos.get_rmw_qos_profile().depth)},{"reliability",qos.get_rmw_qos_profile().reliability==RMW_QOS_POLICY_RELIABILITY_RELIABLE?"reliable":"best_effort"},{"durability",qos.get_rmw_qos_profile().durability==RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL?"transient_local":"volatile"}});
    if (!frame.empty()) description.set("frame", frame);
    interfaces_.append(description);
    return p;
  }
  template <class Msg, class Callback>
  void subscribe(const std::string &name, Callback cb,
                 const rclcpp::QoS &qos = rclcpp::QoS(10),
                 kstone_ros::Node description = kstone_ros::Node::map({})) {
    auto subscription = create_subscription<Msg>(
        name, qos, [this, name, cb](typename Msg::ConstSharedPtr msg) {
          try {
            cb(*msg);
          } catch (const std::exception &e) {
            warn(name, e.what());
          }
        });
    subscriptions_.push_back(subscription);
    resolved_[name] = subscription->get_topic_name();
    description.set("direction","subscribe");
    description.set("topic",subscription->get_topic_name());
    description.set("type",rosidl_generator_traits::name<Msg>());
    description.set("depth",double(qos.get_rmw_qos_profile().depth));
    description.set("reliability",qos.get_rmw_qos_profile().reliability==RMW_QOS_POLICY_RELIABILITY_RELIABLE?"reliable":"best_effort");
    description.set("durability","volatile");
    interfaces_.append(description);
  }
  template <class Srv, class Callback>
  void service(const std::string &name, Callback cb) {
    auto srv = create_service<Srv>(
        name, [cb](const std::shared_ptr<typename Srv::Request> request,
                   std::shared_ptr<typename Srv::Response> response) {
          cb(request, response);
        });
    services_.push_back(srv);
    resolved_[name] = srv->get_service_name();
    interfaces_.append(kstone_ros::Node::map({{"direction","service"},{"service",srv->get_service_name()},{"type",rosidl_generator_traits::name<Srv>()},{"qos","services_default"}}));
  }
  template <class Response, class Callback>
  void accepted(Response response, Callback operation) {
    try {
      const auto id=operation();
      response->success = true;
      response->message = "accepted request_id=" + std::to_string(id);
    } catch (const std::exception &e) {
      response->success = false;
      response->message = e.what();
    }
  }
  std::string topic(const std::string &suffix) const {
    return prefix_ + "/" + suffix;
  }
  std::string port_frame(const std::string &id, const std::string &name) const {
    const auto &p = core_->port(id, name);
    return p.reference.empty() ? world_ : p.reference;
  }
  double age(const std_msgs::msg::Header &header) {
    if (header.stamp.sec == 0 && header.stamp.nanosec == 0)
      return 0.0;
    return (get_clock()->now().nanoseconds() -
            (static_cast<std::int64_t>(header.stamp.sec) * 1000000000 +
             header.stamp.nanosec)) *
           1e-9;
  }
  void joint_target_header(const std_msgs::msg::Header &header) {
    const double a=age(header);
    if(a<0 || a>command_timeout_) throw std::invalid_argument("stale/future stamped joint target");
    if(!header.frame_id.empty())
      throw std::invalid_argument("joint target frame_id must be empty; named joint coordinates need no TF conversion");
  }
  double feedback_age(const std_msgs::msg::Header &header) {
    const double a = age(header);
    if (a < 0 || a > feedback_timeout_)
      throw std::invalid_argument(
          "feedback timestamp is stale or in the future");
    return a;
  }
  Eigen::Isometry3d pose_input(const Pose &msg, const std::string &frame,
                               bool validate_stamp = true) {
    if (validate_stamp) {
      const double a = age(msg.header);
      if (a < 0 || a > command_timeout_)
        throw std::invalid_argument(
            "message timestamp is stale or in the future");
    }
    if (!msg.header.frame_id.empty() && msg.header.frame_id != frame)
      throw std::invalid_argument("expected frame " + frame +
                                  "; no implicit TF conversion");
    const auto &p = msg.pose.position;
    const auto &q = msg.pose.orientation;
    return make_pose({p.x, p.y, p.z}, {q.w, q.x, q.y, q.z});
  }
  Pose pose_message(const Eigen::Isometry3d &p, const std::string &frame) {
    Pose m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = frame;
    m.pose.position.x = p.translation().x();
    m.pose.position.y = p.translation().y();
    m.pose.position.z = p.translation().z();
    Eigen::Quaterniond q(p.linear());
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    return m;
  }
  Twist base_command_message(kstone::Twist2D velocity = {}) {
    Twist m;
    // Bare Twist is implicitly in the chassis control-center/body frame.
    m.linear.x = velocity.vx;
    m.linear.y = velocity.vy;
    m.angular.z = velocity.wz;
    return m;
  }
  void module_interface(const std::string &id,const std::string &endpoint,
                        const std::string &direction,const std::string &source,
                        const std::string &port="",const std::string &frame="") {
    for(const auto &record:interfaces_.items()) {
      const auto *name=record.find("topic");
      if(!name || name->as_string()!=resolved_.at(endpoint) || record.find("direction")->as_string()!=direction) continue;
      auto description=record;
      description.set("semantics",direction=="subscribe"?"target":"feedback");
      description.set("source",source);
      if(!frame.empty()) description.set("frame",frame);
      if(!port.empty()) {
        description.set("port",port);
        module_controls_[{id,port}]=description;
      }

      module_interfaces_.try_emplace(id,kstone_ros::Node::list()).first->second.append(description);
      return;
    }
    throw std::logic_error("missing typed module endpoint metadata");
  }
  Joints joint_message(const Eigen::VectorXd &q,
                       const Eigen::VectorXd *v = nullptr) {
    Joints m;
    m.header.stamp = get_clock()->now();
    auto values = core_->joint_state(q, v);
    m.name = std::move(values.names);
    m.position = std::move(values.positions);
    m.velocity = std::move(values.velocities);
    return m;
  }
  // cmd_vel only after odometry attached; after detach, zero for one command timeout, then silence.
  void base_output(bool attached, const kstone::Twist2D &v) {
    const double t = core_->now();
    if (attached) base_zero_until_ = t + base_hold_;
    else if (!base_zero_until_ || t > *base_zero_until_) return;
    const auto m = base_command_message(attached ? v : kstone::Twist2D{});
    if (base_command_pub_) base_command_pub_->publish(m);
    canonical_base_command_pub_->publish(m);
  }
  void warn(const std::string &key, const std::string &message) {
    const double t = core_->now();
    auto i = warnings_.find(key);
    if (i == warnings_.end() || t - i->second > 2) {
      warnings_[key] = t;
      RCLCPP_WARN(get_logger(), "%s: %s", key.c_str(), message.c_str());
    }
  }
  // Only planar x, y and yaw are read; the SDK drops y on a differential chassis.
  void twist_input(const std::string &id,const Twist &m) {
    kstone::PortValue value; value.vector=Eigen::Vector3d(m.linear.x,m.linear.y,m.angular.z);
    if (!within_bounds(core_->port(id,"velocity"),value)) warn(id+"/control/cmd_vel limited","exceeds velocity bounds; limited by the chassis");
    core_->write(id,"velocity",value);
  }
  // Commands follow the engine rate; a runtime rate change reschedules on the next tick.
  void schedule(double dt) {
    retired_ = timer_;
    if (retired_) retired_->cancel();
    timer_dt_ = dt;
    timer_ = create_wall_timer(std::chrono::duration<double>(dt), [this] { tick(); });
  }
  void tick() {
    retired_.reset();
    try {
      const bool safe = core_->poll();
      if (core_->dt() != timer_dt_) schedule(core_->dt());
      CopiedLog record;
      for (unsigned i=0;i<32 && core_->logs()->pop(record);++i) {
        if(record.level>=kstone::LogLevel::error) RCLCPP_ERROR(get_logger(),"%s: %s",record.component.c_str(),record.message.c_str());
        else if(record.level>=kstone::LogLevel::warn) RCLCPP_WARN(get_logger(),"%s: %s",record.component.c_str(),record.message.c_str());
        else RCLCPP_DEBUG(get_logger(),"%s: %s",record.component.c_str(),record.message.c_str());
      }
      if (safe && core_->joints_attached) {
        const auto m = joint_message(core_->command_q(), &core_->velocity());
        canonical_joint_command_pub_->publish(m);
        if (joint_command_pub_) joint_command_pub_->publish(m);
      }
      base_output(core_->base_attached, safe ? core_->base_velocity() : kstone::Twist2D{});
      if (tf_) {
        const auto pose = pose_message(core_->base_pose(), world_);
        geometry_msgs::msg::TransformStamped t;
        t.header = pose.header;
        t.child_frame_id = base_;
        t.transform.translation.x = pose.pose.position.x;
        t.transform.translation.y = pose.pose.position.y;
        t.transform.translation.z = pose.pose.position.z;
        t.transform.rotation = pose.pose.orientation;
        tf_->sendTransform(t);
      }
      const double t = core_->now();
      if (!next_state_ || t >= *next_state_) {
        // Advance from the deadline, not from t, so tick jitter does not lower the rate.
        next_state_ = next_state_ && *next_state_ + 1 / state_rate_ > t ? *next_state_ + 1 / state_rate_ : t + 1 / state_rate_;
        auto state = core_->state();
        String message;
        message.data = json(state);
        state_pub_->publish(message);
      }
      for (const auto &publish : feedback_publish_)
        publish();
      if (core_->fault != last_fault_) {
        if (!core_->fault.empty()) RCLCPP_WARN(get_logger(), "control: %s", core_->fault.c_str());
        else RCLCPP_INFO(get_logger(), "control: fault cleared");
        last_fault_ = core_->fault;
      }
    } catch (const std::exception &e) {
      core_->written = false;
      core_->fault = std::string("cycle_error: ") + e.what();
      base_output(false, {});
      warn("publish", e.what());
    }
  }
  kstone_ros::Node discovery_, interfaces_{kstone_ros::Node::list()};
  std::unique_ptr<Controller> core_;
  std::string prefix_, world_, base_;
  double command_timeout_{0.5}, feedback_timeout_{0.25}, state_rate_{30}, timer_dt_{0};
  std::optional<double> next_state_, base_zero_until_;
  double base_hold_{0.5};
  std::map<std::string, double> warnings_;
  std::string last_fault_;
  std::map<std::string, std::string> resolved_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<std::function<void()>> feedback_publish_;
  std::map<std::string,kstone_ros::Node> module_interfaces_;
  std::map<std::pair<std::string,std::string>,kstone_ros::Node> module_controls_;
  std::string chassis_frame_;
  rclcpp::Publisher<String>::SharedPtr state_pub_, discovery_pub_;
  rclcpp::Publisher<Joints>::SharedPtr joint_command_pub_, canonical_joint_command_pub_;
  rclcpp::Publisher<Twist>::SharedPtr base_command_pub_, canonical_base_command_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_, retired_;
};
} // namespace kstone_ros
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<kstone_ros::KstoneNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    node->stop();
  } catch (const std::exception &e) {
    std::cerr << "kstone: " << e.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
