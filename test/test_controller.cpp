#include "kstone_ros/controller.hpp"
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <set>
#include <chrono>
#include <thread>

using namespace kstone_ros;
namespace {
int count = 0;
void check(bool v, const std::string &message) {
  if (!v)
    throw std::runtime_error(message);
}
template <class F> void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::exception &) {
    caught = true;
  }
  check(caught, "expected rejection");
}
kstone::PortValue vector(std::initializer_list<double> x) {
  kstone::PortValue v;
  v.vector = Eigen::Map<const Eigen::VectorXd>(x.begin(), x.size());
  return v;
}
bool cycle(Controller &c) {
  c.poll(); auto cycles=c.snapshot().engine.stats.cycles;
  if(c.state().find("lifecycle")->as_number()==0) c.start();
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(std::chrono::steady_clock::now()<deadline) {
    c.poll();
    if(c.snapshot().engine.stats.cycles>cycles) return c.written;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("engine owner did not progress: "+c.fault);
}
bool enabled(const Controller &c,const std::string &id) {
  for(std::size_t i=0;i<c.modules().size();++i) if(c.modules()[i]->id()==id) return c.snapshot().engine.modules[i].enabled;
  throw std::runtime_error("unknown module");
}
struct Fixture {
  std::shared_ptr<kstone::ManualClock> clock =
      std::make_shared<kstone::ManualClock>();
  std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("kstone-unit-" + std::to_string(getpid()));
  Fixture() { std::filesystem::create_directories(directory); }
  ~Fixture() { std::filesystem::remove_all(directory); }
  Options options(bool = false, bool = false) {
    Options o;
    o.clock = clock;
    return o;
  }
  std::string joint_config(bool overlap=false) {
    return config(true,true,[overlap](Node &n) {
      auto hand=*n.find("modules")->find("gripper_joint"), axes=hand;
      hand.set("joints",Node::list({"finger_joint","slide_x"}));
      axes.set("joints",overlap?Node::list({"slide_x","slide_y"}):Node::list({"slide_y","slide_z"}));
      n.set("modules",Node::map({{"hand",hand},{"axes",axes}}));
    });
  }
  void advance(double dt = 0.02) {
    clock->advance_ns(static_cast<int64_t>(dt * 1e9));
  }
  std::string config(bool fixed = false, bool no_chassis = false,
                     const std::function<void(Node &)> &edit = {}) {
    auto n = take(kstone::load_yaml(KSTONE_TEST_CONFIG));
    auto model = *n.find("model");
    model.set("file", std::filesystem::path(KSTONE_TEST_CONFIG)
                          .replace_extension(".urdf")
                          .string());
    if (fixed)
      model.set("root", "fixed");
    n.set("model", model);
    if (no_chassis) {
      auto modules = Node::map();
      for (const auto &m : n.find("modules")->entries())
        if (m.first != "chassis")
          modules.set(m.first, m.second);
      n.set("modules", modules);
    }
    if (edit)
      edit(n);
    auto path = directory / (std::to_string(count) + ".yaml");
    std::ofstream(path) << json(n);
    return path.string();
  }
};
// Feeds fresh joints each cycle until the attach window completes; returns the cycles taken.
int attach(Fixture &f, Controller &c, const JointValues &j) {
  for (int i = 1; i <= 30; ++i) {
    c.feedback_joints(j.names, j.positions);
    cycle(c);
    if (c.joints_attached) return i;
    f.advance();
  }
  throw std::runtime_error("feedback did not attach: " + c.fault);
}
Eigen::VectorXd clipped_seed(const Controller &c, const Eigen::VectorXd &raw) {
  Eigen::VectorXd q = raw;
  for (const auto &m : c.model()->mimics()) {
    const auto &f = c.model()->joints()[take(c.model()->joint_id(m.follower))];
    const auto &l = c.model()->joints()[take(c.model()->joint_id(m.leader))];
    q[f.idx_q] = m.multiplier * q[l.idx_q] + m.offset;
  }
  for (const auto &j : c.model()->joints())
    if (j.nq == 1 && j.nv == 1 && c.model()->has_position_limit(j.id))
      q[j.idx_q] = std::clamp(q[j.idx_q], c.model()->lower_position_limit()[j.idx_q],
                            c.model()->upper_position_limit()[j.idx_q]);
  return q;
}
Eigen::Isometry3d frame_at(const Controller &c, const Eigen::VectorXd &q, const std::string &frame,
                         const std::string &reference = {}) {
  kstone::Workspace at(c.model());
  check(bool(at.update(q)), "FK update failed");
  Eigen::Isometry3d pose;
  check(bool(at.frame_pose(take(c.model()->frame_id(frame)), pose)), "frame FK failed");
  if (!reference.empty()) {
    Eigen::Isometry3d ref;
    check(bool(at.frame_pose(take(c.model()->frame_id(reference)), ref)), "reference FK failed");
    pose = ref.inverse() * pose;
  }
  return pose;
}
class InstrumentedRobot final : public kstone::RobotInterface {
public:
  InstrumentedRobot(const Controller &seed,std::shared_ptr<const kstone::Clock> clock)
    : loop_(seed.measured(),seed.model()->nv(),std::move(clock)) {}
  std::atomic<bool> fail_read{false}, fail_write{false}, blocked{false}, waiting{false}, unknown_stamp{false};
  std::atomic<std::uint64_t> reads{0}, writes{0}, read_thread{0}, write_thread{0};
  std::shared_ptr<BufferedLog> logs;
  kstone::Status read(kstone::RobotState &state) override {
    ++reads; read_thread=std::hash<std::thread::id>{}(std::this_thread::get_id());
    while(blocked){waiting=true;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    waiting=false;
    if(fail_read)return kstone::make_error(kstone::ErrorCode::io_error,"injected read failure");
    if(logs) {
      std::string component="driver", message="owned temporary log view";
      for(int i=0;i<300;++i)logs->write({kstone::LogLevel::warn,component,message,0});
    }
    auto status=loop_.read(state);
    if(unknown_stamp){state.stamp_ns=-1;std::fill(state.group_stamps_ns.begin(),state.group_stamps_ns.end(),-1);}
    return status;
  }
  kstone::Status write(const kstone::RobotCommand &command) override {
    ++writes; write_thread=std::hash<std::thread::id>{}(std::this_thread::get_id());
    if(fail_write)return kstone::make_error(kstone::ErrorCode::io_error,"injected write failure");
    return loop_.write(command);
  }
private:
  kstone::LoopbackRobot loop_;
};
template <class F> void await(Controller &c,F predicate) {
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  do {
    c.poll();if(predicate())return;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }while(std::chrono::steady_clock::now()<deadline);
  throw std::runtime_error("owner condition timed out: "+c.fault);
}
template <class F> void test(const char *name, F f) {
  ++count;
  f();
  std::cout << "PASS " << name << '\n';
}
} // namespace
int main() {
  try {
    test("pose validation normalization and half turn", [] {
      auto p = make_pose({1, 2, 3}, {2, 0, 0, 0});
      check(p.translation().x() == 1, "translation");
      check(p.linear().isApprox(Eigen::Matrix3d::Identity()), "normalization");
      check(std::abs(pose_node(make_pose({0, 0, 0}, {0, 1, 0, 0}))
                         .find("orientation")
                         ->items()[1]
                         .as_number()) == 1,
            "half turn");
      rejects([] { make_pose({0, 0, 0}, {0, 0, 0, 0}); });
      rejects([] { make_pose({NAN, 0, 0}, {1, 0, 0, 0}); });
    });
    test("strict JSON and invalid JSON", [] {
      auto n = parse_json(json(Node::list({NAN, INFINITY, 1})));
      check(n.items()[0].kind() == Node::Kind::null &&
                n.items()[1].kind() == Node::Kind::null &&
                n.items()[2].as_number() == 1,
            "strict JSON");
      rejects([] { parse_json("{bad}"); });
      for (const auto *invalid : {"NaN", "Infinity", "[1,]", "{} trailing",
                                  "{\"value\":1e400}"})
        rejects([&] { parse_json(invalid); });
      const auto text = std::string("quoted \"text\"\n\\ UTF-8: \xc3\xa9");
      auto values = Node::map({{"text", text}, {"flag", true},
                               {"values", Node::list({Node(), false, -2, 1.25})}});
      auto roundtrip = parse_json(json(values));
      check(roundtrip.find("text")->as_string() == text &&
                roundtrip.find("flag")->as_bool() &&
                roundtrip.find("values")->items()[0].kind() == Node::Kind::null &&
                !roundtrip.find("values")->items()[1].as_bool() &&
                roundtrip.find("values")->items()[2].as_number() == -2 &&
                roundtrip.find("values")->items()[3].as_number() == 1.25,
            "nested JSON types and escaped/UTF-8 strings roundtrip");
    });
    test("typed number flag vector and malformed rigid transform", [] {
      kstone::PortInfo p;
      kstone::PortValue v;
      p.type = kstone::PortType::number;
      p.lower = Eigen::VectorXd::Constant(1, -1);
      p.upper = Eigen::VectorXd::Constant(1, 1);
      v.number = 0.5;
      validate_port(p, v);
      check(within_bounds(p, v), "inside bounds");
      v.number = NAN;
      rejects([&] { validate_port(p, v); });
      v.number = 2;
      validate_port(p, v);
      check(!within_bounds(p, v), "above upper bound");
      v.number = -2;
      validate_port(p, v);
      check(!within_bounds(p, v), "below lower bound");
      p.lower = p.upper = Eigen::VectorXd::Zero(1);
      check(within_bounds(p, v), "fixed axis is not checked");
      p.lower = Eigen::VectorXd::Constant(1, -1);
      p.upper = Eigen::VectorXd::Constant(1, 1);
      p.type = kstone::PortType::flag;
      v.flag = true;
      validate_port(p, v);
      v.flag = false;
      validate_port(p, v);
      p.type = kstone::PortType::pose;
      v.pose = Eigen::Isometry3d::Identity();
      validate_port(p, v);
      v.pose.matrix()(3, 0) = 1;
      rejects([&] { validate_port(p, v); });
      v.pose = Eigen::Isometry3d::Identity();
      v.pose.linear()(0, 0) = 2;
      rejects([&] { validate_port(p, v); });
    });
    test("discovery ports schema", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      check(c.discovery().find("schema_version")->as_number() == 1, "schema");
      check(c.modules().size() == 3, "modules");
      check(c.port("chassis", "velocity").labels.size() == 3, "labels");
      check(c.port("tool_pose", "target").reference == "base_link", "reference");
    });
    test("invalid options and token root ownership", [] {
      Fixture f;
      auto o = f.options();
      o.command_timeout_s = 0;
      rejects([&] { Controller c(KSTONE_TEST_CONFIG, o); });
      o = f.options();
      o.feedback_timeout_s = NAN;
      rejects([&] { Controller c(KSTONE_TEST_CONFIG, o); });
      o = f.options();
      o.chassis_module = "tool_pose";
      rejects([&] { Controller c(KSTONE_TEST_CONFIG, o); });
      auto file = f.config(false, false, [](Node &n) {
        auto ms = *n.find("modules");
        ms.set("another", *ms.find("chassis"));
        n.set("modules", ms);
      });
      rejects([&] { Controller c(file, f.options()); });
    });
    test("vector bounds sizes finite and target hold", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      c.write("gripper_joint", "target", vector({0.5}));
      for (int i = 0; i < 5; ++i) { f.advance(); cycle(c); }
      check(enabled(c, "gripper_joint"), "out-of-bounds target is passed to the module, which limits it");
      c.write("gripper_joint", "target", vector({0.01}));
      rejects([&] { c.write("gripper_joint", "target", vector({NAN})); });
      rejects([&] { c.write("gripper_joint", "target", vector({0, 0})); });
      for (int i = 0; i < 30; ++i) {
        f.advance();
        cycle(c);
      }
      check(enabled(c,"gripper_joint") && c.read("gripper_joint","target").vector[0]==0.01, "target must hold without refresh");
      c.enable("gripper_joint", false);
      f.advance(); cycle(c);
      auto id=c.write("gripper_joint", "target", vector({0.01})); f.advance(); cycle(c);
      check(!enabled(c,"gripper_joint"),"manual disable is not expiry rearm");
      check(!c.snapshot().outcomes[(c.snapshot().outcome_count-1)%32].applied && c.snapshot().outcomes[(c.snapshot().outcome_count-1)%32].sequence==id,"async rejection");
    });
    test("joint state follows model order", [] {
      Fixture f; Controller c(f.joint_config(),f.options());
      std::vector<std::string> expected;
      for(const auto &j:c.model()->joints()) if(j.nv==1 && (j.nq==1 || j.nq==2)) expected.push_back(j.name);
      check(expected.size()>1 && c.joint_state(c.q()).names==expected,"joint state is not in model order");
    });
    test("invalid command keeps accepted target", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      c.write("gripper_joint", "target", vector({0.01}));
      f.advance(); cycle(c);
      rejects([&] { c.write("gripper_joint", "target", vector({NAN})); });
      f.advance(); cycle(c);
      check(enabled(c,"gripper_joint") && c.read("gripper_joint","target").vector[0]==0.01, "invalid command changed target");
    });
    test("task motion and reset measured target", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      auto p = c.pose("tool_pose", "target");
      p.translation().x() += 0.08;
      kstone::PortValue v;
      v.pose = p;
      for (int i = 0; i < 100; ++i) {
        c.write("tool_pose", "target", v);
        f.advance();
        check(cycle(c), c.fault);
      }
      check(c.pose("tool_pose", "target").translation().x() > 0.04, "tool moved");
      c.reset_target("tool_pose"); f.advance(); cycle(c);
      check(c.read("tool_pose", "target").pose.isApprox(c.pose("tool_pose", "target")),
            "reset measured");
    });
    test("chassis velocity body frame timeout and rearm", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      for (int i = 0; i < 60; ++i) {
        c.write("chassis", "velocity", vector({0.1, 0.05, 0}));
        f.advance();
        check(cycle(c), c.fault);
      }
      check(c.base_pose().translation().x() > 0.04, "base moved");
      check(c.base_velocity().vx > 0.05, "velocity");
      for (int i = 0; i < 40; ++i) {
        f.advance();
        cycle(c);
      }
      check(c.base_velocity().vx == 0, "stop timeout");
      c.write("chassis", "velocity", vector({0, 0, 0}));
      f.advance();cycle(c);check(enabled(c,"chassis"), "rearm");
    });
    test("aggregate single-joint mapping atomic validation", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      c.joint_targets({"finger_joint"}, {0.02}); f.advance();cycle(c);
      c.joint_targets({"finger_joint", "unknown"}, {0.03, 0}); f.advance();cycle(c);
      check(c.read("gripper_joint", "target").vector[0] == 0.03, "unknown name must be ignored");
      rejects([&] {
        c.joint_targets({"finger_joint", "finger_joint"}, {0.01, 0.02});
      });
    });
    test("partial aggregate after recovery fills omitted joints from measured, not pre-fault targets", [] {
      Fixture f;auto file=f.config(false,false,[](Node &n) {
        auto gripper=*n.find("modules")->find("gripper_joint");
        gripper.set("joints",Node::list({"finger_joint","slide_x"}));
        n.set("modules",Node::map({{"chassis",*n.find("modules")->find("chassis")},{"gripper_joint",gripper}}));
      });
      Controller c(file,f.options());c.joint_targets({"finger_joint","slide_x"},{0.03,0.15});cycle(c);
      f.advance(0.2);check(!cycle(c),"fault missing");c.recover();f.advance();cycle(c);
      auto measured=take(c.model()->joint_position(c.q(),take(c.model()->joint_id("slide_x"))));
      c.joint_targets({"finger_joint"},{0.0});f.advance();cycle(c);
      check(std::abs(c.read("gripper_joint","target").vector[1]-measured)<1e-9,"omitted joint replayed pre-fault target");
      check(c.read("gripper_joint","target").vector[0]==0.0,"partial recovered target lost");
    });
    test("aggregate joint routing any subset and atomic validation", [] {
      Fixture f; Controller c(f.joint_config(),f.options());
      auto id=c.joint_targets({"slide_z","finger_joint","slide_y","slide_x"},{0.1,0.02,0.08,0.12});
      cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.02,0.12}).vector),"hand name mapping");
      check(c.read("axes","target").vector.isApprox(vector({0.08,0.1}).vector),"axes name mapping");
      int outcomes=0;for(const auto &o:c.snapshot().outcomes) if(o.sequence==id && o.applied) ++outcomes;
      check(outcomes==1,"aggregate must have one applied request outcome");
      const auto before=c.state().find("submitted_inputs")->as_number();
      rejects([&]{c.joint_targets({},{});});
      rejects([&]{c.joint_targets({"finger_joint","finger_joint"},{0.01,0.02});});
      rejects([&]{c.joint_targets({"finger_joint","slide_x"},{0.01});});
      rejects([&]{c.joint_targets({"finger_joint","slide_x"},{0.01,INFINITY});});
      check(c.state().find("submitted_inputs")->as_number()==before,"invalid aggregate submitted/renewed");
      f.advance();cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.02,0.12}).vector),"invalid message partially applied");
      c.joint_targets({"slide_x","finger_joint"},{0.07,0.03});f.advance();cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.03,0.07}).vector),"subset failed");
      check(c.read("axes","target").vector.isApprox(vector({0.08,0.1}).vector),"untouched subset changed");
      c.joint_targets({"unknown",""},{0.0,0.0});f.advance();cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.03,0.07}).vector),"all-unknown message changed target");
      std::vector<std::string> ignored;
      c.joint_targets({"finger_joint","slide_x","unknown"},{0.01,0.05,0},&ignored);f.advance();cycle(c);
      check(ignored==std::vector<std::string>{"unknown"},"ignored joint names not reported");
      check(c.read("hand","target").vector.isApprox(vector({0.01,0.05}).vector),"unknown name blocked the message");
      c.joint_targets({"slide_y","slide_z"},{0.06,100});f.advance();cycle(c);
      check(c.read("axes","target").vector[0]==0.06 && c.read("axes","target").vector[1]<100,"out-of-bounds not clamped");
    });
    test("aggregate and module targets last accepted wins in one owner cycle", [] {
      Fixture f;Controller c(f.joint_config(),f.options());
      c.write("axes","target",vector({0.03,0.04}));
      auto aggregate=c.joint_targets({"slide_x","slide_z","finger_joint","slide_y"},{0.1,0.12,0.02,0.08});
      auto newer=c.write("hand","target",vector({0.03,0.06}));
      cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.03,0.06}).vector),"old aggregate overrode newer module");
      check(c.read("axes","target").vector.isApprox(vector({0.08,0.12}).vector),"old module overrode newer aggregate");
      bool a=false,b=false;for(const auto &o:c.snapshot().outcomes) {a|=o.sequence==aggregate&&o.applied;b|=o.sequence==newer&&o.applied;}
      check(a&&b,"aggregate and newer module not applied together");
      // Coalescing retains one latest aggregate, never replays its older values.
      for(int i=0;i<100;++i) c.joint_targets({"finger_joint","slide_x"},{0.01,0.02});
      c.joint_targets({"finger_joint","slide_x"},{0.025,0.09});f.advance();cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.025,0.09}).vector),"coalesced latest lost");
      c.write("hand","target",vector({0.015,0.05}));f.advance();cycle(c);
      f.advance();cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.015,0.05}).vector),"consumed aggregate replayed");
    });
    test("aggregate ambiguous ownership only rejects aggregate routing", [] {
      Fixture f;Controller c(f.joint_config(true),f.options());
      rejects([&]{c.joint_targets({"finger_joint","slide_x","slide_y"},{0.02,0.1,0.05});});
      c.write("hand","target",vector({0.02,0.1}));cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.02,0.1}).vector),"overlap broke module-specific input");
      check(c.read("axes","target").vector.isZero(),"ambiguous aggregate partially applied");
    });
    test("aggregate runtime rejection preserves accepted targets", [] {
      Fixture f;auto options=f.options();options.command_timeout_s=0.06;
      Controller c(f.joint_config(),options);
      c.joint_targets({"finger_joint","slide_x","slide_y","slide_z"},{0.01,0.04,0.005,0.008});cycle(c);
      f.advance(0.03);c.enable("axes",false);
      auto module=c.write("hand","target",vector({0.015,0.06}));
      auto rejected=c.joint_targets({"finger_joint","slide_x","slide_y","slide_z"},{0.02,0.12,0.08,0.1});cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.015,0.06}).vector),"rejected newer aggregate superseded accepted module");
      check(c.read("axes","target").vector.isApprox(vector({0.005,0.008}).vector),"disabled aggregate partially applied");
      bool a=false,b=false;for(const auto &o:c.snapshot().outcomes) {a|=o.sequence==module&&o.applied;b|=o.sequence==rejected&&!o.applied;}
      check(a&&b,"runtime aggregate rejection changed accepted ordering");
      f.advance(0.02);c.enable("axes",true);c.home("axes");
      c.joint_targets({"finger_joint","slide_x","slide_y","slide_z"},{0.02,0.12,0.08,0.1});cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.015,0.06}).vector),"homing rejection partially wrote other module");
    });
    test("aggregate queued across fault recovery cannot replay", [] {
      Fixture f;Controller c(f.joint_config(),f.options());cycle(c);
      c.joint_targets({"finger_joint","slide_x","slide_y","slide_z"},{0.02,0.1,0.05,0.08});
      f.advance(0.2);cycle(c);
      check(c.fault.rfind("scheduler_discontinuity: gap",0)==0,"scheduler fault not detected");
      check(c.read("hand","target").vector.isZero()&&c.read("axes","target").vector.isZero(),"faulted aggregate partially applied");
      c.recover();f.advance();cycle(c);
      check(c.fault.empty()&&!enabled(c,"hand")&&!enabled(c,"axes"),"old aggregate replayed after recovery");
      c.joint_targets({"finger_joint","slide_x"},{0.03,0.08});f.advance();cycle(c);
      check(c.read("hand","target").vector.isApprox(vector({0.03,0.08}).vector)&&!enabled(c,"axes"),"fresh recovered subset failed or old axes target replayed");
    });
    test("aggregate delayed whole-request expiry", [] {
      Fixture f;auto options=f.options();options.command_timeout_s=0.04;
      Controller c(f.joint_config(),options);
      c.joint_targets({"finger_joint","slide_x","slide_y","slide_z"},{0.02,0.1,0.05,0.08});
      f.advance(0.03);cycle(c);
      check(c.read("hand","target").vector[0]==0.02,"fresh delayed aggregate failed");
      f.advance(0.02);cycle(c);
      check(enabled(c,"hand")&&c.read("hand","target").vector[0]==0.02,"accepted aggregate must hold");
      Fixture delayed;auto o=delayed.options();o.command_timeout_s=0.01;
      Controller stale(delayed.joint_config(),o);
      stale.joint_targets({"finger_joint","slide_x","slide_y","slide_z"},{0.02,0.1,0.05,0.08});
      delayed.advance(0.02);cycle(stale);
      check(stale.read("hand","target").vector.isZero()&&stale.read("axes","target").vector.isZero(),"expired batch partially applied");
    });
    test("runtime rate change follows the live period", [] {
      Fixture f;auto o=f.options();o.debug=kstone::DebugServerOptions{};o.debug->port=0;
      Controller c(f.joint_config(),o);cycle(c);
      int fd=::socket(AF_INET,SOCK_STREAM,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(c.debug_port());a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
      check(::connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof a)==0,"debug connect");
      const std::string body=R"({"op":"set_rate","rate_hz":5,"id":1})";
      unsigned char len[4]={0,0,0,static_cast<unsigned char>(body.size())};
      check(::write(fd,len,4)==4 && ::write(fd,body.data(),body.size())==ssize_t(body.size()),"debug send");
      for(int i=0;i<200 && c.dt()<0.1;++i){f.advance();cycle(c);}
      ::close(fd);
      check(std::abs(c.dt()-0.2)<1e-9 && c.discovery().find("rate_hz")->as_number()==5,"rate change not visible");
      f.advance(0.2);check(cycle(c) && c.fault.empty(),"slow rate tripped scheduler: "+c.fault);
      f.advance(1.0);check(!cycle(c) && c.fault.rfind("scheduler_discontinuity: gap",0)==0,"real gap at slow rate not detected");
    });
    test("scheduler latch recovery clears queued motion", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      cycle(c);
      f.advance(0.2);
      check(!cycle(c) && c.fault.rfind("scheduler_discontinuity: gap",0)==0, "latch");
      f.advance();
      check(!cycle(c), "stays latched");
      c.write("chassis", "velocity", vector({0.1, 0, 0}));
      c.recover();
      f.advance();
      check(cycle(c), c.fault);
      check(!enabled(c,"chassis") && c.base_velocity().vx == 0,
            "queued motion discarded");
      c.write("chassis", "velocity", vector({0, 0, 0}));
      f.advance();
      check(cycle(c), c.fault);
    });
    test("manual disable across fault remains safe and explicit enable can rearm", [] {
      Fixture f;Controller c(KSTONE_TEST_CONFIG,f.options());
      c.write("chassis","velocity",vector({0.1,0,0}));cycle(c);f.advance(0.2);check(!cycle(c),"fault missing");
      c.recover();auto disable=c.enable("chassis",false);f.advance();cycle(c);
      bool applied=false;for(const auto &out:c.snapshot().outcomes)if(out.sequence==disable && out.applied)applied=true;
      check(applied && !enabled(c,"chassis"),"fail-safe disable rejected across fault");
      c.write("chassis","velocity",vector({0.05,0,0}));f.advance();cycle(c);
      check(!enabled(c,"chassis"),"manual disable expiry-rearmed");
      c.enable("chassis",true);f.advance();cycle(c);
      check(!enabled(c,"chassis") && c.base_velocity().vx==0,"enable replayed stale motion");
      c.write("chassis","velocity",vector({0.05,0,0}));f.advance();cycle(c);
      check(enabled(c,"chassis"),"manual disable and recovery left module permanently disabled");
    });
    test("home all homes commanded modules and leaves gaze-held joints", [] {
      Fixture f;
      auto file=f.config(true,true,[](Node &n) {
        auto modules=*n.find("modules"), axes=*modules.find("gripper_joint");
        axes.set("joints",Node::list({"slide_x","slide_y"}));
        modules.set("axes",axes);
        modules.set("gaze",Node::map({{"type","gaze"},{"frame","tool_link"},
          {"joints",Node::list({"slide_x","slide_y"})},{"frames",Node::list({"finger_link"})}}));
        n.set("modules",modules);
      });
      Controller c(file,f.options());cycle(c);
      auto home=[&](const std::string &id) {
        for(std::size_t i=0;i<c.modules().size();++i) if(c.modules()[i]->id()==id) return std::string(c.snapshot().home_states[i].c_str());
        throw std::runtime_error("unknown module");
      };
      c.reset_home();f.advance();cycle(c);
      check(home("gripper_joint")=="idle" || home("gripper_joint").empty(),"home all moved a module nobody commanded");
      c.write("gripper_joint","target",vector({0.01}));f.advance();cycle(c);
      c.reset_home();f.advance();cycle(c);
      check(home("gripper_joint")!="idle" && !home("gripper_joint").empty(),"home all skipped commanded module");
      check(home("axes")=="idle" || home("axes").empty(),"home all took joints from gaze");
      c.home("axes");f.advance();cycle(c);
      check(home("axes")!="idle" && !home("axes").empty(),"single home blocked by gaze");
    });
    test("home and snapshot outputs", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options());
      c.home("gripper_joint");
      for (int i = 0; i < 5; ++i) {
        f.advance();
        cycle(c);
      }
      check(c.snapshot().home_states[1].bytes[0] != 0, "home state");
      rejects([&] { c.home("chassis"); });

      auto s = parse_json(json(c.state()));
      check(s.find("modules")->find("chassis")->find("outputs") != nullptr,
            "state outputs");
    });
    test("feedback partial stale recovery and limits", [] {
      Fixture f;
      Controller c(f.config(true, true), f.options(true));
      check(cycle(c) && c.fault.empty() && !c.joints_attached, "engine runs open loop before feedback");
      c.feedback_joints({"slide_x"}, {0});
      f.advance();
      check(cycle(c) && !c.joints_attached, "partial gate");
      auto j = c.joint_state(c.measured());
      for (std::size_t i = 0; i < j.names.size(); ++i) if (j.names[i] == "slide_x") j.positions[i] = 0.01;
      f.advance();
      check(attach(f, c, j) > 10 && c.written && c.fault.empty(), "attach waits for the feedback window");
      check(c.command_q().isApprox(c.measured(), 1e-9), "attach syncs command to measured without a jump");
      auto before = c.measured();
      rejects([&] { c.feedback_joints({"slide_x"}, {std::numeric_limits<double>::quiet_NaN()}); });
      check(c.measured().isApprox(before), "reject atomic");
      rejects([&] { c.feedback_joints({"other"}, {0}); });
      rejects([&] { c.feedback_joints({"slide_x", "slide_x"}, {0, 0}); });
      for (int i = 0; i < 20; ++i) {
        f.advance();
        cycle(c);
      }
      check(c.written && !c.joints_attached && c.fault.empty(), "stale detaches without pausing");
      check(attach(f, c, j) > 10 && c.written, c.fault);
    });
    test("finite out-of-range feedback stays raw while commands and FK synchronize to clipped state", [] {
      Fixture f;
      Controller c(f.config(true, true, [](Node &n) {
        auto modules = *n.find("modules");
        modules.set("posture", Node::map({{"type", "null_space"}, {"primary", "tool_pose"}}));
        n.set("modules", modules);
      }), f.options());
      auto j = c.joint_state(c.measured());
      for (std::size_t i = 0; i < j.names.size(); ++i) {
        if (j.names[i] == "slide_x") j.positions[i] = 0.35;
        if (j.names[i] == "slide_y") j.positions[i] = -0.35;
        if (j.names[i] == "finger_joint") j.positions[i] = -0.01;
      }
      attach(f, c, j);
      const auto at = c.measured().eval();
      const auto clipped = clipped_seed(c, at);
      check(!clipped.isApprox(at, 1e-12), "fixture must contain actual out-of-limit feedback");
      check(c.command_q().isApprox(clipped, 1e-12), "startup command must clip measured feedback to model limits");
      check(c.pose("tool_pose", "target").matrix().isApprox(frame_at(c, at, "tool_link", "base_link").matrix(), 1e-12), "measured FK display must remain raw while command/FK targets are clipped");
      for (int k = 0; k < 5; ++k) {
        f.advance(); c.feedback_joints(j.names, j.positions); cycle(c);
        check(c.measured().isApprox(at, 1e-12) && c.command_q().isApprox(clipped, 1e-12), "raw feedback must remain displayed while default IK holds the clipped FK target");
      }
      rejects([&] { c.feedback_joints({}, {}); });
      rejects([&] { c.feedback_joints({"slide_x"}, {}); });
      rejects([&] { c.feedback_joints({"slide_x", "slide_x"}, {0, 0}); });
      rejects([&] { c.feedback_joints({"other"}, {0}); });
      rejects([&] { c.feedback_joints({"slide_x"}, {std::numeric_limits<double>::infinity()}); });
      rejects([&] { c.feedback_joints({"slide_x"}, {std::numeric_limits<double>::quiet_NaN()}); });
      rejects([&] { c.feedback_joints({"slide_x"}, {0}, -0.01); });
      rejects([&] { c.feedback_joints({"slide_x"}, {0}, 0.3); });
      rejects([&] { c.feedback_joints({"slide_x"}, {0}, std::numeric_limits<double>::quiet_NaN()); });
      rejects([&] { c.feedback_joints({"slide_x"}, {0}, std::numeric_limits<double>::infinity()); });
      rejects([&] { c.feedback_joints({"slide_z", "other"}, {0.1, std::numeric_limits<double>::quiet_NaN()}); });
      rejects([&] { c.feedback_joints({"slide_x"}, {0}, 0.01); });
      f.advance(); c.feedback_joints({"slide_z"}, {0}); cycle(c);
      check(c.measured().isApprox(at, 1e-12), "invalid input is atomic and valid partial feedback survives other outliers");
      auto goal = c.pose("tool_pose", "target");
      goal.translation().x() -= 0.1;
      kstone::PortValue target; target.pose = goal;
      c.write("tool_pose", "target", target);
      auto previous = c.command_q().eval();
      const auto x = take(c.model()->joint_id("slide_x"));
      const auto y = take(c.model()->joint_id("slide_y"));
      const auto finger = take(c.model()->joint_id("finger_joint"));
      const auto ix = c.model()->joints()[x].idx_q;
      const auto iy = c.model()->joints()[y].idx_q;
      const auto ih = c.model()->joints()[finger].idx_q;
      for (int k = 0; k < 15; ++k) {
        f.advance(); c.feedback_joints(j.names, j.positions); cycle(c);
        check(c.command_q()[ix] <= previous[ix] + 1e-9 && previous[ix] - c.command_q()[ix] <= 0.3 * c.dt() + 1e-9, "frame target returns gradually, never outward");
        check(std::abs(c.command_q()[iy] - clipped[iy]) < 1e-9 && std::abs(c.command_q()[ih] - clipped[ih]) < 1e-9, "omitted axes stay at their clipped synchronization state");
        previous = c.command_q(); j = c.joint_state(previous);
      }
      check(previous[ix] < clipped[ix] && c.fault.empty(), "normal frame target moves from the clipped state");
      for (int k = 0; k < 20; ++k) { f.advance(); cycle(c); }
      check(!c.joints_attached, "feedback loss detaches");
      for (std::size_t i = 0; i < j.names.size(); ++i) if (j.names[i] == "slide_x") j.positions[i] = 0.38;
      attach(f, c, j);
      const auto reattached = c.measured().eval();
      const auto reattached_clipped = clipped_seed(c, reattached);
      check(c.command_q().isApprox(reattached_clipped, 1e-12), "reattach first command equals clipped feedback");
      check(c.pose("tool_pose", "target").matrix().isApprox(frame_at(c, reattached, "tool_link", "base_link").matrix(), 1e-12), "reattach measured FK display remains raw");
      for (int k = 0; k < 5; ++k) {
        f.advance(); c.feedback_joints(j.names, j.positions); cycle(c);
        check(c.measured().isApprox(reattached, 1e-12) && c.command_q().isApprox(reattached_clipped, 1e-12), "reattach clips commands but neither feedback nor expired target is replayed");
      }
    });
    test("aggregate partial target preserves omitted clipped joint targets", [] {
      Fixture f;
      Controller c(f.joint_config(), f.options());
      auto j = c.joint_state(c.measured());
      for (std::size_t i = 0; i < j.names.size(); ++i) {
        if (j.names[i] == "slide_x") j.positions[i] = 0.35;
        if (j.names[i] == "finger_joint") j.positions[i] = -0.01;
      }
      attach(f, c, j);
      const auto at = c.measured().eval();
      const auto clipped = clipped_seed(c, at);
      check(c.command_q().isApprox(clipped, 1e-12), "aggregate startup clips command seed");
      c.joint_targets({"slide_x"}, {0.2});
      for (int k = 0; k < 15; ++k) {
        f.advance(); c.feedback_joints(j.names, j.positions); cycle(c);
        const auto state = c.joint_state(c.command_q());
        for (std::size_t i = 0; i < state.names.size(); ++i)
          if (state.names[i] == "finger_joint") check(std::abs(state.positions[i]) < 1e-12, "omitted joint retains clipped synchronization target");
        j = state;
      }
      check(!c.command_q().isApprox(clipped, 1e-9) && c.fault.empty(), "aggregate explicit target moves without replacing omitted clipped target");
    });
    if (std::getenv("KSTONE_LOOKIE_CONFIG") && std::getenv("KSTONE_LOOKIE_URDF"))
      test("Lookie deployed combination: 32 independent joints clip and run automatic IK at startup and reattach", [] {
        Fixture f;
        auto n = take(kstone::load_yaml(std::getenv("KSTONE_LOOKIE_CONFIG")));
        const std::filesystem::path urdf = std::getenv("KSTONE_LOOKIE_URDF");
        auto model = *n.find("model");
        model.set("file", urdf.string());
        model.set("package_dirs", Node::list({urdf.parent_path().parent_path().parent_path().string()}));
        n.set("model", model);
        auto modules = *n.find("modules");
        auto collision = *modules.find("self_collision");
        collision.set("urdf", urdf.string());
        modules.set("self_collision", collision);
        n.set("modules", modules);
        n.set("realtime", false);
        n.set("thread", Node::map({{"priority", 0}, {"lock_memory", false}}));
        const auto path = f.directory / "lookie.yaml";
        std::ofstream(path) << json(n);
        Controller c(path.string(), f.options());
        std::set<std::string> followers;
        for (const auto &m : c.model()->mimics()) followers.insert(m.follower);
        const auto published = c.joint_state(c.measured());
        JointValues j;
        for (std::size_t i = 0; i < published.names.size(); ++i) {
          if (followers.count(published.names[i])) continue;
          j.names.push_back(published.names[i]);
          j.positions.push_back(published.positions[i]);
        }
        check(j.names.size() == 32, "actual model must expose 32 independent non-wheel joints");
        const auto bridge_path = std::filesystem::path(std::getenv("KSTONE_LOOKIE_CONFIG")).parent_path() / "driver_joint_bridge.yaml";
        const auto bridge = take(kstone::load_yaml(bridge_path.string()));
        std::set<std::string> bridged;
        for (const auto &module : bridge.find("modules")->items()) {
          if (const auto *map = module.find("joint_map"))
            for (const auto &entry : map->entries()) bridged.insert(entry.second.as_string());
          else
            for (const auto &name : module.find("joints")->items()) bridged.insert(name.as_string());
        }
        check(bridged == std::set<std::string>(j.names.begin(), j.names.end()), "independent command names must exactly match the actual driver bridge");
        auto set_position = [&](const std::string &name, double value) {
          auto it = std::find(j.names.begin(), j.names.end(), name);
          check(it != j.names.end(), "missing independent joint " + name);
          j.positions[std::size_t(it - j.names.begin())] = value;
        };
        set_position("head_joint1", 0.949);
        set_position("l_arm_joint4", 0.08217);
        set_position("r_arm_joint4", -0.00487);
        const auto base = c.base_pose();
        auto feed = [&] {
          f.advance();
          c.feedback_joints(j.names, j.positions);
          c.feedback_base(base);
          cycle(c);
          check(c.fault.empty(), c.fault);
        };
        auto raw_seed = [&] {
          Eigen::VectorXd q = c.measured();
          for (std::size_t i = 0; i < j.names.size(); ++i) {
            const auto &joint = c.model()->joints()[take(c.model()->joint_id(j.names[i]))];
            q[joint.idx_q] = j.positions[i];
          }
          return q;
        };
        auto check_raw_fk = [&] {
          for (const auto &entry : modules.entries()) {
            if (entry.second.find("type")->as_string() != "frame_target") continue;
            const auto *reference = entry.second.find("reference");
            const auto expected = frame_at(c, c.measured(), entry.second.find("frame")->as_string(),
                                           reference ? reference->as_string() : "");
            check(c.pose(entry.first, "target").matrix().isApprox(expected.matrix(), 1e-10),
                  "measured FK display must remain raw: " + entry.first);
          }
        };
        double max_automatic_delta = 0.0;
        const auto initial = clipped_seed(c, raw_seed());
        auto validate = [&](const Eigen::VectorXd &clipped) {
          check(bool(c.model()->check_position_limits(c.command_q())), "command must obey unchanged model limits");
          const auto feedback = c.joint_state(c.measured());
          const auto command = c.joint_state(c.command_q());
          for (std::size_t i = 0; i < j.names.size(); ++i) {
            const auto measured = std::find(feedback.names.begin(), feedback.names.end(), j.names[i]);
            check(measured != feedback.names.end(), "missing measured joint");
            check(std::abs(feedback.positions[std::size_t(measured - feedback.names.begin())] - j.positions[i]) < 1e-12,
                  "displayed feedback must remain raw: " + j.names[i]);
            const auto output = std::find(command.names.begin(), command.names.end(), j.names[i]);
            check(output != command.names.end(), "missing independent command joint");
            const auto id = take(c.model()->joint_id(j.names[i]));
            max_automatic_delta = std::max(max_automatic_delta,
                std::abs(command.positions[std::size_t(output - command.names.begin())] -
                         take(c.model()->joint_position(clipped, id))));
          }
        };
        feed();
        check_raw_fk();
        validate(initial);
        check(enabled(c, "gaze") && enabled(c, "com") && enabled(c, "self_collision"), "automatic modules retain their enabled startup state");
        for (int k = 1; k < 35; ++k) {
          // Sensed arm motion requests gaze through existing idle-claim arbitration, not a target.
          if (k == 1) set_position("l_arm_joint2", 0.01);
          const auto seed = clipped_seed(c, raw_seed());
          feed(); validate(seed);
        }
        check(c.joints_attached && c.base_attached && c.written, "real configuration attaches offline");
        check(max_automatic_delta > 1e-8, "automatic IK/gaze must operate without a fresh target");
        c.joint_targets({"head_joint1"}, {0.85});
        const auto before = c.command_q().eval();
        for (int k = 0; k < 10; ++k) {
          const auto previous = c.command_q().eval();
          feed();
          for (const auto &name : j.names) {
            const auto id = take(c.model()->joint_id(name));
            const double delta = take(c.model()->joint_position(c.command_q(), id)) - take(c.model()->joint_position(previous, id));
            check(std::abs(delta) < 0.05, "target must not snap any independent joint to a boundary");
          }
          const auto command = c.joint_state(c.command_q());
          for (std::size_t i = 0; i < j.names.size(); ++i) {
            const auto it = std::find(command.names.begin(), command.names.end(), j.names[i]);
            j.positions[i] = command.positions[std::size_t(it - command.names.begin())];
          }
        }
        const auto head = take(c.model()->joint_id("head_joint1"));
        check(std::abs(take(c.model()->joint_position(c.command_q(), head)) - 0.85) < std::abs(take(c.model()->joint_position(before, head)) - 0.85), "explicit valid target progresses toward the requested in-limit position");
        c.write("chassis", "velocity", vector({0, 0, 0}));
        feed();
        for (int k = 0; k < 20; ++k) { f.advance(); c.feedback_base(base); cycle(c); }
        check(!c.joints_attached, "joint feedback loss detaches");
        set_position("head_joint1", 0.97);
        set_position("l_arm_joint4", 0.08217);
        set_position("r_arm_joint4", -0.00487);
        const auto reattached = clipped_seed(c, raw_seed());
        feed();
        check_raw_fk();
        validate(reattached);
        for (int k = 1; k < 35; ++k) {
          if (k == 5) set_position("l_arm_joint2", 0.04);
          const auto seed = clipped_seed(c, raw_seed());
          feed(); validate(seed);
        }
        check(enabled(c, "gaze") && enabled(c, "com") && enabled(c, "self_collision"), "reattach does not disable automatic modules pending a fresh target");
        for (std::size_t i = 0; i < c.modules().size(); ++i)
          if (c.modules()[i]->id() == "com" || c.modules()[i]->id() == "self_collision")
            check(c.snapshot().engine.modules[i].running, "reattached protection runs without a fresh target");
        check(c.joints_attached && c.written, "reattach resumes clipped commands and normal automatic IK");
        std::cout << "Lookie independent_joints=32 max_automatic_delta=" << max_automatic_delta << '\n';
      });
    test("continuous feedback coordinates",
         [] {
           Fixture f;
           auto urdf = f.directory / "continuous.urdf";
           std::ofstream(urdf)
               << "<robot name='continuous'><link name='base_link'/><link "
                  "name='tip'/><joint name='spin' type='continuous'><parent "
                  "link='base_link'/><child link='tip'/><axis xyz='0 0 "
                  "1'/><limit effort='1' velocity='1'/></joint></robot>";
           auto path = f.directory / "continuous.yaml";
           std::ofstream(path) << json(
               Node::map({{"model", Node::map({{"file", urdf.string()}})},
                          {"rate_hz", 50},
                          {"robot", Node::map({{"type", "loopback"}})},
                          {"modules", Node::map()}}));
           Controller c(path.string(), f.options(true));
           c.feedback_joints({"spin"}, {1.2});
           check(cycle(c), c.fault);
           check(std::abs(c.joint_state(c.q()).positions[0] - 1.2) < 1e-9,
                 "continuous position");
         });
    test("free root feedback without chassis", [] {
      Fixture f;
      Controller c(f.config(false, true), f.options(true, true));
      auto j = c.joint_state(c.measured());
      check(attach(f, c, j) > 0 && !c.base_attached, "base waits for odometry");
      int cycles = 0;
      for (; cycles < 30 && !c.base_attached; ++cycles) {
        f.advance();
        c.feedback_joints(j.names, j.positions);
        c.feedback_base(make_pose({1, 2, 0}, {2, 0, 0, 0}));
        check(cycle(c), c.fault);
      }
      check(c.base_attached && cycles > 10, "base attaches after the feedback window");
      check(c.base_pose().translation().isApprox(Eigen::Vector3d(1, 2, 0)),
            "measured root");
    });
    test("unsupported floating-root actuation is latched", [] {
      Fixture f;
      auto file = f.config(false, true, [](Node &n) {
        auto modules = *n.find("modules");
        auto tool = *modules.find("tool_pose");
        tool.set("reference", "");
        tool.set("joints", Node::list());
        modules.set("tool_pose", tool);
        n.set("modules", modules);
      });
      Controller c(file, f.options(true));
      auto j = c.joint_state(c.measured());
      check(attach(f, c, j) > 0 && c.written, "attach before commanding");
      kstone::PortValue value;
      value.pose = c.pose("tool_pose", "target");
      value.pose.translation().x() += 0.1;
      c.write("tool_pose", "target", value);
      f.advance();
      check(!cycle(c) &&
                !c.fault.empty(),
            "unrepresentable root command must be rejected");
      f.advance();
      check(!cycle(c), "cycle fault must remain latched");
    });
    test("chassis body twist at measured world yaw", [] {
      Fixture f;
      Controller c(KSTONE_TEST_CONFIG, f.options(true, true));
      auto j = c.joint_state(c.measured());
      attach(f, c, j);
      const auto odom =
          make_pose({1, 2, 0}, {std::sqrt(0.5), 0, 0, std::sqrt(0.5)});
      for (int i = 0; i < 30 && !c.base_attached; ++i) {
        f.advance();
        c.feedback_joints(j.names, j.positions);
        c.feedback_base(odom);
        check(cycle(c), c.fault);
      }
      for (int i = 0; i < 9; ++i) {
        c.write("chassis", "velocity", vector({0.1, 0, 0}));
        f.advance();
        c.feedback_joints(j.names, j.positions);
        c.feedback_base(odom);
        check(cycle(c), c.fault);
      }
      auto root = c.base_pose();
      auto velocity = c.base_velocity();
      Eigen::Vector3d world_velocity =
          root.linear() * Eigen::Vector3d(velocity.vx, velocity.vy, 0);
      check(world_velocity.y() > 0.05 && std::abs(world_velocity.x()) < 1e-6,
            "body-forward velocity must map to world-y at pi/2 yaw");
      check(root.translation().isApprox(Eigen::Vector3d(1, 2, 0)),
            "fresh measured odometry must not be replaced by dead reckoning");
    });
    test("chassis odometry stale future duplicate and root transform", [] {
      Fixture f;
      auto file = f.config(false, false, [](Node &n) {
        auto ms = *n.find("modules");
        auto base = *ms.find("chassis");
        base.set("center_in_root",
                 Node::map({{"position", Node::list({0.2, 0.0, 0.0})}}));
        ms.set("chassis", base);
        n.set("modules", ms);
      });
      Controller c(file, f.options(true, true));
      auto j = c.joint_state(c.measured());
      c.feedback_joints(j.names, j.positions);
      c.feedback_base(
          make_pose({1, 2, 0}, {std::sqrt(0.5), 0, 0, std::sqrt(0.5)}));
      f.advance();
      check(cycle(c), c.fault);
      check(
          c.base_pose().translation().isApprox(Eigen::Vector3d(1, 2, 0), 1e-6),
          "center to root transform");
      check(c.chassis_pose().translation().isApprox(Eigen::Vector3d(1, 2.2, 0), 1e-6),
            "chassis feedback must copy engine control-center estimate, not root or target");
      const auto base_info=std::find_if(c.modules().begin(),c.modules().end(),
          [](const auto &m) { return m->id()=="chassis"; });
      check(base_info!=c.modules().end() && (*base_info)->chassis_omni &&
            (*base_info)->chassis_frame=="base_link", "immutable chassis metadata");
      rejects(
          [&] { c.feedback_base(make_pose({1, 2, 0}, {1, 0, 0, 0}), -0.1); });
      rejects(
          [&] { c.feedback_base(make_pose({1, 2, 0}, {1, 0, 0, 0}), 0.3); });
      rejects([&] { c.feedback_base(make_pose({1, 2, 1}, {1, 0, 0, 0})); });
      for (int i = 0; i < 20; ++i) {
        f.advance();
        c.feedback_joints(j.names, j.positions);
        cycle(c);
      }
      check(c.joints_attached && !c.base_attached && c.base_velocity().vx == 0, "stale base gate");
      c.feedback_base(make_pose({1.5, 2, 0}, {1, 0, 0, 0}));
      rejects([&] { c.feedback_base(make_pose({1.5, 2, 0}, {1, 0, 0, 0})); });
      f.advance();
      check(cycle(c), c.fault);
      check(std::abs(c.base_pose().translation().x() - 1.5) < 1e-6,
            "fresh odom recovery");
    });

    test("owner progresses while snapshots and copied logs are not consumed", [] {
      Fixture f; Options o;
      Controller c(KSTONE_TEST_CONFIG,o); c.start(); c.poll();
      auto before=c.snapshot().engine.stats.cycles;
      std::string component="plugin", message="temporary view";
      for(int i=0;i<2000;++i) c.logs()->write({kstone::LogLevel::warn,component,message,123});
      component="changed";message="changed";
      std::this_thread::sleep_for(std::chrono::milliseconds(260));
      c.poll();check(c.snapshot().engine.stats.cycles>=before+10,"consumer stalled engine owner");
      check(c.logs()->dropped>=1744,"logs are fixed at 256 records");
      CopiedLog record;check(c.logs()->pop(record),"log present");
      check(std::string(record.message.c_str())=="temporary view" && std::string(record.component.c_str())=="plugin","log borrowed views were not copied");
    });
    test("bounded request overload and coalesced latest target", [] {
      Fixture f; Controller c(KSTONE_TEST_CONFIG,f.options());
      for(int i=0;i<64;++i)c.enable("gripper_joint",true);
      rejects([&]{c.enable("gripper_joint",true);});
      std::uint64_t last=0;
      for(int i=0;i<2000;++i)last=c.write("gripper_joint","target",vector({i==1999?0.03:0.01}));
      cycle(c);
      check(std::abs(c.read("gripper_joint","target").vector[0]-0.03)<1e-9,"latest target lost");
      check(c.snapshot().outcome_count==65,"input overload must coalesce to one applied value");
      check(c.snapshot().outcomes[(c.snapshot().outcome_count-1)%32].sequence==last,"latest sequence");
      check(c.state().find("rejected_requests")->as_number()==1,"bounded queue rejection counter");
    });
    test("delayed command expires at original receipt not owner application", [] {
      Fixture f;Controller c(KSTONE_TEST_CONFIG,f.options());
      auto id=c.write("gripper_joint","target",vector({0.03})); f.advance(0.6);cycle(c);
      check(c.read("gripper_joint","target").vector[0]==0,"expired input applied");
      const auto &out=c.snapshot().outcomes[(c.snapshot().outcome_count-1)%32];
      check(out.sequence==id && !out.applied,"expired request outcome");
    });
    test("full per-joint freshness survives coalesced partial feedback", [] {
      Fixture f;Controller c(f.config(true,true),f.options(true));
      auto j=c.joint_state(c.measured());attach(f,c,j);
      for(int i=0;i<10;++i){f.advance();cycle(c);}
      c.feedback_joints({"slide_x"},{0.01});f.advance();cycle(c);
      f.advance(0.08);cycle(c);check(!c.joints_attached,"partial feedback renewed unrelated joints");
      f.advance();check(attach(f,c,j)>0 && c.written,c.fault);
    });
    test("finite logical duration completes safely without ROS consumer", [] {
      Fixture f;Options o;o.duration_s=0.2;
      Controller c(KSTONE_TEST_CONFIG,o);c.write("chassis","velocity",vector({0.1,0,0}));c.start();
      auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
      do { c.poll();if(c.fault=="duration_complete")break;std::this_thread::sleep_for(std::chrono::milliseconds(1)); } while(std::chrono::steady_clock::now()<deadline);
      check(c.fault=="duration_complete" && !c.written,"finite completion safe snapshot");
      check(std::abs(c.snapshot().engine.stats.time-0.2)<1e-8,"logical duration");
      check(c.velocity().isZero() && c.base_velocity().vx==0,"completion zero output");
      rejects([&]{c.recover();});c.stop();c.stop();rejects([&]{c.start();});
      o.duration_s=-1;rejects([&]{Controller invalid(KSTONE_TEST_CONFIG,o);});
      o.duration_s=INFINITY;rejects([&]{Controller invalid(KSTONE_TEST_CONFIG,o);});
    });
    test("homing is exempt from command lease timeout", [] {
      Fixture f; auto o=f.options();o.command_timeout_s=0.001;
      auto file=f.config(false,false,[](Node &n) {
        auto modules=*n.find("modules");auto tool=*modules.find("tool_pose");
        tool.set("home",Node::map({{"position",Node::list({0.2,0.0,0.35})}}));
        modules.set("tool_pose",tool);n.set("modules",modules);
      });
      Controller c(file,o);c.home("tool_pose");cycle(c);
      for(int i=0;i<8;++i){f.advance();cycle(c);}
      // Home waiting/executing owns its own lifecycle; no ROS keepalive.
      const auto snapshot_state=c.state();auto state=snapshot_state.find("modules")->find("tool_pose");
      const auto h=state->find("home_state")->as_string();
      check(h=="succeeded" || h=="executing" || h=="waiting","homing state: "+h);
      if(h=="waiting" || h=="executing")check(enabled(c,"tool_pose"),"homing expired");
    });
    test("fault recovery drops old targets including explicit enable replay", [] {
      Fixture f;Controller c(KSTONE_TEST_CONFIG,f.options());
      c.write("gripper_joint","target",vector({0.03}));cycle(c);
      f.advance(0.2);check(!cycle(c),"scheduler fault");
      auto old=c.write("gripper_joint","target",vector({0.02}));c.recover();f.advance();cycle(c);
      check(!enabled(c,"gripper_joint"),"recovery replays command");
      c.enable("gripper_joint",true);f.advance();cycle(c);
      check(!enabled(c,"gripper_joint"),"explicit enable replays pre-fault target");
      bool discarded=false;for(const auto &out:c.snapshot().outcomes)if(out.sequence==old && !out.applied)discarded=true;
      check(discarded,"old command lacks discarded outcome");
      c.write("gripper_joint","target",vector({0.0}));f.advance();cycle(c);
      check(enabled(c,"gripper_joint"),"new post-recovery command does not rearm");
    });
    test("integrated chassis pose_target and velocity share engine owner cycle", [] {
      Fixture f;Controller c(KSTONE_TEST_CONFIG,f.options());
      auto id=c.write("chassis","pose_target",vector({0.1,0,0}));cycle(c);
      for(int i=0;i<8;++i){f.advance();cycle(c);}
      check(c.base_pose().translation().x()>0,"pose_target did not move root");
      check(c.snapshot().engine.stats.cycles>=9 && c.snapshot().outcomes[0].sequence==id,"chassis not in engine snapshot");
      c.write("chassis","velocity",vector({0,0,0}));f.advance();cycle(c);
      check(c.read("chassis","velocity").vector.isZero(),"velocity port switch");
    });
    test("engine driver and log production continue without any ROS consumer", [] {
      Fixture f;Controller seed(KSTONE_TEST_CONFIG,f.options());
      auto r=std::make_shared<InstrumentedRobot>(seed,kstone::Context::defaults().clock);
      Options o;o.robot=r;Controller c(KSTONE_TEST_CONFIG,o);
      r->logs=c.logs();auto initial=r->reads.load();c.start();
      std::this_thread::sleep_for(std::chrono::milliseconds(260));c.stop();c.poll();
      check(r->reads>=initial+10 && r->writes>=10,"engine driver stalled with consumer: reads="+std::to_string(r->reads)+" writes="+std::to_string(r->writes)+" failed="+std::to_string(c.snapshot().engine.stats.failed_cycles)+" "+c.fault);
      auto consumer=std::hash<std::thread::id>{}(std::this_thread::get_id());
      check(r->read_thread==r->write_thread && r->read_thread!=consumer,"ROS thread owns driver calls");
      check(c.logs()->dropped>1000,"owner log overload is not bounded/dropping");
      CopiedLog record;check(c.logs()->pop(record),"copied owner log missing");
      check(std::string(record.message.c_str())=="owned temporary log view","owner retained borrowed log view");
    });
    test("failed-only before observer latches and recovery cannot replay", [] {
      Fixture f;Controller seed(KSTONE_TEST_CONFIG,f.options());
      auto r=std::make_shared<InstrumentedRobot>(seed,f.clock);
      auto o=f.options();o.robot=r;Controller c(KSTONE_TEST_CONFIG,o);
      c.write("gripper_joint","target",vector({0.03}));cycle(c);
      r->fail_read=true;
      await(c,[&]{return c.snapshot().engine.stats.failed_cycles>=6 && !c.written;});
      check(!c.written && c.velocity().isZero() && c.base_velocity().vx==0,"failed cycles exported unsafe output");
      auto stale=c.write("gripper_joint","target",vector({0.02}));c.recover();
      r->fail_read=false;f.advance();cycle(c);
      check(!enabled(c,"gripper_joint"),"failed cycle recovery replayed command");
      bool discarded=false;for(const auto &out:c.snapshot().outcomes)if(out.sequence==stale && !out.applied)discarded=true;
      check(discarded,"generation-discarded motion has no outcome");
      c.write("gripper_joint","target",vector({0.0}));f.advance();cycle(c);
      check(enabled(c,"gripper_joint"),"fresh post-failure command not applied");
    });
    test("driver write failure latches even when asynchronous logs are full", [] {
      Fixture f;Controller seed(KSTONE_TEST_CONFIG,f.options());
      auto r=std::make_shared<InstrumentedRobot>(seed,f.clock);
      auto o=f.options();o.robot=r;Controller c(KSTONE_TEST_CONFIG,o);
      c.write("gripper_joint","target",vector({0.03}));cycle(c);
      r->logs=c.logs();r->fail_write=true;
      await(c,[&]{return c.snapshot().engine.stats.failed_cycles>0 && !c.written;});
      check(c.logs()->dropped>0 && c.velocity().isZero(),"full log sink obscured write failure");
      c.poll();check(!c.written && c.base_velocity().vx==0,"failed write exposed output");
      c.write("gripper_joint","target",vector({0.02}));c.recover();r->fail_write=false;
      f.advance();cycle(c);c.enable("gripper_joint",true);f.advance();cycle(c);
      check(!enabled(c,"gripper_joint"),"write failure recovery replayed old motion");
    });
    test("stale driver feedback clears targets and requires recovery", [] {
      Fixture f;Controller seed(KSTONE_TEST_CONFIG,f.options());
      auto r=std::make_shared<InstrumentedRobot>(seed,f.clock);
      auto o=f.options();o.robot=r;Controller c(KSTONE_TEST_CONFIG,o);
      c.write("gripper_joint","target",vector({0.03}));cycle(c);r->unknown_stamp=true;
      await(c,[&]{return c.fault=="driver_stale_feedback";});
      check(!c.written && !enabled(c,"gripper_joint"),"stale driver feedback did not clear motion");
      r->unknown_stamp=false;cycle(c);check(!c.written,"sensing restart bypassed recovery");
      c.recover();cycle(c);c.enable("gripper_joint",true);cycle(c);
      check(!enabled(c,"gripper_joint"),"stale driver feedback replayed old target");
    });
    test("blocked engine cycle suppresses telemetry and latches on return", [] {
      Fixture f;Controller seed(KSTONE_TEST_CONFIG,f.options());
      auto r=std::make_shared<InstrumentedRobot>(seed,f.clock);
      auto o=f.options();o.robot=r;Controller c(KSTONE_TEST_CONFIG,o);
      struct Release {std::shared_ptr<InstrumentedRobot> r;~Release(){r->blocked=false;}} release{r};
      c.write("chassis","velocity",vector({0.1,0,0}));cycle(c);
      r->blocked=true;await(c,[&]{return r->waiting.load();});f.advance(0.2);c.poll();
      check(!c.written && c.base_velocity().vx==0,"blocked owner exposed stale output");
      r->blocked=false;await(c,[&]{return c.fault.rfind("scheduler_stall: cycle",0)==0;});
      check(!c.written && c.velocity().isZero(),"overrun result published safe");
      c.stop();check(!r->waiting,"stop failed to join released driver");
    });
    test("finite duration terminates even with persistent driver read failures", [] {
      Fixture f;Controller seed(KSTONE_TEST_CONFIG,f.options());
      auto r=std::make_shared<InstrumentedRobot>(seed,f.clock);
      auto o=f.options();o.robot=r;o.duration_s=0.2;
      Controller c(KSTONE_TEST_CONFIG,o);r->fail_read=true;c.start();
      await(c,[&]{return c.fault=="duration_complete";});c.stop();
      check(!c.written && c.velocity().isZero(),"persistent failure completion unsafe");
      check(c.snapshot().engine.stats.failed_cycles>=10 && r->reads<20,"failed runs renewed finite duration");
    });
    test("chassis timeout uses original receipt and empty modules reset safely", [] {
      Fixture f;Controller c(KSTONE_TEST_CONFIG,f.options());
      auto id=c.write("chassis","velocity",vector({0.1,0,0}));f.advance(0.3);cycle(c);
      bool rejected=false;for(const auto &out:c.snapshot().outcomes)if(out.sequence==id && !out.applied)rejected=true;
      check(rejected && c.base_velocity().vx==0,"chassis timeout renewed at application");
      c.write("chassis","pose_target",vector({0.1,0,0}));f.advance();cycle(c);
      for(int i=0;i<16;++i){f.advance();cycle(c);}
      check(std::abs(c.read("chassis","pose_target").vector[0]-0.1)<1e-9 && c.base_pose().translation().x()<0.09,
            "expired accepted pose_target was replaced by measured pose");
      auto file=f.config(true,true,[](Node &n){n.set("modules",Node::map());});
      Controller empty(file,f.options());auto request=empty.reset_home();cycle(empty);
      check(empty.snapshot().outcomes[0].sequence==request && empty.snapshot().outcomes[0].applied,"empty module reset dereferenced a module");
    });
    test("fresh odometry remains valid across many paused engine cycles", [] {
      Fixture f;Controller c(KSTONE_TEST_CONFIG,f.options(true,true));
      auto j=c.joint_state(c.measured());c.feedback_joints(j.names,j.positions);
      c.feedback_base(make_pose({1,2,0},{1,0,0,0}));cycle(c);
      f.advance(0.2);cycle(c);
      for(int i=0;i<20;++i) {
        f.advance();c.feedback_joints(j.names,j.positions);
        c.feedback_base(make_pose({1+0.001*i,2,0},{1,0,0,0}));cycle(c);
      }
      check(!c.written && c.fresh,"paused owner rejected fresh sensing");
      check(std::abs(c.base_pose().translation().x()-1.019)<1e-8,"paused measured root became old estimated pose");
      c.recover();f.advance();c.feedback_joints(j.names,j.positions);
      c.feedback_base(make_pose({1.02,2,0},{1,0,0,0}));check(cycle(c),c.fault);
      check(std::abs(c.base_pose().translation().x()-1.02)<1e-8,"driver odometry did not resume logical clock");
    });
    test("continuous driver odometry cannot advance chassis beyond engine cycle time", [] {
      Fixture f;Controller c(KSTONE_TEST_CONFIG,f.options(true,true));
      cycle(c); // Begin with missing sensing and held engine cycles.
      auto j=c.joint_state(c.measured());
      double maximum_vx=0;
      for(int i=0;i<80;++i) {
        f.advance();c.feedback_joints(j.names,j.positions);
        c.feedback_base(make_pose({0,0,0},{1,0,0,0}));
        c.write("chassis","velocity",vector({0.08,0,0}));
        check(cycle(c),c.fault);
        maximum_vx=std::max(maximum_vx,c.base_velocity().vx);
        auto state=c.state();
        const auto *outputs=state.find("modules")->find("chassis")->find("outputs");
        check(!outputs->find("fault")->as_bool(),"fresh odometry caused chassis scheduler fault");
      }
      check(maximum_vx>0.005,"fresh driver command never produced nonzero chassis output");
    });
    test("sensing loss detaches output and reattach rebases targets to measured", [] {
      Fixture f;Controller c(f.config(true,true),f.options(true));
      auto j=c.joint_state(c.measured());
      check(attach(f,c,j)>0 && c.written,c.fault);
      c.write("gripper_joint","target",vector({0.03}));f.advance();cycle(c);
      for(int i=0;i<15;++i){f.advance();cycle(c);}
      check(!c.joints_attached && enabled(c,"gripper_joint") && c.fault.empty(),"loss must detach without disabling modules");
      for(std::size_t i=0;i<j.names.size();++i) if(j.names[i]=="finger_joint") j.positions[i]=0.01;
      f.advance();
      check(attach(f,c,j)>0 && c.written && enabled(c,"gripper_joint"),c.fault);
      check(c.command_q().isApprox(c.measured(),1e-9),"reattach output jumped away from measured");
      const auto at=c.command_q();
      for(int i=0;i<15;++i){f.advance();c.feedback_joints(j.names,j.positions);cycle(c);}
      check(c.command_q().isApprox(at,1e-9),"reattach replayed pre-loss target");
    });
    std::cout << count << " controller regression cases passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL case " << count << ": " << e.what() << '\n';
    return 1;
  }
}
