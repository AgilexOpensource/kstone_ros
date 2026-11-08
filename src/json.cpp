#include "kstone_ros/controller.hpp"
#include <cmath>
#include <nlohmann/json.hpp>

namespace kstone_ros {
namespace {
nlohmann::json encode(const Node &node) {
  using K = Node::Kind;
  switch (node.kind()) {
  case K::null:
    return nullptr;
  case K::boolean:
    return node.as_bool();
  case K::number:
    return std::isfinite(node.as_number())
               ? nlohmann::json(node.as_number())
               : nlohmann::json(nullptr);
  case K::string:
    return nlohmann::json(node.as_string());
  case K::list: {
    auto a = nlohmann::json::array();
    for (const auto &v : node.items())
      a.push_back(encode(v));
    return a;
  }
  case K::map: {
    auto o = nlohmann::json::object();
    for (const auto &v : node.entries())
      o[v.first] = encode(v.second);
    return o;
  }
  }
  return nullptr;
}
Node decode(const nlohmann::json &v) {
  if (v.is_null())
    return {};
  if (v.is_boolean())
    return Node(v.get<bool>());
  if (v.is_number())
    return Node(v.get<double>());
  if (v.is_string())
    return Node(v.get<std::string>());
  if (v.is_array()) {
    auto n = Node::list();
    for (const auto &x : v)
      n.append(decode(x));
    return n;
  }
  auto n = Node::map();
  for (auto x = v.begin(); x != v.end(); ++x)
    n.set(x.key(), decode(x.value()));
  return n;
}
} // namespace
std::string json(const Node &value) {
  return encode(value).dump();
}
Node parse_json(const std::string &text) {
  return decode(nlohmann::json::parse(text));
}
void require(kstone::Status status) {
  if (!status)
    throw std::runtime_error(status.error().message);
}
Eigen::Isometry3d make_pose(const std::vector<double> &t,
                            const std::vector<double> &r) {
  if (t.size() != 3 || r.size() != 4)
    throw std::invalid_argument("pose requires xyz and wxyz");
  for (double x : t)
    if (!std::isfinite(x))
      throw std::invalid_argument("non-finite position");
  for (double x : r)
    if (!std::isfinite(x))
      throw std::invalid_argument("non-finite quaternion");
  Eigen::Quaterniond q(r[0], r[1], r[2], r[3]);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-9)
    throw std::invalid_argument("quaternion must be nonzero");
  auto p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
  p.linear() = q.normalized().toRotationMatrix();
  return p;
}
Node pose_node(const Eigen::Isometry3d &p) {
  Eigen::Quaterniond q(p.linear());
  return Node::map(
      {{"position", Node::list({p.translation().x(), p.translation().y(),
                                p.translation().z()})},
       {"orientation", Node::list({q.w(), q.x(), q.y(), q.z()})}});
}
} // namespace kstone_ros
