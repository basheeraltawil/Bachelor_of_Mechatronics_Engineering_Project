// RoboCraft grasp system plugin for Gazebo Fortress.
//
// Real grippers hold parts by friction; in simulation that is fragile, so a
// successful grasp is modelled as a fixed joint between the gripper (wrist)
// link and the object - but only if the object really is between the fingers
// (distance check against the TCP), like a sensor-verified industrial grasp.
//
// Command topic (gz.msgs.StringMsg):
//   "grasp"        attach the nearest link named <graspable_prefix>* at the TCP
//   "grasp_locked" same, but weld to the object's model <lock_link> instead
//                  (platform brake engaged: the leg bearing can no longer turn)
//   "release"      remove the joint
// State topic (gz.msgs.StringMsg): "idle" | "holding:<model>/<link>[:locked]"
//                                  | "failed:<reason>"

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <chrono>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

namespace robocraft_gz
{
using namespace ignition;
using namespace ignition::gazebo;

class GraspPlugin : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  void Configure(const Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
                 EntityComponentManager & _ecm, EventManager &) override
  {
    model_ = Model(_entity);
    if (!model_.Valid(_ecm)) {
      ignerr << "[RoboCraftGrasp] must be attached to a model\n";
      return;
    }
    auto get = [&](const std::string & k, const std::string & d) {
      return _sdf->HasElement(k) ? _sdf->Get<std::string>(k) : d;
    };
    arm_ = get("arm", "arm");
    parent_link_name_ = get("parent_link", "");
    prefix_ = get("graspable_prefix", "grasp_");
    lock_link_ = get("lock_link", "plate");
    if (_sdf->HasElement("tcp_offset")) {
      tcp_offset_ = _sdf->Get<math::Vector3d>("tcp_offset");
    }
    if (_sdf->HasElement("max_distance")) {
      max_dist_ = _sdf->Get<double>("max_distance");
    }
    const auto cmd_topic = get("command_topic", "/robocraft/" + arm_ + "/grasp_cmd");
    const auto state_topic = get("state_topic", "/robocraft/" + arm_ + "/grasp_state");

    node_.Subscribe(cmd_topic, &GraspPlugin::OnCommand, this);
    state_pub_ = node_.Advertise<msgs::StringMsg>(state_topic);
    ignmsg << "[RoboCraftGrasp] " << arm_ << ": parent=" << parent_link_name_
           << " cmd=" << cmd_topic << " state=" << state_topic << "\n";
    configured_ = true;
  }

  void PreUpdate(const UpdateInfo & _info, EntityComponentManager & _ecm) override
  {
    if (!configured_) {return;}
    if (parent_link_ == kNullEntity) {
      parent_link_ = model_.LinkByName(_ecm, parent_link_name_);
      if (parent_link_ == kNullEntity) {return;}
    }

    std::optional<std::string> cmd;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      cmd.swap(pending_);
    }
    if (cmd) {
      if (*cmd == "release") {
        Release(_ecm);
      } else if (*cmd == "grasp" || *cmd == "grasp_locked") {
        Grasp(_ecm, *cmd == "grasp_locked");
      } else {
        state_ = "failed:unknown_command";
      }
      Publish();
    }
    // periodic state heartbeat (2 Hz of sim time)
    if (_info.simTime - last_pub_ > std::chrono::milliseconds(500)) {
      last_pub_ = _info.simTime;
      Publish();
    }
  }

private:
  void OnCommand(const msgs::StringMsg & _msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    pending_ = _msg.data();
  }

  void Release(EntityComponentManager & _ecm)
  {
    if (joint_ != kNullEntity) {
      _ecm.RequestRemoveEntity(joint_);
      joint_ = kNullEntity;
    }
    state_ = "idle";
  }

  void Grasp(EntityComponentManager & _ecm, bool locked)
  {
    if (joint_ != kNullEntity) {Release(_ecm);}
    const math::Pose3d tcp = worldPose(parent_link_, _ecm) * math::Pose3d(tcp_offset_, math::Quaterniond::Identity);

    Entity best = kNullEntity;
    double best_d = std::numeric_limits<double>::max();
    _ecm.Each<components::Link, components::Name, components::ParentEntity>(
      [&](const Entity & e, const components::Link *, const components::Name * name,
          const components::ParentEntity * parent) -> bool
      {
        if (parent->Data() == model_.Entity()) {return true;}           // never grasp ourselves
        if (name->Data().rfind(prefix_, 0) != 0) {return true;}
        const auto p = worldPose(e, _ecm).Pos();
        const double dxy = std::hypot(p.X() - tcp.Pos().X(), p.Y() - tcp.Pos().Y());
        const double dz = std::abs(p.Z() - tcp.Pos().Z());
        if (dxy <= max_dist_ && dz <= 0.05 && dxy < best_d) {
          best_d = dxy;
          best = e;
        }
        return true;
      });

    if (best == kNullEntity) {
      state_ = "failed:no_object_between_fingers";
      ignwarn << "[RoboCraftGrasp] " << arm_ << ": nothing to grasp at TCP " << tcp.Pos() << "\n";
      return;
    }

    const Entity obj_model = _ecm.Component<components::ParentEntity>(best)->Data();
    Entity child = best;
    if (locked) {
      Model m(obj_model);
      const Entity lock = m.LinkByName(_ecm, lock_link_);
      if (lock != kNullEntity) {child = lock;}
    }
    joint_ = _ecm.CreateEntity();
    _ecm.CreateComponent(joint_, components::DetachableJoint({parent_link_, child, "fixed"}));

    std::ostringstream ss;
    auto mname = _ecm.Component<components::Name>(obj_model);
    ss << "holding:" << (mname ? mname->Data() : "?") << "/"
       << _ecm.Component<components::Name>(best)->Data() << (locked ? ":locked" : "");
    state_ = ss.str();
    ignmsg << "[RoboCraftGrasp] " << arm_ << " " << state_ << " (offset " << best_d * 1000 << " mm)\n";
  }

  void Publish()
  {
    msgs::StringMsg m;
    m.set_data(state_);
    state_pub_.Publish(m);
  }

  Model model_{kNullEntity};
  bool configured_{false};
  std::string arm_, parent_link_name_, prefix_, lock_link_;
  math::Vector3d tcp_offset_{0, 0, 0};
  double max_dist_{0.02};
  Entity parent_link_{kNullEntity};
  Entity joint_{kNullEntity};
  std::string state_{"idle"};
  std::chrono::steady_clock::duration last_pub_{0};

  transport::Node node_;
  transport::Node::Publisher state_pub_;
  std::mutex mutex_;
  std::optional<std::string> pending_;
};
}  // namespace robocraft_gz

IGNITION_ADD_PLUGIN(robocraft_gz::GraspPlugin, ignition::gazebo::System,
                    robocraft_gz::GraspPlugin::ISystemConfigure,
                    robocraft_gz::GraspPlugin::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(robocraft_gz::GraspPlugin, "robocraft_gz::GraspPlugin")
