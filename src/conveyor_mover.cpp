#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace gazebo
{
class ConveyorMover : public WorldPlugin
{
public:
  ConveyorMover() = default;
  ~ConveyorMover()
  {
    stopRequested_ = true;
    queue_.clear();
    queue_.disable();
    if (queueThread_.joinable())
      queueThread_.join();
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("ROS must be initialized before loading ConveyorMover plugin.");
      return;
    }

    world_ = world;
    minX_ = sdf->HasElement("min_x") ? sdf->Get<double>("min_x") : -1.5;
    maxX_ = sdf->HasElement("max_x") ? sdf->Get<double>("max_x") : 1.5;
    minY_ = sdf->HasElement("min_y") ? sdf->Get<double>("min_y") : -0.25;
    maxY_ = sdf->HasElement("max_y") ? sdf->Get<double>("max_y") : 0.25;
    minZ_ = sdf->HasElement("min_z") ? sdf->Get<double>("min_z") : 0.0;
    maxZ_ = sdf->HasElement("max_z") ? sdf->Get<double>("max_z") : 0.6;
    speed_ = sdf->HasElement("speed") ? sdf->Get<double>("speed") : 0.6;
    updatePeriod_ = sdf->HasElement("update_period") ? sdf->Get<double>("update_period") : 0.02;
    lastUpdate_ = common::Time::Zero;
    commandedSpeed_ = speed_;

    nh_.setCallbackQueue(&queue_);
    auto so = ros::SubscribeOptions::create<std_msgs::Float32>(
        "/conveyor/speed",
        1,
        boost::bind(&ConveyorMover::OnSpeedMsg, this, _1),
        ros::VoidPtr(),
        &queue_);
    speedSub_ = nh_.subscribe(so);

    queueThread_ = std::thread(&ConveyorMover::QueueThread, this);
    updateConn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ConveyorMover::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo &info)
  {
    if (!world_)
      return;

    if ((info.simTime - lastUpdate_).Double() < updatePeriod_)
      return;
    lastUpdate_ = info.simTime;

    for (auto model : world_->Models())
    {
      if (!model || model->IsStatic())
        continue;

      const auto &name = model->GetName();
      // skip anything clearly not the spawned boxes
      const bool isBox = name.rfind("box_", 0) == 0 || name.rfind("waste_", 0) == 0;
      if (!isBox)
        continue;

      auto pose = model->WorldPose();
      if (pose.Pos().X() < minX_ || pose.Pos().X() > maxX_ ||
          pose.Pos().Y() < minY_ || pose.Pos().Y() > maxY_ ||
          pose.Pos().Z() < minZ_ || pose.Pos().Z() > maxZ_)
      {
        continue;
      }

      double commanded = speed_;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hasSpeedCommand_)
          commanded = commandedSpeed_;
      }

      ignition::math::Vector3d vel(commanded, 0, 0);
      model->SetLinearVel(vel);
    }
  }

  void OnSpeedMsg(const std_msgs::Float32ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commandedSpeed_ = std::max(0.0, static_cast<double>(msg->data));
    hasSpeedCommand_ = true;
  }

  void QueueThread()
  {
    static const double timeout = 0.01;
    while (ros::ok() && !stopRequested_)
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  physics::WorldPtr world_;
  event::ConnectionPtr updateConn_;
  ros::NodeHandle nh_;
  ros::Subscriber speedSub_;
  ros::CallbackQueue queue_;
  std::thread queueThread_;
  std::mutex mutex_;

  double minX_{};
  double maxX_{};
  double minY_{};
  double maxY_{};
  double minZ_{};
  double maxZ_{};
  double speed_{};
  double commandedSpeed_{};
  bool hasSpeedCommand_{false};
  bool stopRequested_{false};
  double updatePeriod_{};
  common::Time lastUpdate_;
};

GZ_REGISTER_WORLD_PLUGIN(ConveyorMover)

}  // namespace gazebo
