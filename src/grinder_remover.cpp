#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <string>

namespace gazebo
{
class GrinderRemover : public WorldPlugin
{
public:
  GrinderRemover() = default;

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    minX_ = sdf->HasElement("min_x") ? sdf->Get<double>("min_x") : 4.2;
    maxX_ = sdf->HasElement("max_x") ? sdf->Get<double>("max_x") : 5.2;
    minY_ = sdf->HasElement("min_y") ? sdf->Get<double>("min_y") : -0.6;
    maxY_ = sdf->HasElement("max_y") ? sdf->Get<double>("max_y") : 0.6;
    minZ_ = sdf->HasElement("min_z") ? sdf->Get<double>("min_z") : 0.0;
    maxZ_ = sdf->HasElement("max_z") ? sdf->Get<double>("max_z") : 1.0;

    updateConn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GrinderRemover::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo & /*info*/)
  {
    if (!world_)
      return;

    // Iterate over models and remove any box/waste in grinder region
    for (auto model : world_->Models())
    {
      if (!model || model->IsStatic())
        continue;

      const auto &name = model->GetName();
      const bool isBox = name.rfind("box_", 0) == 0 || name.rfind("waste_", 0) == 0;
      if (!isBox)
        continue;

      const auto pose = model->WorldPose();
      const auto &p = pose.Pos();
      if (p.X() < minX_ || p.X() > maxX_ ||
          p.Y() < minY_ || p.Y() > maxY_ ||
          p.Z() < minZ_ || p.Z() > maxZ_)
      {
        continue;
      }

      world_->RemoveModel(model);
    }
  }

  physics::WorldPtr world_;
  event::ConnectionPtr updateConn_;

  double minX_{};
  double maxX_{};
  double minY_{};
  double maxY_{};
  double minZ_{};
  double maxZ_{};
};

GZ_REGISTER_WORLD_PLUGIN(GrinderRemover)

}  // namespace gazebo
