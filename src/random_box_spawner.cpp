#include <gazebo/common/Events.hh>

#include <gazebo/common/Time.hh>

#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/physics/physics.hh>

#include <gazebo/transport/transport.hh>

#include <functional>

#include <random>

#include <sstream>

#include <string>

#include <vector>

namespace gazebo

{

class RandomBoxSpawner : public WorldPlugin

{

public:

  RandomBoxSpawner() = default;

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override

  {

    world_ = world;

    // Parameters

    spawnInterval_ = sdf->HasElement("spawn_interval") ? sdf->Get<double>("spawn_interval") : 3.0;

    boxSize_ = sdf->HasElement("box_size") ? sdf->Get<double>("box_size") : 0.06;

    dropHeight_ = sdf->HasElement("drop_height") ? sdf->Get<double>("drop_height") : 1.0;

    spawnX_ = sdf->HasElement("spawn_x") ? sdf->Get<double>("spawn_x") : -4.5;

    spawnYSpread_ = sdf->HasElement("spawn_y_spread") ? sdf->Get<double>("spawn_y_spread") : 0.12;

    rng_.seed(std::random_device{}());

    // Gazebo transport

    node_ = transport::NodePtr(new transport::Node());

    node_->Init(world_->Name());

    factoryPub_ = node_->Advertise<msgs::Factory>("~/factory");

    lastSpawn_ = world_->SimTime();

    

    // Spawn first box immediately

    PublishBox();

    

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(

        std::bind(&RandomBoxSpawner::OnUpdate, this, std::placeholders::_1));

    

    gzmsg << "â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•" << std::endl;

    gzmsg << "RandomBoxSpawner: Plugin loaded!" << std::endl;

    gzmsg << "  Spawn interval: " << spawnInterval_ << "s" << std::endl;

    gzmsg << "  Box size: " << boxSize_ << "m" << std::endl;

    gzmsg << "  Drop height: " << dropHeight_ << "m" << std::endl;

    gzmsg << "  NO INITIAL SPEED - Belt will move boxes" << std::endl;

    gzmsg << "â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•" << std::endl;

  }

private:

  void OnUpdate(const common::UpdateInfo &info)

  {

    if ((info.simTime - lastSpawn_).Double() < spawnInterval_)

      return;

    lastSpawn_ = info.simTime;

    PublishBox();

  }

  void PublishBox()

  {

    if (!factoryPub_)

      return;

    // Random Y offset

    std::uniform_real_distribution<double> yDist(-spawnYSpread_, spawnYSpread_);

    double y = yDist(rng_);

    // 6 color categories

    const std::vector<std::pair<std::string, std::string>> colors = {

        {"metal", "1 1 1 1"},            // Metal (White - algılama için yüksek kontrast)

        {"plastic", "0.2 0.4 0.9 1"},    // Plastic (Blue)

        {"glass", "0.2 0.8 0.3 1"},      // Glass (Green)

        {"paper", "0.9 0.8 0.2 1"},      // Paper (Yellow)

        {"battery", "0.6 0.2 0.8 1"},    // Battery (Purple)

        {"organic", "0.9 0.3 0.2 1"}     // Organic (Red)

    };

    std::uniform_int_distribution<int> colorDist(0, static_cast<int>(colors.size()) - 1);

    auto color = colors[colorDist(rng_)];

    // Build SDF

    std::ostringstream sdfStr;

    sdfStr << "<sdf version='1.6'>"

           << "<model name='waste_" << counter_++ << "'>"

           << "<pose>" << spawnX_ << " " << y << " " << dropHeight_ << " 0 0 0</pose>"

           << "<static>false</static>"

           << "<link name='box_link'>"

           << "<pose>0 0 0 0 0 0</pose>"

           << "<inertial>"

           << "<mass>0.10</mass>"

           << "<inertia>"

           << "<ixx>2e-4</ixx><iyy>2e-4</iyy><izz>2e-4</izz>"

           << "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"

           << "</inertia>"

           << "</inertial>"

           << "<collision name='box_collision'>"

           << "<geometry><box><size>" << boxSize_ << " " << boxSize_ << " " << boxSize_ << "</size></box></geometry>"

           << "<surface>"

           << "<friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>"

           << "<bounce><restitution_coefficient>0.1</restitution_coefficient></bounce>"

           << "</surface>"

           << "</collision>"

           << "<visual name='box_visual'>"

           << "<geometry><box><size>" << boxSize_ << " " << boxSize_ << " " << boxSize_ << "</size></box></geometry>"

           << "<material>"

           << "<ambient>" << color.second << "</ambient>"

           << "<diffuse>" << color.second << "</diffuse>"

           << "<specular>0.1 0.1 0.1 1</specular>"

           << "</material>"

           << "</visual>"

           << "</link>"

           << "</model>"

           << "</sdf>";

    msgs::Factory msg;

    msg.set_sdf(sdfStr.str());

    factoryPub_->Publish(msg);

    

    gzmsg << "ðŸ“¦ Spawned waste_" << counter_ << " (" << color.first << ") at x=" 

          << spawnX_ << ", y=" << y << std::endl;

  }

  physics::WorldPtr world_;

  transport::NodePtr node_;

  transport::PublisherPtr factoryPub_;

  event::ConnectionPtr updateConnection_;

  common::Time lastSpawn_;

  double spawnInterval_{};

  double boxSize_{};

  double dropHeight_{};

  double spawnX_{};

  double spawnYSpread_{};

  std::default_random_engine rng_;

  uint64_t counter_ = 0;

};

GZ_REGISTER_WORLD_PLUGIN(RandomBoxSpawner)

}  // namespace gazebo iÃ§indeki kodlar bunlar olsun
