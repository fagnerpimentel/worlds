#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <iostream>

namespace gazebo
{
  class AutoSlidingDoor : public WorldPlugin
  {
    std::string doorName = "";
    bool regionIsOccupied = false;

    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::ModelPtr doorModel;

    public: AutoSlidingDoor() : WorldPlugin()
    {
    }

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      this->world = _parent;
      this->doorName = _sdf->Get<std::string>("door_name");
      this->doorModel = _parent->ModelByName(this->doorName);

      std::cout << "Starting AutoSlidingDoor: " << this->doorName << std::endl;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AutoSlidingDoor::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      // Get all the models.
      physics::Model_V models = this->world->Models();

      // get scene
      // rendering::ScenePtr scene = rendering::RenderEngine::Instance()->GetScene();


      // Process each model.
      for (physics::Model_V::iterator iter = models.begin();
           iter != models.end(); ++iter)
      {
        // Skip models that are static
        // if ((*iter)->IsStatic())
        //   continue;

        // Skip door
        if ((*iter)->GetName() == this->doorName)
          continue;

        // rendering::VisualPtr visual = scene->GetVisual(this->doorName.c_str());
        // ignition::math::Box door_box = visual->BoundingBox();


        // printf("%s -> (%f,%f,%f)\n",
        //   (*iter)->GetName().c_str(),
        //   (*iter)->WorldPose().Pos().X(),
        //   (*iter)->WorldPose().Pos().Y(),
        //   (*iter)->WorldPose().Pos().Z());

        printf("a %s %s\n", this->doorName.c_str(), (*iter)->GetName().c_str());


        // ignition::math::Vector3<double> pos = this->doorModel->WorldPose().Pos();

        ignition::math::Box door_box = this->doorModel->BoundingBox();
        ignition::math::Box model_box = (*iter)->BoundingBox();
        // printf("a %s %f\n", (*iter)->GetName().c_str(), door_box.Max().X());

        // if(door_box.Intersects(model_box))
        // {
        //   printf("a %s %d %d\n", (*iter)->GetName().c_str(), );
        // }

        printf("b %f %f %f\n", door_box.Min().X(), door_box.Min().Y(), door_box.Min().Z());
        printf("b %f %f %f\n", door_box.Max().X(), door_box.Max().Y(), door_box.Max().Z());
        printf("b %f %f %f\n", (*iter)->WorldPose().Pos().X(), (*iter)->WorldPose().Pos().Y(), (*iter)->WorldPose().Pos().Z());
        std::cout << "Intersects " <<  door_box.Intersects(model_box) << std::endl;
        std::cout << std::endl;

        // If inside, then open the door.
        // if (((*iter)->WorldPose().Pos().X() > pos.X()-1.0 & (*iter)->WorldPose().Pos().X() < pos.X()+1.0) &
        //     ((*iter)->WorldPose().Pos().Y() > pos.Y()-0.5 & (*iter)->WorldPose().Pos().Y() < pos.Y()+0.5) &
        //     ((*iter)->WorldPose().Pos().Z() > pos.Z()-0.0 & (*iter)->WorldPose().Pos().Z() < pos.Z()+2.0))
        if ((((*iter)->WorldPose().Pos().X() > door_box.Min().X()) & ((*iter)->WorldPose().Pos().X() < door_box.Max().X())) &
            (((*iter)->WorldPose().Pos().Y() > door_box.Min().Y()) & ((*iter)->WorldPose().Pos().Y() < door_box.Max().Y())) &
            (((*iter)->WorldPose().Pos().Z() > door_box.Min().Z()) & ((*iter)->WorldPose().Pos().Z() < door_box.Max().Z())))
        {
          this->regionIsOccupied = true;
          break;
        }
        else
        {
          this->regionIsOccupied = false;
        }
      }

      std::string jointDoorName = this->doorName + "::door_joint";
      physics::JointPtr jointDoor = this->doorModel->GetJoint(jointDoorName);
      if(this->regionIsOccupied)
      {
        jointDoor->SetForce(0, 1);
      }
      else
      {
        jointDoor->SetForce(0, -1);
      }

    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(AutoSlidingDoor)
}
