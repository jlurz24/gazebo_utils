#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

using namespace std;

#define PRINT_DEBUG 1

namespace gazebo {

class GroundJointPlugin : public ModelPlugin {


private:
    physics::ModelPtr model;
    ros::ServiceServer createJointService;
    ros::NodeHandle nh;
public:
    GroundJointPlugin() : ModelPlugin() {
#if(PRINT_DEBUG)
        std::cout << "Constructing the ground position plugin" << std::endl;
#endif
    }

private: void connectVirtualJoint() {
   // Setup the virtual joint if needed
   physics::LinkPtr parent = model->GetLink("ground_plane::link");
#if(PRINT_DEBUG)
   cout << "Connecting virtual joint" << endl;
#endif
    physics::JointPtr joint = model->GetWorld()->GetPhysicsEngine()->CreateJoint(/*"universal" */ "ball", model);
    physics::LinkPtr child = model->GetLink("human::link");
    joint->Attach(parent, child);
    // Set the height of the joint to the radius of the pole to allow the pole to fall all the
    // way to the ground
    joint->Load(parent, child, math::Pose(math::Vector3(0.0, 0.0, -0.84675), math::Quaternion()));
    joint->SetName("virtual_ground_human_connection");

    joint->Init();
#if(PRINT_DEBUG)
   cout << "Virtual joint connected" << endl;
#endif
}

private: bool callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
#if(PRINT_DEBUG)
        std::cout << "Ground joint plugin received callback" << std::endl;
#endif
    connectVirtualJoint();
    return true;
}

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        model = _model;

#if(PRINT_DEBUG)
        std::cout << "Loading the ground joint plugin" << std::endl;
#endif
        createJointService = nh.advertiseService("/create_ground_joint", &GroundJointPlugin::callback, this);
    }
};
GZ_REGISTER_MODEL_PLUGIN(GroundJointPlugin)
}

