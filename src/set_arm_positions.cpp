#include <ros/ros.h>
#include <boost/math/constants/constants.hpp>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <std_srvs/Empty.h>

using namespace std;
using namespace ros;

class SetArmPositions {
    private:
        //! Node handle
        ros::NodeHandle nh;

        //! Private nh
        ros::NodeHandle pnh;

    public:
        SetArmPositions() : pnh("~") {
        }

        void moveToBasePositions(){

            ROS_INFO("Moving arms to base position");

            string listControllersServiceName = "/pr2_controller_manager/list_controllers";
            ros::service::waitForService(listControllersServiceName);

            // wait for pr2 controllers
            ros::ServiceClient listControllersClient = nh.serviceClient<pr2_mechanism_msgs::ListControllers>(listControllersServiceName);
            pr2_mechanism_msgs::ListControllers listControllers;
            unsigned int controllersCount = 0;
            while (controllersCount != 2)
            {
                controllersCount = 0;
                if (!listControllersClient.call(listControllers)) {
                    ROS_ERROR("List controllers call failed");
                }
                for (unsigned int i = 0; i < listControllers.response.controllers.size() ; ++i) {
                    string name = listControllers.response.controllers[i];
                    string state = listControllers.response.state[i];
                    if ((name == "r_arm_controller" || name == "l_arm_controller") && state == "running") {
                        controllersCount++;
                        ROS_INFO("controller [%s] state [%s] count[%d]", name.c_str(), state.c_str(), controllersCount);
                    }
                }
            }

            ROS_INFO("Stopping controllers");

            // Stop the arm controllers
            const string switchControllerServiceName = "/pr2_controller_manager/switch_controller";
            ROS_INFO("Waiting for arm controller service");
            ros::service::waitForService(switchControllerServiceName);
            ROS_INFO("Arm controller service ready");
            ros::ServiceClient switchControllerClient = nh.serviceClient<pr2_mechanism_msgs::SwitchController>(switchControllerServiceName, true);

            ROS_INFO("Stopping controllers");
            pr2_mechanism_msgs::SwitchController switchControllersOff;
            switchControllersOff.request.start_controllers.clear();
            switchControllersOff.request.stop_controllers.clear();
            switchControllersOff.request.stop_controllers.push_back("r_arm_controller");
            switchControllersOff.request.stop_controllers.push_back("l_arm_controller");
            switchControllersOff.request.strictness = pr2_mechanism_msgs::SwitchControllerRequest::STRICT;
            if (!switchControllerClient.call(switchControllersOff)) {
                ROS_ERROR("Failed to switch controllers off");
                return;
            }

            ROS_INFO("Controllers stopped");

            string pauseServiceName = "/gazebo/pause_physics";
            ROS_INFO("Waiting for pause simulation service");
            ros::service::waitForService(pauseServiceName);
            ROS_INFO("Pause simulation service ready");

            ROS_INFO("Pausing physics");

            // Pause simulation
            ros::ServiceClient pauseSimulationClient = nh.serviceClient<std_srvs::Empty>(pauseServiceName, true);
            std_srvs::Empty ps;
            if (!pauseSimulationClient.call(ps)) {
                ROS_ERROR("Failed to pause simulation");
                return;
            }

            ROS_INFO("Simulation paused");

            string setModelConfigurationName = "/gazebo/set_model_configuration";
            ROS_INFO("Waiting for set model configuration service");
            ros::service::waitForService(setModelConfigurationName);
            ROS_INFO("Set model configuration service ready");
            ros::ServiceClient gazeboClient = nh.serviceClient<gazebo_msgs::SetModelConfiguration>(setModelConfigurationName, true /* persistent */);

            ROS_INFO("Setting the model configuration");
            gazebo_msgs::SetModelConfiguration config;
            config.request.model_name = "pr2";
            config.request.urdf_param_name = "robot_description";
            config.request.joint_names.resize(4);
            config.request.joint_names[0] = "l_shoulder_pan_link";
            config.request.joint_names[1] = "r_shoulder_pan_link";
            config.request.joint_names[2] = "l_upper_arm_roll_joint";
            config.request.joint_names[3] = "r_upper_arm_roll_joint";
            config.request.joint_positions.resize(4);
            config.request.joint_positions[0] = 2.1350018853307198;
            config.request.joint_positions[1] = -0.3689082990354322;
            config.request.joint_positions[2] = 1.6399982206352233;
            config.request.joint_positions[3] = -1.6399982206352233;

            if (!gazeboClient.call(config))
            {
                ROS_ERROR("Failed to set model configuration");
                return;
            }
            if (!config.response.success) {
                ROS_ERROR("Setting the configuration was unsuccessful: %s", config.response.status_message.c_str());
                return;
            }
            ROS_INFO("Model configuration set");

            string unpauseServiceName = "/gazebo/unpause_physics";
            ROS_INFO("Waiting for unpause simulation service");
            ros::service::waitForService(unpauseServiceName);
            ROS_INFO("Unpause simulation service ready");

            ROS_INFO("Unpausing physics");

            // Unpause simulation
            ros::ServiceClient unpauseSimulationClient = nh.serviceClient<std_srvs::Empty>(unpauseServiceName, true);
            std_srvs::Empty ups;
            if (!unpauseSimulationClient.call(ups)) {
                ROS_ERROR("Failed to unpause simulation");
                return;
            }

            ROS_INFO("Simulation unpaused");

            // Restart the controllers
            ROS_INFO("Starting controllers");
            pr2_mechanism_msgs::SwitchController switchControllersOn;
            switchControllersOn.request.start_controllers.clear();
            switchControllersOn.request.stop_controllers.clear();
            switchControllersOn.request.start_controllers.push_back("r_arm_controller");
            switchControllersOn.request.start_controllers.push_back("l_arm_controller");
            switchControllersOn.request.strictness = pr2_mechanism_msgs::SwitchControllerRequest::STRICT;
            if (!switchControllerClient.call(switchControllersOn)) {
                ROS_ERROR("Failed to switch controllers on");
                return;
            }
            ROS_INFO("Controllers started");
        }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "set_arm_positions");
  SetArmPositions sap;
  sap.moveToBasePositions();
  ros::Duration(3.0).sleep();
}
