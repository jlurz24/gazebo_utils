#include <ros/ros.h>
#include <boost/math/constants/constants.hpp>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace ros;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
static const double pi = boost::math::constants::pi<double>();

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

            TrajClient* lTrajectoryClient = new TrajClient("l_arm_controller/joint_trajectory_action", true);
            TrajClient* rTrajectoryClient = new TrajClient("r_arm_controller/joint_trajectory_action", true);

            while(!lTrajectoryClient->waitForServer()){
                ROS_INFO("Waiting for the left joint_trajectory_action server");
            }
            while(!rTrajectoryClient->waitForServer()){
                ROS_INFO("Waiting for the right joint_trajectory_action server");
            }

            pr2_controllers_msgs::JointTrajectoryGoal lGoal;
            lGoal.trajectory.header.stamp = ros::Time::now();
            lGoal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
            lGoal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
            lGoal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
            lGoal.trajectory.joint_names.push_back("l_elbow_flex_joint");
            lGoal.trajectory.joint_names.push_back("l_forearm_roll_joint");
            lGoal.trajectory.joint_names.push_back("l_wrist_flex_joint");
            lGoal.trajectory.joint_names.push_back("l_wrist_roll_joint");
            lGoal.trajectory.points.resize(1);
            lGoal.trajectory.points[0].positions.resize(7);
            lGoal.trajectory.points[0].positions[0] = pi / 4.0;
            lGoal.trajectory.points[0].positions[1] = 0.0;
            lGoal.trajectory.points[0].positions[2] = pi / 2.0;
            lGoal.trajectory.points[0].positions[3] = -pi / 4.0;
            lGoal.trajectory.points[0].positions[4] = 0.0;
            lGoal.trajectory.points[0].positions[5] = 0.0;
            lGoal.trajectory.points[0].positions[6] = 0.0;

            lGoal.trajectory.points[0].velocities.resize(7);
            lGoal.trajectory.points[0].velocities[0] = 0.0;

            lTrajectoryClient->sendGoal(lGoal);

            pr2_controllers_msgs::JointTrajectoryGoal rGoal;
            rGoal.trajectory.header.stamp = ros::Time::now();
            rGoal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
            rGoal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
            rGoal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
            rGoal.trajectory.joint_names.push_back("r_elbow_flex_joint");
            rGoal.trajectory.joint_names.push_back("r_forearm_roll_joint");
            rGoal.trajectory.joint_names.push_back("r_wrist_flex_joint");
            rGoal.trajectory.joint_names.push_back("r_wrist_roll_joint");

            rGoal.trajectory.points.resize(1);
            rGoal.trajectory.points[0].positions.resize(7);
            rGoal.trajectory.points[0].positions[0] = -pi / 4.0;
            rGoal.trajectory.points[0].positions[1] = 0.0;
            rGoal.trajectory.points[0].positions[2] = -pi / 2.0;
            rGoal.trajectory.points[0].positions[3] = -pi / 4.0;
            rGoal.trajectory.points[0].positions[4] = 0.0;
            rGoal.trajectory.points[0].positions[5] = 0.0;
            rGoal.trajectory.points[0].positions[6] = 0.0;

            rGoal.trajectory.points[0].velocities.resize(7);
            rGoal.trajectory.points[0].velocities[0] = 0.0;

            rTrajectoryClient->sendGoal(rGoal);

            lTrajectoryClient->waitForResult(ros::Duration(5.0));
            if (lTrajectoryClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Left trajectory client failed");
            }

            rTrajectoryClient->waitForResult(ros::Duration(5.0));
            if (rTrajectoryClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Right trajectory client failed");
            }

            // Resend goals to force the arms to stop at the position
            rTrajectoryClient->sendGoal(rGoal);
            lTrajectoryClient->sendGoal(lGoal);

            lTrajectoryClient->waitForResult(ros::Duration(5.0));
            if (lTrajectoryClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Left trajectory client failed");
            }

            rTrajectoryClient->waitForResult(ros::Duration(5.0));
            if (rTrajectoryClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Right trajectory client failed");
            }

            delete lTrajectoryClient;
            delete rTrajectoryClient;
            ROS_INFO("Setting initial position complete");
        }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "set_arm_positions");
  SetArmPositions sap;
  sap.moveToBasePositions();
}
