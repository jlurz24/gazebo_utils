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

            const string listControllersServiceName = "/pr2_controller_manager/list_controllers";
            ROS_INFO("Waiting for the list controllers service");
            if (!ros::service::waitForService(listControllersServiceName, ros::Duration(60))) {
                ROS_WARN("List controllers service could not be found");
            }
            ROS_INFO("List controllers service is up");

            // wait for pr2 controllers
            ros::ServiceClient listControllersClient = nh.serviceClient<pr2_mechanism_msgs::ListControllers>(listControllersServiceName);
            pr2_mechanism_msgs::ListControllers listControllers;
            unsigned int controllersCount = 0;

            ros::Time start = ros::Time::now();
            ros::Duration timeout = ros::Duration(60);

            while (ros::ok() && controllersCount != 2 && ros::Time::now() - start < timeout)
            {
                controllersCount = 0;
                if (!listControllersClient.call(listControllers)) {
                    ROS_ERROR("List controllers call failed");
                    continue;
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

            if (controllersCount!= 2) {
                ROS_WARN("Failed to find both running arm controllers");
            }

            if (!ros::ok()) {
                ROS_WARN("Node shutdown detected");
                return;
            }

            auto_ptr<TrajClient> lTrajectoryClient(new TrajClient("l_arm_controller/joint_trajectory_action", true));
            auto_ptr<TrajClient> rTrajectoryClient(new TrajClient("r_arm_controller/joint_trajectory_action", true));

            ROS_INFO("Waiting for the left joint_trajectory_action server");
            if(!lTrajectoryClient->waitForServer(ros::Duration(30))){
                ROS_WARN("left joint trajectory server could not be found");
            }
            ROS_INFO("left joint_trajectory_action server is up");

            ROS_INFO("Waiting for the right joint_trajectory_action server");
            if(!rTrajectoryClient->waitForServer(ros::Duration(30))){
                ROS_WARN("right joint trajectory server could not be found");
            }
            ROS_INFO("right joint_trajectory_action server is up");

            if (!ros::ok()) {
                ROS_WARN("Node shutdown detected");
                return;
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
            lGoal.trajectory.points[0].time_from_start = ros::Duration(1.0);

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
            rGoal.trajectory.points[0].time_from_start = ros::Duration(1.0);

            rTrajectoryClient->sendGoal(rGoal);

            if (!ros::ok()) {
                ROS_WARN("Node shutdown detected");
                return;
            }

            lTrajectoryClient->waitForResult(ros::Duration(5.0));
            if (lTrajectoryClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Left trajectory client failed");
            }

            rTrajectoryClient->waitForResult(ros::Duration(5.0));
            if (rTrajectoryClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Right trajectory client failed");
            }

            ROS_INFO("Setting initial position complete");
        }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "set_arm_positions");
  SetArmPositions sap;
  sap.moveToBasePositions();
}
