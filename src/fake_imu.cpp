#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>

namespace
{
using namespace std;
using namespace geometry_msgs;

static const double FREQUENCY = 0.033; // 30HZ

class FakeIMU
{
private:
    //! Publisher for the pose
    ros::Publisher posePub;

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Frequency at which to publish the humanoid state
    ros::Timer timer;

    //! Cached service client.
    ros::ServiceClient modelStateServ;

    //! Cached service client.
    ros::ServiceClient worldStateServ;

    //! Publisher for the pose visualization
    ros::Publisher poseVizPub;

    //! Model name
    string modelName;

    //! Whether the model is initialized
    bool isModelInitialized;

public:
    FakeIMU() :
        pnh("~")
    {
        posePub = nh.advertise<sensor_msgs::Imu>("out", 1);

        ros::service::waitForService("/gazebo/get_world_properties");
        worldStateServ = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties", true /* persistent */);

        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);

        pnh.param<string>("modelName", modelName, "human");

        isModelInitialized = false;

        timer = nh.createTimer(ros::Duration(FREQUENCY), &FakeIMU::callback, this);
        timer.start();

        ROS_INFO("Fake human IMU initialized successfully");
    }

private:

    sensor_msgs::Imu getIMUData(bool& success)
    {
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = modelName;
        if (!modelStateServ.call(modelState)){
            ROS_WARN("Failed to get model state in IMU");
            success = false;
        }
        else {
            success = true;
        }

        sensor_msgs::Imu data;
        data.header.stamp = ros::Time::now();
        data.header.frame_id = "/odom_combined";

        data.angular_velocity = modelState.response.twist.angular;
        data.orientation = modelState.response.pose.orientation;

        ROS_DEBUG("Position: [%f %f %f], Orientation: [%f %f %f %f], Linear Velocity: [%f %f %f], Angular Velocity: [%f %f %f]",
                modelState.response.pose.position.x, modelState.response.pose.position.y, modelState.response.pose.position.z,
                modelState.response.pose.orientation.x, modelState.response.pose.orientation.y, modelState.response.pose.orientation.z, modelState.response.pose.orientation.w,
                modelState.response.twist.linear.x, modelState.response.twist.linear.y, modelState.response.twist.linear.z,
                modelState.response.twist.angular.x, modelState.response.twist.angular.y, modelState.response.twist.angular.z);

        return data;
    }

    void callback(const ros::TimerEvent& event)
    {
        if (!isModelInitialized) {
            gazebo_msgs::GetWorldProperties worldProperties;
            if (!worldStateServ.call(worldProperties)) {
                ROS_ERROR("Failed to get world properties");
                return;
            }

            for (unsigned int i = 0; i < worldProperties.response.model_names.size(); ++i) {
                if (worldProperties.response.model_names[i] == modelName) {
                    ROS_INFO("Human model is available");
                    isModelInitialized = true;
                }
            }
        }

        if (!isModelInitialized) {
            ROS_DEBUG("Human model not initialized");
            return;
        }

        // Lookup the current IMU data for the human
        bool success;
        sensor_msgs::Imu data = getIMUData(success);
        if (success) {
            // Publish the event
            ROS_DEBUG_STREAM("Publishing an IMU event: " << data);
            posePub.publish(data);
        }
    }
};
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_imu");

    FakeIMU fi;
    ros::spin();
}
