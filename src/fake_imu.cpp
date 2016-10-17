#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/GetModelState.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>

namespace {
using namespace std;
using namespace geometry_msgs;

static const double FREQUENCY = 0.01;
static const double BASE_X_DEFAULT = 0.4;

class FakeIMU {
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

    //! Publisher for the pose visualization
    ros::Publisher poseVizPub;

    //! Model name
    string modelName;

public:
	FakeIMU() :
		pnh("~") {
        posePub = nh.advertise<sensor_msgs::Imu>(
				"out", 1);

        ros::service::waitForService("/gazebo/get_model_state");
        modelStateServ = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true /* persistent */);
        pnh.param<string>("modelName", modelName, "human");
        poseVizPub = nh.advertise<geometry_msgs::PoseStamped>("imu/pose", 1);

        timer = nh.createTimer(ros::Duration(FREQUENCY), &FakeIMU::callback, this);
        timer.start();
	}

private:

    void visualizeOrientation(const std_msgs::Header& header, const geometry_msgs::Quaternion& orientation) {
        PoseStamped ps;
        ps.header = header;
        ps.pose.orientation = orientation;
        ps.pose.position.x = BASE_X_DEFAULT;
        poseVizPub.publish(ps);
    }

    sensor_msgs::Imu getIMUData(){
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = modelName;
        modelStateServ.call(modelState);
        sensor_msgs::Imu data;
        data.header.stamp = ros::Time::now();
        data.header.frame_id = "/odom_combined";

        data.angular_velocity = modelState.response.twist.angular;
        data.orientation = modelState.response.pose.orientation;

        return data;
    }

    void callback(const ros::TimerEvent& event){

        // Lookup the current IMU data for the human
        sensor_msgs::Imu data = getIMUData();
        visualizeOrientation(data.header, data.orientation);

        // Publish the event
        ROS_DEBUG_STREAM("Publishing an IMU event: " << data);
        posePub.publish(data);
      }
};
}
int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_imu");

	FakeIMU fi;
	ros::spin();
}
