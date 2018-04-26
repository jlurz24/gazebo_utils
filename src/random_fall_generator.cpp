#include <ros/ros.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/random.hpp>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

#define USE_FIXED_SEED 1

namespace
{

using namespace std;
using namespace ros;

static const unsigned int FIXED_SEED = 0;
static const double pi = boost::math::constants::pi<double>();
static const double MASS = 0.896;
static const double DURATION = 0.02;

class RandomFallGenerator
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Gazebo client
    ros::ServiceClient gazeboClient;

    //! Wrench client
    ros::ServiceClient wrenchClient;

    //! Create ground joint client
    ros::ServiceClient createGroundJointClient;

    //! Notifier
    ros::Publisher notifier;

    //! X position
    double x;

    //! Y position
    double y;

    //! X velocity
    double xDot;

    //! Y velocity
    double yDot;

    //! rng
    boost::mt19937 rng;
public:
    RandomFallGenerator() :
        pnh("~")
    {
        ros::service::waitForService("/gazebo/set_link_state");
        ros::service::waitForService("/create_ground_joint");
        ros::service::waitForService("/gazebo/apply_body_wrench");

        gazeboClient = nh.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state", true /* persistent */);
        createGroundJointClient = nh.serviceClient<std_srvs::Empty>("/create_ground_joint", true /* persistent */);
        wrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench", true /* persistent */);


        notifier = nh.advertise<std_msgs::Header>("/human/fall", 1, true);

        unsigned int seed = 0;
#if(!USE_FIXED_SEED)
        seed = static_cast<unsigned int>(std::time(NULL));
#else
        const char* scenarioNumberStr = std::getenv("i");
        if(scenarioNumberStr != NULL)
        {
            unsigned int scenarioNumber = boost::lexical_cast<unsigned int>(scenarioNumberStr);
            seed = scenarioNumber + 1000;
        }
        else
        {
            ROS_INFO("No scenario number set. Using %u", FIXED_SEED);
            seed = FIXED_SEED;
        }
#endif
        rng.seed(seed);

        {
            // Generate x distance. 0.4 ensures the pole is in front of the base and the elbows
            boost::uniform_real<double> xRange(0.4, 0.6);
            boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > getX(rng, xRange);
            x = getX();

            // Distance between the end effectors minus 15cm padding for the size of the links
            boost::uniform_real<double> yRange(-0.4, 0.4);
            boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > getY(rng, yRange);
            y = getY();
            ROS_INFO("Generated x, y [%f, %f]", x, y);
        }

        {
            // Generate total velocity
            // maximum velocity of PR2 base is 1.0m/s
            boost::uniform_real<double> vRange(0, 1.0);
            boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genV(rng, vRange);
            double v = genV();
            ROS_INFO("Generated velocity [%f]", v);

            // Generate random direction
            boost::uniform_real<double> dRange(0.0, pi * 2.0);
            boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genD(rng, dRange);
            double d = genD();
            ROS_INFO("Generated direction [%f]", d);

            tf::Vector3 directionVector;

            // Set the vector length to the velocity
            directionVector.setX(v);

            // Rotate about the z axis
            directionVector = directionVector.rotate(tf::Vector3(0, 0, 1), tfScalar(d));

            xDot = directionVector.x();
            yDot = directionVector.y();

            // assert(fabs(directionVector.z()) < 0.001);
            ROS_INFO("Generated x_dot, y_dot [%f, %f]", xDot, yDot);
        }

        nh.setParam("base_x", x);
        nh.setParam("base_y", y);
        nh.setParam("base_z", 0.0);
    }

    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Scenario Number, x, y, x_dot, y_dot" << endl;
    }

    void printState(const gazebo_msgs::LinkState& linkState)
    {
        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            ROS_DEBUG_STREAM("Results folder not set. Using current directory.");
            resultsFolder = "";
        }

        const char* scenarioNumberStr = std::getenv("i");
        if(scenarioNumberStr == NULL)
        {
            ROS_DEBUG_STREAM("Scenario number not set. Using 0.");
            scenarioNumberStr = "0";
        }

        const string resultsFileName = boost::filesystem::current_path().string() + "/" + string(resultsFolder) + "/" + "velocities.csv";
        ROS_DEBUG_STREAM("Using results file: " << resultsFileName);
        bool exists = boost::filesystem::exists(resultsFileName);
        ofstream outputCSV;
        outputCSV.open(resultsFileName.c_str(), ios::out | ios::app);
        assert(outputCSV.is_open());

        if(!exists)
        {
            writeHeader(outputCSV);
        }

        outputCSV << scenarioNumberStr << ", " << linkState.pose.position.x << ", " << linkState.pose.position.y << ", " << linkState.twist.linear.x << ", " << linkState.twist.linear.y << endl;
        outputCSV.close();
        ROS_DEBUG_STREAM("Printing output file complete");
    }

    void apply()
    {
    	ROS_INFO("Setting position [%f, %f]", x, y);
        gazebo_msgs::SetLinkState state;
        state.request.link_state.link_name = "human::link";
        state.request.link_state.reference_frame = "world";
        state.request.link_state.pose.position.x = x;
        state.request.link_state.pose.position.y = y;
        state.request.link_state.pose.position.z = 0.8785;

        printState(state.request.link_state);

        ROS_INFO("Setting the base position in Gazebo");
        if (!gazeboClient.call(state))
        {
           ROS_ERROR("Failed to apply state");
           return;
        }

        std_srvs::Empty message = std_srvs::Empty();
        if (!createGroundJointClient.call(message)) {
            ROS_ERROR("Failed to create ground joint");
            return;
        }

        ROS_INFO("Setting velocity [%f, %f]", xDot, yDot);

        // Convert to acceleration
        const double forceX = 1.35 * MASS * xDot / DURATION ;
        const double forceY = 1.35 * MASS * yDot / DURATION;

        ROS_INFO("Applying forces [%f, %f]", forceX, forceY);

        gazebo_msgs::ApplyBodyWrench wrench;
        wrench.request.body_name = "human::link";
        wrench.request.wrench.force.x = forceX;
        wrench.request.wrench.force.y = forceY;
        wrench.request.wrench.torque.z = 0;
        wrench.request.duration = ros::Duration(DURATION);

        ROS_INFO("Starting the fall in Gazebo");
        if (!wrenchClient.call(wrench))
        {
           ROS_ERROR("Failed to apply wrench");
           return;
        }

        ROS_INFO("Notifying");
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        notifier.publish(header);
        ROS_INFO("Setting position and velocity complete");
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_fall_generator");
    RandomFallGenerator rfg;

    // Wait for the fall catcher to be ready
    ROS_INFO("Waiting for the catch_human_controller");
    ros::service::waitForService("/humanoid_catching/catch_human_controller");
    ROS_INFO("Catch human controller ready");

    // Allow all the sockets to connect
    ros::Duration(0.5).sleep();

    // Set the position/velocity and send the start fall message
    rfg.apply();

    // Wait until the trial is complete
    ros::Duration(10.0).sleep();
}
