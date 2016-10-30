#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/random.hpp>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#define USE_FIXED_SEED 1

namespace
{

using namespace std;
using namespace ros;

static const double DURATION_DEFAULT = 0.001;
static const unsigned int FIXED_SEED = 0;

class RandomTorqueApplier
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Gazebo client
    ros::ServiceClient gazeboClient;

    //! Notifier
    ros::Publisher notifier;

    //! X torque
    double x;

    //! Y torque
    double y;

    //! Z torque
    double z;

    //! Torque Duration
    double duration;

    //! Set random values for x-y
    bool random;

    //! Wait for a specific topic
    bool waitForTopic;

    //! Topic to wait for
    string topic;

    //! rng
    boost::mt19937 rng;
public:
    RandomTorqueApplier() :
        pnh("~")
    {
        gazeboClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench", true /* persistent */);
        notifier = nh.advertise<std_msgs::Header>("/human/fall", 1, true);
        pnh.param("x", x, 0.0);
        pnh.param("y", y, 0.0);
        pnh.param("z", z, 0.0);
        pnh.param("duration", duration, DURATION_DEFAULT);
        pnh.param("waitForTopic", waitForTopic, true);
        pnh.param("random", random, true);
        pnh.param<string>("topic", topic, "/balancer/torques");

        if (waitForTopic)
        {
            ros::service::waitForService(topic);
        }

        if (random)
        {
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

            // Generate total force
            boost::uniform_real<double> totalForceRange(150, 250);
            boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genTotal(rng, totalForceRange);
            double total = genTotal();

            // Now distribute between x and y
            boost::uniform_real<double> ratioRange(0, 1);
            boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genRatio(rng, ratioRange);
            double ratio = genRatio();

            x = total * ratio;
            y = total * (1 - ratio);

            // Distribute between positive and negative x force (which pushes along the y axis)
            boost::bernoulli_distribution<> negativeGenerator(0.5);
            x *= (negativeGenerator(rng) ? -1 : 1);
        }
    }

    void writeHeader(ofstream& outputCSV)
    {
        outputCSV << "Scenario Number, Wrench (x), Wrench (y), Wrench(z)" << endl;
    }

    void printWrench(const geometry_msgs::Wrench& wrench)
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

        const string resultsFileName = boost::filesystem::current_path().string() + "/" + string(resultsFolder) + "/" + "wrenches.csv";
        ROS_DEBUG_STREAM("Using results file: " << resultsFileName);
        bool exists = boost::filesystem::exists(resultsFileName);
        ofstream outputCSV;
        outputCSV.open(resultsFileName.c_str(), ios::out | ios::app);
        assert(outputCSV.is_open());

        if(!exists)
        {
            writeHeader(outputCSV);
        }

        outputCSV << scenarioNumberStr << ", " << wrench.torque.x << ", " << wrench.torque.y << ", " << wrench.torque.z << endl;
        outputCSV.close();
        ROS_DEBUG_STREAM("Printing output file complete");
    }

    void apply()
    {
    	ROS_INFO("Applying forces [%f, %f, %f]", x, y, z);
        gazebo_msgs::ApplyBodyWrench wrench;
        wrench.request.body_name = "human::link";
        wrench.request.wrench.torque.x = x;
        wrench.request.wrench.torque.y = y;
        wrench.request.wrench.torque.z = z;
        wrench.request.duration = ros::Duration(duration);

        printWrench(wrench.request.wrench);

        if (!gazeboClient.call(wrench))
        {
           ROS_ERROR("Failed to apply wrench");
           return;
        }

        ROS_DEBUG("Notifying");
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        notifier.publish(header);
        ROS_DEBUG("Torque application complete");
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_torque_applier");
    RandomTorqueApplier rta;

    // Allow the socket to connect
    ros::Duration(1.0).sleep();

    // Send the torque and message
    rta.apply();

    // Ensure the message is delivered
    ros::Duration(3.0).sleep();
}
