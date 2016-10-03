#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/random.hpp>
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
            unsigned int scenarioNumber = 0;
            if(scenarioNumberStr != NULL)
            {
                scenarioNumber = boost::lexical_cast<unsigned int>(scenarioNumberStr);
                seed = scenarioNumber + 1000;
            }
            else
            {
                cout << "No scenario number set. Using 0" << endl;
                seed = FIXED_SEED;
            }
#endif
            rng.seed(seed);


            bool found = false;
            while (!found)
            {
                // X generator
                boost::uniform_real<double> xRange(-250, 250);
                boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genX(rng, xRange);
                x = genX();

                // Y generator
                boost::uniform_real<double> yRange(0, 250);
                boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > genY(rng, yRange);
                y = genY();
                if (fabs(x) >= 150 || fabs(y) >= 150)
                {
                    found = true;
                }
            }
        }
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
    rta.apply();
    ros::Duration(3.0).sleep();
}
