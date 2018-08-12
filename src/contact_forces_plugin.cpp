#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Header.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>

#define PRINT_SENSORS 0
#define PRINT_DEBUG 0

using namespace std;

namespace gazebo {

    static const std::string contactsLeft[] = {
        "l_wrist_roll",
        "l_gripper_r_finger",
        "l_gripper_r_finger_tip",
        "l_gripper_l_finger",
        "l_gripper_l_finger_tip",
        "l_wrist_flex",
        "l_forearm_roll",
        "l_elbow_flex",
        "l_upper_arm_roll",
        "l_shoulder_lift",
        "l_shoulder_pan"
    };

    static const std::string contactsRight[] = {
        "r_wrist_roll",
        "r_gripper_r_finger",
        "r_gripper_r_finger_tip",
        "r_gripper_l_finger",
        "r_gripper_l_finger_tip",
        "r_wrist_flex",
        "r_forearm_roll",
        "r_elbow_flex",
        "r_upper_arm_roll",
        "r_shoulder_lift",
        "r_shoulder_pan"
    };

    static const string humanLink = "human::link::collision";

    class ContactForcesPlugin : public ModelPlugin {

    private: bool collisionWithRobotOccurred;
    private: bool collisionWithHumanOccurred;
    private: gazebo::transport::NodePtr node;
    private: event::ConnectionPtr connection;
    private: vector<event::ConnectionPtr> sensorConnections;
    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: ros::NodeHandle nh;
    private: auto_ptr<message_filters::Subscriber<std_msgs::Header> > writeResultsSub;

    public: ContactForcesPlugin() : ModelPlugin() {
        #if(PRINT_DEBUG)
        cout << "Constructing the contact forces plugin" << std::endl;
        #endif
        collisionWithRobotOccurred = false;
        collisionWithHumanOccurred = false;
    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      world = _model->GetWorld();
      model = _model;

      cout << "Loading the contact forces plugin" << endl;

      #if(PRINT_SENSORS)
      cout << "Listing all sensors: " << endl;
      vector<sensors::SensorPtr> sensors = sensors::SensorManager::Instance()->GetSensors();
      for(unsigned int i = 0; i < sensors.size(); ++i){
        cout << sensors[i]->GetScopedName() << endl;
      }
      cout << endl;
      #endif

      for(unsigned int i = 0; i < boost::size(contactsLeft); ++i){
        sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor("default::pr2::" + contactsLeft[i] + "_link::" + contactsLeft[i] + "_contact_sensor");
        if(sensor == NULL){
            cout << "Could not find sensor " << contactsLeft[i] << endl;
            continue;
        }
        else {
            #if(PRINT_DEBUG)
            cout << "Registered contact for " << contactsLeft[i] << endl;
            #endif
        }
        sensor->SetActive(true);
        sensorConnections.push_back(sensor->ConnectUpdated(boost::bind(&ContactForcesPlugin::updateContactsLeft, this, sensor, contactsLeft[i])));
      }

      for(unsigned int i = 0; i < boost::size(contactsRight); ++i){
        sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor("default::pr2::" + contactsRight[i] + "_link::" + contactsRight[i] + "_contact_sensor");
        if(sensor == NULL){
            cout << "Could not find sensor " << contactsRight[i] << endl;
            continue;
        }
        else {
            #if(PRINT_DEBUG)
            cout << "Registered contact for " << contactsLeft[i] << endl;
            #endif
        }
        sensor->SetActive(true);
        sensorConnections.push_back(sensor->ConnectUpdated(boost::bind(&ContactForcesPlugin::updateContactsRight, this, sensor, contactsRight[i])));

        writeResultsSub.reset(new message_filters::Subscriber<std_msgs::Header>(nh, "/write_results", 1));
        writeResultsSub->registerCallback(boost::bind(&ContactForcesPlugin::update, this, _1));
        writeResultsSub->subscribe();
      }
    }

    private: void updateContactsLeft(sensors::SensorPtr sensor, const string& sensorName){
      // Get all the contacts.
      msgs::Contacts contacts = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor)->GetContacts();

      for (int i = 0; i < contacts.contact_size(); ++i){
           vector<string> strs;
           boost::split(strs, sensorName, boost::is_any_of("::"));

           string collidingLink = !boost::contains(contacts.contact(i).collision1(), strs[0]) ? contacts.contact(i).collision1() : contacts.contact(i).collision2();
#if PRINT_DEBUG
           cout << "Colliding link is: " << collidingLink << ". Options were: " << contacts.contact(i).collision1() << " and " << contacts.contact(i).collision2() << endl;
#endif
           if (collidingLink == humanLink) {
#if PRINT_DEBUG
                cout << "Contact with human occurred" << endl;
#endif
                collisionWithHumanOccurred = true;
           }
           else if (!boost::contains(collidingLink, "l_")) {
#if PRINT_DEBUG
                cout << "Contact is with a link from outside the arm " << collidingLink << endl;
#endif
                collisionWithRobotOccurred = true;
           }
           else {
#if PRINT_DEBUG
                cout << "Self collision occurred" << endl;
#endif
           }
        }
    }

    private: void updateContactsRight(sensors::SensorPtr sensor, const string& sensorName){
      // Get all the contacts.
      msgs::Contacts contacts = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor)->GetContacts();

      for (int i = 0; i < contacts.contact_size(); ++i){
           vector<string> strs;
           boost::split(strs, sensorName, boost::is_any_of("::"));

           string collidingLink = !boost::contains(contacts.contact(i).collision1(), strs[0]) ? contacts.contact(i).collision1() : contacts.contact(i).collision2();
#if PRINT_DEBUG
           cout << "Colliding link is: " << collidingLink << ". Options were: " << contacts.contact(i).collision1() << " and " << contacts.contact(i).collision2() << endl;
#endif
           if (collidingLink == humanLink) {
#if PRINT_DEBUG
                cout << "Contact with human occurred" << endl;
#endif
                collisionWithHumanOccurred = true;
           }
           else if (!boost::contains(collidingLink, "r_")) {
#if PRINT_DEBUG
                cout << "Contact is with a link from outside the arm " << collidingLink << endl;
#endif
                collisionWithRobotOccurred = true;
           }
           else {
#if PRINT_DEBUG
                cout << "Self collision occurred" << endl;
#endif
           }
        }
    }

    private: void writeHeader(ofstream& outputCSV){
      outputCSV << "Collision with Robot Occurred, Collission with Human Occurred " << endl;
    }

    private: void update(const std_msgs::HeaderConstPtr& imuData)
    {
        printResults();
    }

    private: void printResults(){
        // Get the name of the folder to store the result in
        const char* resultsFolder = std::getenv("RESULTS_FOLDER");
        if(resultsFolder == NULL)
        {
            cout << "Results folder not set. Using current directory." << endl;
            resultsFolder = "";
        }

        const char* scenarioNumberStr = std::getenv("i");
        if(scenarioNumberStr == NULL)
        {
            cout << "Scenario number not set. Using 0." << endl;
            scenarioNumberStr = "0";
        }

        const string resultsFileName = boost::filesystem::current_path().string() + "/" + string(resultsFolder) + "/" + "contacts.csv";
        cout << "Using contact results file: " << resultsFileName << endl;
        bool exists = boost::filesystem::exists(resultsFileName);
        ofstream outputCSV;
        outputCSV.open(resultsFileName.c_str(), ios::out | ios::app);
        assert(outputCSV.is_open());

        if(!exists)
        {
            writeHeader(outputCSV);
        }

        outputCSV << (collisionWithRobotOccurred ? "true" : "false") << ", " << (collisionWithHumanOccurred ? "true" : "false") << endl;

        outputCSV.close();
    }

    public: ~ContactForcesPlugin() {

        #if(PRINT_DEBUG)
        cerr << "Scenario completed. Writing contact results" << endl;
        #endif
        event::Events::DisconnectWorldUpdateBegin(this->connection);

        // Disconnect the sensors
        for(unsigned int i = 0; i < boost::size(contactsLeft); ++i){
            sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(world->GetName() + "::" + "pr2"
                                                       + "::" + contactsLeft[i]);
            if(sensor == NULL){
              cout << "Could not find sensor " << contactsLeft[i] << endl;
              continue;
            }
            sensor->SetActive(false);
            sensor->DisconnectUpdated(sensorConnections[i]);
          }

         for(unsigned int i = 0; i < boost::size(contactsRight); ++i){
            sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor(world->GetName() + "::" + "pr2"
                                                       + "::" + contactsRight[i]);
            if(sensor == NULL){
              cout << "Could not find sensor " << contactsRight[i] << endl;
              continue;
            }
            sensor->SetActive(false);
            sensor->DisconnectUpdated(sensorConnections[i]);
          }
          sensorConnections.clear();
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(ContactForcesPlugin);
}


