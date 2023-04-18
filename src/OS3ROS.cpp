#include <ctime>
#include <chrono>
#include <math.h>
#include "OS3ROS.h"

using namespace SimTK;
using namespace OpenSim;
using namespace std;

#define LOGGING 

namespace OS3ROS {

    //callbacks
    void problemSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
    // void createSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj, const std::string& clientName, const std::string& topicName, const InMessage& callback);
    void createSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj, const std::string& clientName, const std::string& topicName, const InMessage& callback);

    namespace {
        std::promise<void> problemSubExitSignal;
        std::thread* problemSubThread;
        std::future<void> problemSubFutureObj;

        //public within namespace
        RosbridgeWsClient RBcppClient("localhost:9090");

        // Force Message Storage
        std::mutex problemMutex;
        static OS3ROS::ProblemInput latestProblem;

        // Utilities
        
        void unwrap_json_double_array(const rapidjson::Value& docArray, std::vector<double>& arrayList) {
            for (rapidjson::Value::ConstValueIterator itr = docArray.Begin(); itr != docArray.End(); ++itr) {
                const rapidjson::Value& attribute = *itr;

                arrayList.push_back(attribute.GetDouble());
            }
        }
        /*converts clock ticks into seconds */
        double _clock_secs(clock_t ctim) ;
    }


    void init(void) {
        problemSubFutureObj = problemSubExitSignal.get_future();

        const std::string problemSubscriberThreadName = "problem_subscriber";
        const std::string problemSubscriberThreadTopic = "/problem_topic";

        // std::thread problemSubThread(&createSubscriberThread, std::ref(RBcppClient), std::cref(problemSubFutureObj), std::cref(problemSubscriberThreadName), std::cref(problemSubscriberThreadTopic));

        problemSubThread = new std::thread(&createSubscriberThread, std::ref(RBcppClient), std::cref(problemSubFutureObj), std::cref(problemSubscriberThreadName), std::cref(problemSubscriberThreadTopic), std::cref(problemSubscriberCallback));
    
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << " ...threads/clients created\n";
        
        return;
    }

    /* Threadsafe method to get the latest force that we've received from our subscriber thread
    */
    ProblemInput get_latest_problem(void) {
        return latestProblem;
    }

    /*
    Threadsafe method to set the latest force (alternative to callback fn)
    */
    bool set_latest_problem(ProblemInput problem) {

        if (problemMutex.try_lock()) {
            latestProblem = problem;
            problemMutex.unlock();
            return true;
        }
        return false;
    }

    void createSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj, const std::string& clientName, const std::string& topicName, const InMessage& callback) {

        std::cout << "Subscribing to " << topicName << endl;

        client.addClient(clientName);
        std::cout << "Added Client " << topicName << endl;
        client.subscribe(clientName, topicName, callback);
        std::cout << "Subscribed to " << topicName << endl;

        while(futureObj.wait_for(std::chrono::microseconds(500*10000)) == std::future_status::timeout) {
            //do nothing (subcription is handled by interrupts)
        }

    }

    // void createSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj, const std::string& clientName, const std::string& topicName) {

    //     std::cout << "Subscribing to " << topicName << endl;

    //     client.addClient(clientName);
    //     client.subscribe(clientName, topicName, problemSubscriberCallback);
    //     std::cout << "Subscribed to " << topicName << endl;

    //     while(futureObj.wait_for(std::chrono::microseconds(500*10000)) == std::future_status::timeout) {
    //         //do nothing (subcription is handled by interrupts)         
    //     }

    // }


    void problemSubscriberCallback(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message)
    {  

        auto callIn =  std::chrono::steady_clock::now();
        
        #undef LOGGING
        #ifdef LOGGING
        std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl; //THIS DESTROYS THE BUFFER AND SO CAN ONLY BE CALLED ONCE
        #endif
        
        rapidjson::Document problemDoc;

        if (problemDoc.Parse(in_message->string().c_str()).HasParseError() ) {
            std::cerr << "\n\nparse error\n" << std::endl;
        };

        assert(problemDoc.IsObject());    // Document is a JSON value represents the root of DOM. Root can be either an object or array

        // Check Header Present
        assert(problemDoc["msg"].HasMember("header"));
        assert(problemDoc["msg"]["header"].HasMember("stamp"));
        assert(problemDoc["msg"]["header"]["stamp"].HasMember("secs"));
        assert(problemDoc["msg"]["header"]["stamp"].HasMember("nsecs"));
        double timestamp = problemDoc["msg"]["header"]["stamp"]["secs"].GetUint() + ((double) (problemDoc["msg"]["header"]["stamp"]["nsecs"].GetUint()))/1000000000;
        
        #ifdef LOGGING
            std::cout << "Message Time: "  << timestamp << std::endl;
        #endif
        
        // Check Arm Joint Names Present
        assert(problemDoc["msg"].HasMember("name"));
        const rapidjson::Value& nameDoc = problemDoc["msg"]["name"];
        std::vector<std::string> nameList;
        for (rapidjson::Value::ConstValueIterator itr = nameDoc.Begin(); itr != nameDoc.End(); ++itr) {
            const rapidjson::Value& attribute = *itr;
            nameList.push_back(attribute.GetString());
        }

        // Arm joint position
        assert(problemDoc["msg"].HasMember("position"));
        const rapidjson::Value& posDoc = problemDoc["msg"]["position"];
        std::vector <double> posList;
        OS3ROS::unwrap_json_double_array(posDoc, posList);

        // Arm joint velocity (will be empty as of implementation)
        assert(problemDoc["msg"].HasMember("velocity"));
        const rapidjson::Value& velDoc = problemDoc["msg"]["velocity"];
        std::vector <double> velList;
        OS3ROS::unwrap_json_double_array(velDoc, velList);

        // End effector wrench
        assert(problemDoc["msg"].HasMember("effort"));
        const rapidjson::Value& wrenchDoc = problemDoc["msg"]["effort"];
        std::vector <double> wrenchList;
        OS3ROS::unwrap_json_double_array(wrenchDoc, wrenchList);

        OS3ROS::ProblemInput pi;
        pi.names.insert(pi.names.end(), nameList.begin(), nameList.end());
        pi.angles.insert(pi.angles.end(), posList.begin(), posList.end());
        pi.velocities.insert(pi.velocities.end(), velList.begin(), velList.end());
        pi.wrench.insert(pi.wrench.end(), wrenchList.begin(), wrenchList.end());
        SimTK::Vec3 force{pi.wrench[0], pi.wrench[1], pi.wrench[2]};
        SimTK::Vec3 torque{pi.wrench[3], pi.wrench[4], pi.wrench[5]};
        
        pi.forceMag = sqrt(~force * force);
        if (pi.forceMag == 0) {
            pi.forceDirection = {1,1,1}; //no force anyway
        } else {
            pi.forceDirection = force / pi.forceMag;
        }

        pi.torqueMag = sqrt(~torque * torque);
        if (pi.torqueMag == 0) {
            pi.torqueDirection = {1,1,1}; //no force anyway
        } else {
            pi.torqueDirection = torque / pi.torqueMag;
        }
        
        timestamp = -1;
        if (timestamp < 0) { //DEBUG CODE, USED TO SEND REPEATED INPUT WITHOUT FRANKA
            pi.timestamp = latestProblem.timestamp + 0.002;
        } else {
            pi.timestamp = timestamp;
        }

        OS3ROS::set_latest_problem(pi);

        auto callOut = std::chrono::steady_clock::now();
        std::chrono::duration<double> callDur = std::chrono::duration_cast<std::chrono::duration<double>>(callOut - callIn);
        #ifdef LOGGING
            std::cout << " calltime: " << callDur.count() << std::endl;
        #endif
    }

}