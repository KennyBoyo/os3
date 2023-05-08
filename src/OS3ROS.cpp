#include <ctime>
#include <chrono>
#include <math.h>
#include "OS3ROS.h"
#include "OS3Engine.h"

using namespace SimTK;
using namespace OpenSim;
using namespace std;
using namespace OS3;

#define LOGGING

namespace OS3ROS
{

    // callbacks
    void problemSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
    // void createSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj, const std::string& clientName, const std::string& topicName, const InMessage& callback);
    void createSubscriberThread(RosbridgeWsClient &client, const std::future<void> &futureObj, const std::string &clientName, const std::string &topicName, const InMessage &callback);
    void solutionPublisherThread(RosbridgeWsClient &client, const std::future<void> &futureObj);
    bool publishState(OS3ROS::ProblemOutput problemOutput);

    namespace
    {
        std::promise<void> problemSubExitSignal;
        std::thread *problemSubThread;
        std::future<void> problemSubFutureObj;

        std::promise<void> problemPubExitSignal;
        std::thread *problemPubThread;
        std::future<void> problemPubFutureObj;

        // public within namespace
        RosbridgeWsClient RBcppClient("127.0.0.1:9090");
        // RosbridgeWsClient RBcppClient("172.16.0.102:9090");

        // Force Message Storage
        std::mutex problemMutex;
        static OS3ROS::ProblemInput latestProblem;
        std::mutex solutionMutex;
        static OS3ROS::ProblemOutput latestSolution;
        bool _newSolutionAvailable;

        // Utilities

        void unwrap_json_double_array(const rapidjson::Value &docArray, std::vector<double> &arrayList)
        {
            for (rapidjson::Value::ConstValueIterator itr = docArray.Begin(); itr != docArray.End(); ++itr)
            {
                const rapidjson::Value &attribute = *itr;

                arrayList.push_back(attribute.GetDouble());
            }
        }
        /*converts clock ticks into seconds */
        double _clock_secs(clock_t ctim);
    }

    void init(void)
    {
        _newSolutionAvailable = false;
        problemSubFutureObj = problemSubExitSignal.get_future();

        const std::string problemSubscriberThreadName = "problem_subscriber";
        const std::string problemSubscriberThreadTopic = "/os3/synchronised_step_problem";

        // std::thread problemSubThread(&createSubscriberThread, std::ref(RBcppClient), std::cref(problemSubFutureObj), std::cref(problemSubscriberThreadName), std::cref(problemSubscriberThreadTopic));

        RBcppClient.addClient("topic_advertiser");
        // RBcppClient.advertise("topic_advertiser", "/os3/step_problem_solution", "sensor_msgs/JointState");
        RBcppClient.advertise("topic_advertiser", "/os3/step_problem_solution", "sensor_msgs/JointState");

        problemSubThread = new std::thread(&createSubscriberThread, std::ref(RBcppClient), std::cref(problemSubFutureObj), std::cref(problemSubscriberThreadName), std::cref(problemSubscriberThreadTopic), std::cref(problemSubscriberCallback));

        problemPubFutureObj = problemPubExitSignal.get_future();
        problemPubThread = new std::thread(&solutionPublisherThread, std::ref(RBcppClient), std::cref(problemPubFutureObj));
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << " ...threads/clients created\n";

        return;
    }

    /* Threadsafe method to get the latest force that we've received from our subscriber thread
     */
    ProblemInput get_latest_problem(void)
    {
        return latestProblem;
    }

    /*
    Threadsafe method to set the latest force (alternative to callback fn)
    */
    bool set_latest_problem(ProblemInput problem)
    {

        if (problemMutex.try_lock())
        {
            latestProblem = problem;
            problemMutex.unlock();
            return true;
        }
        return false;
    }

    void createSubscriberThread(RosbridgeWsClient &client, const std::future<void> &futureObj, const std::string &clientName, const std::string &topicName, const InMessage &callback)
    {

        std::cout << "Subscribing to " << topicName << endl;

        client.addClient(clientName);
        std::cout << "Added Client " << topicName << endl;
        client.subscribe(clientName, topicName, callback);
        std::cout << "Subscribed to " << topicName << endl;

        while (futureObj.wait_for(std::chrono::microseconds(500 * 10000)) == std::future_status::timeout)
        {
            // do nothing (subcription is handled by interrupts)
        }
    }

    /*angles
    Threadsafe method to set the latest force (alternative to callback fn)
    */
    bool set_latest_solution(ProblemOutput problemOutput)
    {

        if (solutionMutex.try_lock())
        {
            latestSolution = problemOutput;
            _newSolutionAvailable = true;
            solutionMutex.unlock();
            return true;
        }
        return false;
    }

    void solutionPublisherThread(RosbridgeWsClient &client, const std::future<void> &futureObj)
    {

        std::cout << "solution publisher";

        client.addClient("solution_publisher");

        OS3ROS::ProblemOutput problemOutput;
        bool newData = false;
        // std::this_thread::sleep_for(std::chrono::seconds(10)); std::cout << "pub thread: ready for data";
        while (futureObj.wait_for(std::chrono::microseconds(500)) == std::future_status::timeout)
        {

            if (solutionMutex.try_lock())
            {
                if (_newSolutionAvailable)
                {
                    problemOutput = latestSolution;
                    newData = true;                // for this thread's reference
                    _newSolutionAvailable = false; // we've accessed this data point, so mark it as old
                }
                solutionMutex.unlock();
            }
            if (newData)
            {
                publishState(problemOutput);
                newData = false;
                // std::cout << "             PUBTHREAD: pubbed!";
            }
        }

        client.removeClient("solution_publisher");
        std::cout << "publisher thread has stopped" << std::endl;

        return;
    }

    void JSONArrayFromDoubles(rapidjson::Document d, std::string fieldName, std::vector<double> vec)
    {
        // d.SetObject();
        rapidjson::Value arr(rapidjson::kArrayType);

        for (int i = 0; i < vec.size(); i++)
        {
            arr.PushBack(vec[i], d.GetAllocator()); // allocator is needed for potential realloc().
        }

        rapidjson::Value str(rapidjson::kStringType);
        str.SetString(fieldName.c_str(), fieldName.size());

        d.AddMember(str, arr, d.GetAllocator());
    }

    void JSONArrayFromStrings(rapidjson::Document d, std::string fieldName, std::vector<std::string> vec)
    {
        // d.SetObject();
        rapidjson::Value arr(rapidjson::kArrayType);

        for (int i = 0; i < vec.size(); i++)
        {
            arr.PushBack(rapidjson::Value(vec[i].c_str(), vec[i].size()), d.GetAllocator()); // allocator is needed for potential realloc().
        }

        rapidjson::Value str(rapidjson::kStringType);
        str.SetString(fieldName.c_str(), fieldName.size());

        d.AddMember(str, arr, d.GetAllocator());
    }

    bool publishState(OS3ROS::ProblemOutput problemOutput)
    {

        // std::cout << problemOutput.names[0] << std::endl;
        // std::cout << problemOutput.angles[0] << std::endl;
        // std::cout << problemOutput.velocities[0] << std::endl;
        // std::cout << problemOutput.torques[0] << std::endl;

        // struct ProblemOutput {
        //     std::vector<std::string> names;
        //     std::vector<double> angles;
        //     d.AddMember("names", names, d.GetAllocator());std::vector<double> velocities;
        //     std::vector<double> torques;
        // };

        // for (auto i = 0; i < problemOutput.names.size(); i++)
        // {
        //     std::cout << "name" << problemOutput.names[i] << std::endl;
        // }
        // for (auto i = 0; i < problemOutput.angles.size(); i++)
        // {
        //     std::cout << "angles" << problemOutput.angles[i] << std::endl;
        // }
        // for (auto i = 0; i < problemOutput.velocities.size(); i++)
        // {
        //     std::cout << "velocities" << problemOutput.velocities[i] << std::endl;
        // }
        // for (auto i = 0; i < problemOutput.torques.size(); i++)
        // {
        //     std::cout << "wrench" << problemOutput.torques[i] << std::endl;
        // }

        rapidjson::Document d(rapidjson::kStringType);
        d.SetObject();
        rapidjson::Value header(rapidjson::kObjectType);
        rapidjson::Value stamp(rapidjson::kObjectType);
        // rapidjson::Value secs(0);
        // rapidjson::Value nsecs(0);
        rapidjson::Value frame_id("panda_link0");
        rapidjson::Value seq(1);

        header.AddMember("seq", seq, d.GetAllocator());
        header.AddMember("stamp", stamp, d.GetAllocator());
        header.AddMember("frame_id", frame_id, d.GetAllocator());

        // JSONArrayFromStrings(d, std::string("names"), problemOutput.names);

        rapidjson::Value names(rapidjson::kArrayType);
        rapidjson::Value angles(rapidjson::kArrayType);
        rapidjson::Value velocities(rapidjson::kArrayType);
        rapidjson::Value torques(rapidjson::kArrayType);

        for (int i = 0; i < problemOutput.names.size(); i++)
        {
            names.PushBack(rapidjson::Value(problemOutput.names[i].c_str(), problemOutput.names[i].size()), d.GetAllocator()); // allocator is needed for potential realloc().
            // names.PushBack(rapidjson::Value("name", d.GetAllocator()).Move(), d.GetAllocator());   // allocator is needed for potential realloc().
        }
        for (int i = 0; i < problemOutput.angles.size(); i++)
        {
            angles.PushBack(rapidjson::Value(problemOutput.angles[i]).GetDouble(), d.GetAllocator()); // allocator is needed for potential realloc().
            // angles.PushBack(1, d.GetAllocator());   // allocator is needed for potential realloc().
        }
        for (int i = 0; i < problemOutput.velocities.size(); i++)
        {
            velocities.PushBack(rapidjson::Value(problemOutput.velocities[i]), d.GetAllocator()); // allocator is needed for potential realloc().
            // velocities.PushBack(1, d.GetAllocator());   // allocator is needed for potential realloc().
        }
        for (int i = 0; i < problemOutput.torques.size(); i++)
        {
            torques.PushBack(rapidjson::Value(problemOutput.torques[i]), d.GetAllocator()); // allocator is needed for potential realloc().
            // torques.PushBack(1, d.GetAllocator());   // allocator is needed for potential realloc().
        }

        d.AddMember("name", names, d.GetAllocator());
        d.AddMember("position", angles, d.GetAllocator());
        d.AddMember("velocity", velocities, d.GetAllocator());
        d.AddMember("effort", torques, d.GetAllocator());
        d.AddMember("header", header, d.GetAllocator());

        // rapidjson::StringBuffer buffer;
        // rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        // d.Accept(writer);

        // std::cout << "publishing" << buffer.GetString() << std::endl;

        RBcppClient.publish("/os3/step_problem_solution", d);
        return true;
    }

    void problemSubscriberCallback(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message)
    {

        auto callIn = std::chrono::steady_clock::now();
        // std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl; //THIS DESTROYS THE BUFFER AND SO CAN ONLY BE CALLED ONCE

#undef LOGGING
#ifdef LOGGING
        std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl; // THIS DESTROYS THE BUFFER AND SO CAN ONLY BE CALLED ONCE
#endif

        rapidjson::Document problemDoc;

        if (problemDoc.Parse(in_message->string().c_str()).HasParseError())
        {
            std::cerr << "\n\nparse error\n"
                      << std::endl;
        };

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        problemDoc.Accept(writer);

        // std::cout << "Received messsage " << buffer.GetString() << std::endl;

        assert(problemDoc.IsObject()); // Document is a JSON value represents the root of DOM. Root can be either an object or array

        // Check Header Present
        assert(problemDoc["msg"].HasMember("header"));
        assert(problemDoc["msg"]["header"].HasMember("stamp"));
        assert(problemDoc["msg"]["header"]["stamp"].HasMember("secs"));
        assert(problemDoc["msg"]["header"]["stamp"].HasMember("nsecs"));
        double timestamp = problemDoc["msg"]["header"]["stamp"]["secs"].GetUint() + ((double)(problemDoc["msg"]["header"]["stamp"]["nsecs"].GetUint())) / 1000000000;

#ifdef LOGGING
        std::cout << "Message Time: " << timestamp << std::endl;
#endif

        // Check Arm Joint Names Present
        assert(problemDoc["msg"].HasMember("name"));
        const rapidjson::Value &nameDoc = problemDoc["msg"]["name"];
        std::vector<std::string> nameList;
        for (rapidjson::Value::ConstValueIterator itr = nameDoc.Begin(); itr != nameDoc.End(); ++itr)
        {
            const rapidjson::Value &attribute = *itr;
            nameList.push_back(attribute.GetString());
        }

        // Arm joint position
        assert(problemDoc["msg"].HasMember("position"));
        const rapidjson::Value &posDoc = problemDoc["msg"]["position"];
        std::vector<double> posList;
        OS3ROS::unwrap_json_double_array(posDoc, posList);

        // Arm joint velocity (will be empty as of implementation)
        assert(problemDoc["msg"].HasMember("velocity"));
        const rapidjson::Value &velDoc = problemDoc["msg"]["velocity"];
        std::vector<double> velList;
        OS3ROS::unwrap_json_double_array(velDoc, velList);

        // End effector wrench
        assert(problemDoc["msg"].HasMember("effort"));
        const rapidjson::Value &wrenchDoc = problemDoc["msg"]["effort"];
        std::vector<double> wrenchList;
        OS3ROS::unwrap_json_double_array(wrenchDoc, wrenchList);

        OS3ROS::ProblemInput pi;
        pi.names.insert(pi.names.end(), nameList.begin(), nameList.end());
        pi.angles.insert(pi.angles.end(), posList.begin(), posList.end());
        pi.velocities.insert(pi.velocities.end(), velList.begin(), velList.end());
        pi.wrench.insert(pi.wrench.end(), wrenchList.begin(), wrenchList.end());
        SimTK::Vec3 force{pi.wrench[0], pi.wrench[1], pi.wrench[2]};
        SimTK::Vec3 torque{pi.wrench[3], pi.wrench[4], pi.wrench[5]};
        // pi.header = problemDoc["msg"]["header"];

        pi.forceMag = sqrt(~force * force);
        if (pi.forceMag == 0)
        {
            pi.forceDirection = {1, 1, 1}; // no force anyway
        }
        else
        {
            pi.forceDirection = force / pi.forceMag;
        }

        pi.torqueMag = sqrt(~torque * torque);
        if (pi.torqueMag == 0)
        {
            pi.torqueDirection = {1, 1, 1}; // no force anyway
        }
        else
        {
            pi.torqueDirection = torque / pi.torqueMag;
        }

        timestamp = -1;
        if (timestamp < 0)
        { // DEBUG CODE, USED TO SEND REPEATED INPUT WITHOUT FRANKA
            pi.timestamp = latestProblem.timestamp + 0.002;
        }
        else
        {
            pi.timestamp = timestamp;
        }

        OS3ROS::set_latest_problem(pi);

        auto callOut = std::chrono::steady_clock::now();
        std::chrono::duration<double> callDur = std::chrono::duration_cast<std::chrono::duration<double>>(callOut - callIn);
#undef LOGGING
#ifdef LOGGING
        std::cout << " calltime: " << callDur.count() << std::endl;
#endif
        // #undef LOGGING
    }

}