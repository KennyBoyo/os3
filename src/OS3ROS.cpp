#include <ctime>
#include <chrono>
#include <math.h>
#include "OS3ROS.h"

using namespace SimTK;
using namespace OpenSim;
using namespace std;
using namespace OS3;


#define LOGGING 

/*converts clock ticks into seconds */
double _clock_secs(clock_t ctim) ;

//callbacks (will need more)
static void advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
static void forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
static void positionPublisherThread(RosbridgeWsClient& client, const std::future<void>& futureObj);
static void forceSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj);

bool publishState(OS3ROS::OS3Data stateData);

//public within namespace
RosbridgeWsClient RBcppClient("localhost:9090");


// Force Message Storage
std::mutex fInMutex;
static SimTK::Vec3 latestForce;
static double latestTime;

static SimTK::Vec3 forceBuffer[10];
static int forceBufferIndex;


// Position Publisher Globals
std::mutex posOutMutex;
static OS3ROS::OS3Data latestPositionData; // should be accessed only through posOutMutex!!
static bool _newPosAvailable; //represents whether the data in latestPositionData has been published yet (use only with posOutMutex!)

/* This and other rosbridgecpp functions are created based on the rosbridgecpp library and example code
    See licences for details
    */
void OS3ROS::init(void) {

    std::cout << "OS3 comms init..." ;
    
    
    if (fInMutex.try_lock()) {
        latestForce = {0,0,0};
        latestTime = 0.0;
        fInMutex.unlock();
    }

    for (int i = 0; i < 10; i++) {
        forceBuffer[i].setToZero();
    }    
    forceBufferIndex = 0;

    RBcppClient.addClient("service_advertiser");
    RBcppClient.advertiseService("service_advertiser", "/OS3service", "std_srvs/SetBool", advertiserCallback);
    
    RBcppClient.addClient("topic_advertiser");
    RBcppClient.advertise("topic_advertiser", "/OS3topic", "franka_panda_controller_swc/ArmJointPos");

    subFutureObj = subExitSignal.get_future();
    subTh = new std::thread(&forceSubscriberThread, std::ref(RBcppClient), std::cref(subFutureObj));

    
    

    // Fetch std::future object associated with promise
    futureObj = pubExitSignal.get_future();

    pubTh = new std::thread(&positionPublisherThread, std::ref(RBcppClient), std::cref(futureObj));
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << " ...threads/clients created\n";
    
    
    
    return;
}

/* Threadsafe method to get the latest force that we've received from our subscriber thread
*/
OS3ROS::ForceInput OS3ROS::get_latest_force(void) {
    if (fInMutex.try_lock()) {
        _latestInput.force = latestForce;
        _latestInput.time = latestTime;

        fInMutex.unlock();
    }
    
    
  

    //threadsafe
    return _latestInput;
}
/*
Threadsafe method to set the latest force (alternative to callback fn)
*/
bool OS3ROS::set_latest_force_time(SimTK::Vec3 forceIn, double tim) {

    if (fInMutex.try_lock()) {
        latestForce = forceIn;
        latestTime = tim;
        fInMutex.unlock();
        return true;
    }
    return false;
}


//threadsafe function to add the latest force to publishing queue
bool OS3ROS::setPositionToPublish(OS3Data stateData) {

    if (posOutMutex.try_lock()) {
        latestPositionData = stateData;
        
        _newPosAvailable = true; //mark that there's a new position available
                        //NOTE: if the publisher hasn't published
                                // the previous data point
                                // then it will be overwritten
        
        posOutMutex.unlock();
        return true; //position recorded
    }
    return false; // position not recorded (mutex was already locked)
}

bool publishState(OS3ROS::OS3Data stateData) {

    if (stateData.valid == false) {

        std::cout << "INVALID DATA\n";
        return false;
    }

    rapidjson::Document d;
    d.SetObject();

    rapidjson::Value msg(rapidjson::kObjectType);


    //create wrist values
    rapidjson::Value wristVec(rapidjson::kObjectType);
    rapidjson::Value wX;
    rapidjson::Value wY;
    rapidjson::Value wZ;
    wX.SetDouble(stateData.wristPos[0]);
    wY.SetDouble(stateData.wristPos[1]);
    wZ.SetDouble(stateData.wristPos[2]);
    wristVec.AddMember("x",wX,d.GetAllocator());
    wristVec.AddMember("y",wY,d.GetAllocator());
    wristVec.AddMember("z",wZ,d.GetAllocator());

    //create elbow values
    rapidjson::Value elbowVec(rapidjson::kObjectType);
    rapidjson::Value eX;
    rapidjson::Value eY;
    rapidjson::Value eZ;
    eX.SetDouble(stateData.elbowPos[0]);
    eY.SetDouble(stateData.elbowPos[1]);
    eZ.SetDouble(stateData.elbowPos[2]);
    elbowVec.AddMember("x",eX,d.GetAllocator());
    elbowVec.AddMember("y",eY,d.GetAllocator());
    elbowVec.AddMember("z",eZ,d.GetAllocator());

    //create time values
    rapidjson::Value timestamp;
    timestamp.SetDouble(stateData.timestamp);

    //add values to d
    d.AddMember("wrist",wristVec,d.GetAllocator());
    d.AddMember("elbow",elbowVec,d.GetAllocator());
    d.AddMember("time",timestamp,d.GetAllocator());
    
    // std::cout << d["wrist"]["x"].GetDouble() << "<--wrist pos\n";
    // std::cout << "    published stuff to /OS3topic   ";
    RBcppClient.publish("/OS3topic",d);    //TODO Currently the rosbridgecpp library 
                    //opens and closes a new connection each time a message is sent. 
                    //This is computationally expensive and so the library should be modified accordingly


    return true; //when should we return false? //TODO
}

// to test (from CMD line):
// rostopic pub -r 10 /twistfromCMD geometry_msgs/Twist  "{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.0}}"
// roslaunch rosbridge_server rosbridge_websocket.launch

//deletes threads when program exits
OS3ROS::~OS3ROS() {
    pubExitSignal.set_value();
    pubTh->join();
    std::cerr << "EXITING OS3ROS\n";

}


//private definitions
void advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message)
{
    // message->string() is destructive, so we have to buffer it first
    std::string messagebuf = in_message->string();
    std::cout << "advertiseServiceCallback(): Message Received: " << messagebuf << std::endl;

    rapidjson::Document document;
    if (document.Parse(messagebuf.c_str()).HasParseError())
    {
    std::cerr << "advertiseServiceCallback(): Error in parsing service request message: " << messagebuf << std::endl;
    return;
    }

    rapidjson::Document values(rapidjson::kObjectType);
    rapidjson::Document::AllocatorType& allocator = values.GetAllocator();
    values.AddMember("success", document["args"]["data"].GetBool(), allocator);
    values.AddMember("message", "from advertiseServiceCallback", allocator);

    // RBcppClientptr->addClient("testclient");
    RBcppClient.serviceResponse(document["service"].GetString(), document["id"].GetString(), true, values);


}


// void forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message)
// {  

//     // auto callIn =  std::chrono::steady_clock::now();
//     #undef DEBUG
//     #ifdef DEBUG
//     std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl; //THIS DESTROYS THE BUFFER AND SO CAN ONLY BE CALLED ONCE
//     #endif 
//     // #define DEBUG
    
  
//     rapidjson::Document forceD;

//     // char *cstr = new char[in_message->string().length() + 1];
//     // strcpy(cstr, in_message->string().c_str());
//     // std::cout << "msg length : " << in_message->string().length() << std::endl;
//     // std::cout << "actual string received : " << cstr << std::endl;
//     // std::cerr << "parse error: " << forceD.Parse(in_message->string().c_str()).GetParseError() << std::endl; while(1) {}
//     if (forceD.Parse(in_message->string().c_str()).HasParseError() ) {
//         std::cerr << "\n\nparse error\n" << std::endl;
//     };

//     #ifdef DEBUG
//         rapidjson::StringBuffer buffer;
//         buffer.Clear();
//         rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
//         forceD.Accept(writer);
//         std::cout << std::endl << "msg received:   " << buffer.GetString() << std::endl << std::endl;
//     #endif

    
//     assert(forceD.IsObject());    // Document is a JSON value represents the root of DOM. Root can be either an object or array.

    

//     assert(forceD.HasMember("msg"));
//     #ifdef TWIST_TEST
//     assert(forceD["msg"].HasMember("linear"));
    
//     assert(forceD["msg"]["linear"].HasMember("x"));
//     assert(forceD["msg"]["linear"]["x"].IsDouble());

//     // std::cout << forceD["msg"]["linear"]["x"].GetDouble() << std::endl;

//     double x = forceD["msg"]["linear"]["x"].GetDouble();
//     double y = forceD["msg"]["linear"]["y"].GetDouble();
//     double z = forceD["msg"]["linear"]["z"].GetDouble();

//     #endif

//     assert(forceD["msg"].HasMember("force"));
//     assert(forceD["msg"]["force"].HasMember("x"));
//     assert(forceD["msg"].HasMember("time"));
//     assert(forceD["msg"]["force"]["x"].IsDouble());
//     assert(forceD["msg"]["time"].IsDouble());

//     double x = forceD["msg"]["force"]["x"].GetDouble();
//     double y = forceD["msg"]["force"]["y"].GetDouble();
//     double z = forceD["msg"]["force"]["z"].GetDouble();
//     double timestamp = forceD["msg"]["time"].GetDouble();

//         forceBuffer[forceBufferIndex] = {x,y,z};
//         forceBufferIndex++;
//         forceBufferIndex = forceBufferIndex % 10; //cycle through each value in array 

//         SimTK::Vec3 fSum = {0,0,0};

//         for (int i = 0; i < 10; i++) {
//             fSum = fSum + forceBuffer[i];
//         }
//         fSum = fSum / 10;

//     {
//         fInMutex.lock(); //locks mutex
        
        

//         //save to global variable
//         // latestForce = {x,y,z};
//         latestForce = fSum;
//         if (timestamp < 0) { //DEBUG CODE, USED TO SEND REPEATED INPUT WITHOUT FRANKA
//             latestTime += 0.002;
//         } else {
//             latestTime = timestamp;
//         }
//         fInMutex.unlock(); // unlocks mutex
//     }
//     std::cout << " fReceived x: " << x << " y: " << y << " z: " << z << "time: " << timestamp << std::endl;
    

//     // auto callOut = std::chrono::steady_clock::now();
//     // std::chrono::duration<double> callDur = std::chrono::duration_cast<std::chrono::duration<double>>(callOut - callIn);
//     // std::cout << " calltime: " << callDur.count() << " ";
// }

void forceSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message)
{  

    // auto callIn =  std::chrono::steady_clock::now();
    #undef DEBUG
    #ifdef DEBUG
    std::cout << "subscriberCallback(): Message Received: " << in_message->string() << std::endl; //THIS DESTROYS THE BUFFER AND SO CAN ONLY BE CALLED ONCE
    #endif 
    // #define DEBUG
    
  
    rapidjson::Document forceD;

    // char *cstr = new char[in_message->string().length() + 1];
    // strcpy(cstr, in_message->string().c_str());
    // std::cout << "msg length : " << in_message->string().length() << std::endl;
    // std::cout << "actual string received : " << cstr << std::endl;
    // std::cerr << "parse error: " << forceD.Parse(in_message->string().c_str()).GetParseError() << std::endl; while(1) {}
    if (forceD.Parse(in_message->string().c_str()).HasParseError() ) {
        std::cerr << "\n\nparse error\n" << std::endl;
    };

    #ifdef DEBUG
        rapidjson::StringBuffer buffer;
        buffer.Clear();
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        forceD.Accept(writer);
        std::cout << std::endl << "msg received:   " << buffer.GetString() << std::endl << std::endl;
    #endif

    
    assert(forceD.IsObject());    // Document is a JSON value represents the root of DOM. Root can be either an object or array.

    

    assert(forceD.HasMember("msg"));
    #ifdef TWIST_TEST
    assert(forceD["msg"].HasMember("linear"));
    
    assert(forceD["msg"]["linear"].HasMember("x"));
    assert(forceD["msg"]["linear"]["x"].IsDouble());

    // std::cout << forceD["msg"]["linear"]["x"].GetDouble() << std::endl;

    double x = forceD["msg"]["linear"]["x"].GetDouble();
    double y = forceD["msg"]["linear"]["y"].GetDouble();
    double z = forceD["msg"]["linear"]["z"].GetDouble();

    #endif

    assert(forceD["msg"].HasMember("wrench"));
    assert(forceD["msg"]["wrench"].HasMember("force"));
    assert(forceD["msg"]["wrench"]["force"].HasMember("x"));
    assert(forceD["msg"]["wrench"]["force"].HasMember("y"));
    assert(forceD["msg"]["wrench"]["force"].HasMember("z"));
    assert(forceD["msg"]["wrench"]["force"]["x"].IsDouble());
    assert(forceD["msg"]["wrench"]["force"]["y"].IsDouble());
    assert(forceD["msg"]["wrench"]["force"]["z"].IsDouble());

    assert(forceD["msg"]["wrench"].HasMember("torque"));
    assert(forceD["msg"]["wrench"]["torque"].HasMember("x"));
    assert(forceD["msg"]["wrench"]["torque"].HasMember("y"));
    assert(forceD["msg"]["wrench"]["torque"].HasMember("z"));
    assert(forceD["msg"]["wrench"]["torque"]["x"].IsDouble());
    assert(forceD["msg"]["wrench"]["torque"]["y"].IsDouble());
    assert(forceD["msg"]["wrench"]["torque"]["z"].IsDouble());

    assert(forceD["msg"].HasMember("header"));
    assert(forceD["msg"]["header"].HasMember("stamp"));
    assert(forceD["msg"]["header"]["stamp"].HasMember("secs"));
    assert(forceD["msg"]["header"]["stamp"].HasMember("nsecs"));
    assert(forceD["msg"]["header"]["stamp"]["secs"].IsUint());
    assert(forceD["msg"]["header"]["stamp"]["nsecs"].IsUint());

    double x = forceD["msg"]["wrench"]["force"]["x"].GetDouble();
    double y = forceD["msg"]["wrench"]["force"]["y"].GetDouble();
    double z = forceD["msg"]["wrench"]["force"]["z"].GetDouble();

    // double timestamp = forceD["msg"]["header"]["stamp"]["secs"].GetUint() + (double) forceD["msg"]["header"]["stamp"]["secs"].GetUint()/pow(10, 9);
    // double timestamp = forceD["msg"]["stamp"]["time"].GetDouble();
    double timestamp = -1.0;

        forceBuffer[forceBufferIndex] = {x,y,z};
        forceBufferIndex++;
        forceBufferIndex = forceBufferIndex % 10; //cycle through each value in array 

        SimTK::Vec3 fSum = {0,0,0};

        for (int i = 0; i < 10; i++) {
            fSum = fSum + forceBuffer[i];
        }
        fSum = fSum / 10;

    {
        fInMutex.lock(); //locks mutex
        
        

        //save to global variable
        // latestForce = {x,y,z};
        latestForce = fSum;
        if (timestamp < 0) { //DEBUG CODE, USED TO SEND REPEATED INPUT WITHOUT FRANKA
            latestTime += 0.002;
        } else {
            latestTime = timestamp;
        }
        fInMutex.unlock(); // unlocks mutex
    }
    std::cout << " fReceived x: " << x << " y: " << y << " z: " << z << "time: " << timestamp << std::endl;
    

    // auto callOut = std::chrono::steady_clock::now();
    // std::chrono::duration<double> callDur = std::chrono::duration_cast<std::chrono::duration<double>>(callOut - callIn);
    // std::cout << " calltime: " << callDur.count() << " ";
}

void positionPublisherThread(RosbridgeWsClient& client, const std::future<void>& futureObj) {

    std::cout << "pos publisher";

    client.addClient("position_publisher");

    OS3ROS::OS3Data data;
    bool newData = false;
    // std::this_thread::sleep_for(std::chrono::seconds(10)); std::cout << "pub thread: ready for data";
    while(futureObj.wait_for(std::chrono::microseconds(500)) == std::future_status::timeout) {
        
        if (posOutMutex.try_lock()) {
                if (_newPosAvailable) {
                    data = latestPositionData;
                    newData = true; //for this thread's reference
                    _newPosAvailable = false; // we've accessed this data point, so mark it as old
                }
            posOutMutex.unlock();
        }
        if (newData) {
            publishState(data);
            newData = false;            
            // std::cout << "             PUBTHREAD: pubbed!";
        }
        
    }

    client.removeClient("position_publisher");
    std::cout << "publisher thread has stopped" << std::endl;

    return;
}


void forceSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj) {

    std::cout << " force subscriber, ";


    client.addClient("topic_subscriber"); //TODO: put this in its own thread
    // RBcppClient.subscribe("topic_subscriber", "cartesian_impedance_controller_NR/force_output",forceSubscriberCallback);
    // client.subscribe("topic_subscriber", "/ROSforceOutput",forceSubscriberCallback);
    client.subscribe("topic_subscriber", "/franka_state_controller/F_ext",forceSubscriberCallback);


    while(futureObj.wait_for(std::chrono::microseconds(500*10000)) == std::future_status::timeout) {
        //do nothing (subcription is handled by interrupts)
    }

}