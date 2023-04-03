#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

#include <mutex>
#include <future>
#include <thread>
#include <fstream>


#include <rosbridge_ws_client.hpp>

namespace OS3 {

class OS3ROS {
    public:
        void init(void);
        int subscriber(void);
        int publisher(void);

        // struct ForceInput {
        //     SimTK::Vec3 force;
        //     double time;
        // };

        struct ForceInput {
            SimTK::Vec3 force;
            double time;
        };
        
        ForceInput get_latest_force(void); //THREADSAFE

        bool set_latest_force_time(SimTK::Vec3 forceIn, double tim); //THREADSAFE

        struct OS3Data {

            double timestamp; 
            
            SimTK::Vec3 wristPos;
            SimTK::Vec3 elbowPos;
            bool valid;
        };

        bool setPositionToPublish(OS3Data stateData);

        ~OS3ROS();

    private:

        //publishes state (use inside publisher thread)
        // bool publishState(OS3Data stateData);
        
        
        ForceInput _latestInput;

        //repeatedly publishes position, until pubExitSignal is received
        // void positionPublisherThread(RosbridgeWsClient& client, const std::future<void>& futureObj);
        std::promise<void> pubExitSignal;
        std::thread* pubTh;
        std::future<void> futureObj;

        std::promise<void> subExitSignal;
        std::thread* subTh;
        std::future<void> subFutureObj;
};
}