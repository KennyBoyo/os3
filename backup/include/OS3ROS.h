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

        // struct Wrench {
        //     SimTK::Vec3 force;
        //     SimTK::Vec3 torque;
        // };

        // struct JointInfo {
        //     // Name of Joint
        //     std::string jointName;
        //     // Array of angles for Joint in order (if exists): Flexion/Extension, Abduction/Adduction, Internal(Medial)/External(Lateral) Rotation
        //     std::vector<double> angles;
        // };

        struct ProblemInput {
            std::vector<std::string> names;
            std::vector<double> angles;
            std::vector<double> velocities;
            std::vector<double> wrench;
            double time;
        };
        
        ProblemInput get_latest_problem(void);

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
        
        
        ProblemInput _latestProblem;
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