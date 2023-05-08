#pragma once

#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

#include <mutex>
#include <future>
#include <thread>
#include <fstream>


#include <rosbridge_ws_client.hpp>

namespace OS3ROS {

    void init(void);

    struct ProblemInput {
        std::vector<std::string> names;
        std::vector<double> angles;
        std::vector<double> velocities;
        std::vector<double> wrench;
        SimTK::Vec3 forceDirection;
        SimTK::Vec3 torqueDirection;
        double forceMag;
        double torqueMag;
        double timestamp;
    };

    struct ProblemOutput {
        std::vector<std::string> names;
        std::vector<double> angles;
        std::vector<double> velocities;
        std::vector<double> torques;
    };
    
    ProblemInput get_latest_problem(void);

    bool set_latest_solution(ProblemOutput problemOutput);
    bool set_latest_problem(ProblemInput problem);
}