#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include "OS3ROS.h"

#define ENGINE_STANDBY 0
#define ENGINE_RUN 1
#define ENGINE_END_EXPERIMENT 2

#define OPTIMAL_TORQUE 100 //N m

namespace OS3 {

class OS3Engine {

    public:
        void init(void);
        void loop(void);

        int get_state(void);
        void set_state(int state);

        ~OS3Engine();
    
    private:

        int programState; //running/stopping
        // static SimTK::Vec<3,double> latestForce;
        bool generateIDModel(void); //imports/configures inverse dynamics model
        OpenSim::Model IDModel;
        OpenSim::Manager* IDmanager;
        OpenSim::InverseDynamicsSolver* idSolver;
        //point force used to represent force exterted on end effector
        OpenSim::PointActuator endEffector;
        double IDtimestep;
        double prevSimTime;

        struct ID_Output {
            double timestamp;
            SimTK::Vector residualMobilityForces;
            bool valid;
        };
        
        //performs one iteration of inverse dynamics
        ID_Output inverseD(void);

        void step(void);

        struct ProblemParams {

            double timestamp;
            double forceMag;
            SimTK::Vec3 forceDirection;
        };

        std::ofstream logger;
};

}