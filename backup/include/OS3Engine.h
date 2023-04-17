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

        void start(void);

        void loop(void);

        OS3ROS commsClient;

        int get_state(void);
        void set_state(int state);

        void testPointActuator(void); //delete this //TODO

        ~OS3Engine();

        struct FD_Output {

            double timestamp; //should this be a time_t or not?

            SimTK::Vec3 wristPos;
            SimTK::Vec3 elbowPos;
            bool valid;

        };
    
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


        bool generateFDModel(void); //imports/configures forward dynamics model
        OpenSim::Model FDModel;
        OpenSim::Manager* FDmanager;
        SimTK::TimeStepper* FDstepper;
        std::unique_ptr<SimTK::TimeStepper> _FDstepper;
        std::unique_ptr<SimTK::Integrator> _FDintegr;
        double FDtimestep; //should be same as IDtimestep
        double prevSimTime;
        
        //actuators to apply input from ID in FD model
        OpenSim::TorqueActuator FDshoulderTorque1; // shoulder elevation
        OpenSim::TorqueActuator FDshoulderTorque2; // inward rotation (positive is adduction, neg abduction)
        OpenSim::TorqueActuator FDelbowTorque; // elbow flexion (pos is flexion, neg is extension)

        struct ID_Output {

            double timestamp;
            SimTK::Vector residualMobilityForces;
            bool valid;
        };

        
        //performs one iteration of inverse dynamics
        ID_Output inverseD(void);

        //performs one iteration of forward dynamics
        OS3Engine::FD_Output forwardD(OS3Engine::ID_Output input);
        SimTK::Vec3 FDshoulderPosG;
        //applies spring to joint when it approaches its limit
        double torqueSpring(double q, double u, double udot, double torque, const OpenSim::Coordinate* coord);
        double Kspring = 20; //K constant for joint springs (N.m/rad)
        double Bspring = 12; //Beta constant for joint springs (damping) (N.m.s/rad)
        double Bfree = 0.1; //Beta constant for damping within range of motion (N.m.s/rad)

        void step(void);

        struct ID_Input {

            double timestamp;
            double forceMag;
            SimTK::Vec3 forceDirection;
        };

        ID_Input forceVecToInput (OS3ROS::ForceInput forceInput);

        
        //COMBINED FD/ID - performs forward dynamics on a non-static model while still calculating joint torques
        OS3Engine::FD_Output forwardInverseD(void);

        std::ofstream logger;

        OS3ROS::OS3Data FDoutputToOS3Data(OS3Engine::FD_Output calculatedState);
        
};

}