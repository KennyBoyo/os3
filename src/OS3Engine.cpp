#include <ctime>
#include <chrono>
#include "OS3Engine.h"

using namespace SimTK;
using namespace OpenSim;
using namespace std;
using namespace OS3;

//order of joints in system state Q/Qdot/Udot vectors
#define SHOULDER_ELEV_QNUM 0
#define SHOULDER_ROT_QNUM 1
#define ELBOW_QNUM 2
#define LOGGING 

void OS3Engine::init(void) {

    programState = ENGINE_STANDBY;

    //initialise communication
    OS3ROS::init();
    generateIDModel();

    #ifdef LOGGING
        //setup logger
        time_t _tm =time(NULL );
        struct tm * curtime = localtime ( &_tm );
        std::cout<<"The current date/time is:"<<asctime(curtime);
        logger.open("Output/position.log", std::ios_base::app); //todo: customise name of file so we have a new log file each runtime
        logger << "OS3 DATA\n DATE: " << asctime(curtime) << "\nTIMESTAMP Q\n";
        logger << "cpus available: " << std::thread::hardware_concurrency() <<
         "\nTIMEIN FINmag FINdir RESt STEPPED TIMEOUT Q WRISTPOS ELBOWPOS\n" ;
    #endif

    auto startStepClock = std::chrono::steady_clock::now();
}

int OS3Engine::get_state(void) {
    return programState;
}

void OS3Engine::set_state(int state) {
    programState = state;
}


void OS3Engine::loop(void) {
    /**
     * Loops through the Inverse dynamics calculation for each problem received
    */
    while(1) {
        step();
    }
}

bool OS3Engine::generateIDModel(void) {
    /**
     * Initialise an OpenSim model to be used for inverse dynamics
    */

    // Define the initial and final simulation times
    double initialTime = 0.0;
    double finalTime = 30.00 / 30;
    const double timestep = 1e-3;
    IDtimestep = timestep; //this should be received from control input
    prevSimTime = 0; //can be overriden by control

    //import model
    IDModel =  OpenSim::Model("Models/arm26-mod.osim");
    //setup everything
    Vec3 grav = IDModel.get_gravity()*0; //end effector force is the net force (so includes gravity)
    IDModel.set_gravity(grav); //redundant unless we change grav above
    std::cout << IDModel.getBodySet() << std::endl;
    std::cout << IDModel.getJointSet() << std::endl;
    std::cout << IDModel.getMarkerSet() << std::endl;

    //get joints
    const OpenSim::JointSet& IDjointset = IDModel.get_JointSet();
    const OpenSim::Joint& IDshoulderJoint = IDjointset.get("r_shoulder");

    const OpenSim::Joint& IDelbowJoint = IDjointset.get("r_elbow");
    const OpenSim::Coordinate& IDelbowflex = IDelbowJoint.getCoordinate();

    //get bods
    const OpenSim::BodySet& IDbodyset = IDModel.get_BodySet();
    const OpenSim::Body& humerusbod = IDbodyset.get("r_humerus");
    const OpenSim::Body& radiusbod = IDbodyset.get("r_ulna_radius_hand");
    IDbodyset.print("bodyset.bods");

    //get muscles and disable
    const OpenSim::Set<OpenSim::Muscle>& muscleSet =  IDModel.getMuscles();
    for (int i=0; i < muscleSet.getSize(); i++) {
        muscleSet.get(i).set_appliesForce(false);
    }

    //get wrist point
    SimTK::Vec3 wristPointLoc = IDModel.getMarkerSet().get("r_radius_styloid").get_location();

    /*SimTK::State& si = */IDModel.initSystem();

    endEffector.set_body(radiusbod.getName());
    endEffector.setName("end_effector");
    endEffector.set_point(wristPointLoc);
    endEffector.set_point_is_global(false); //point coordinates are relative to radius
    endEffector.set_force_is_global(true); //force coordinates will be in ground frame
    endEffector.set_direction(SimTK::Vec3(1,0,0));   //x is front, y is up, 
    double optimalEndForce = 100; //Newtons (should be maximum force)
    endEffector.setOptimalForce(optimalEndForce);
    

    IDModel.addForce(&endEffector);
    IDModel.finalizeConnections();
    IDModel.setUseVisualizer(true);
    SimTK::State& si = IDModel.initSystem();

    Vector endEffectorControls(1);
    endEffectorControls(0) = 1*0;// 0 for realtime (controls will be added back later)
    
    
    //add control values
    Vector modelControls = IDModel.getDefaultControls();
    endEffector.addInControls(endEffectorControls,modelControls);
    IDModel.setDefaultControls(modelControls);

    //print initial accelerations to test
    IDModel.computeStateVariableDerivatives(si);
    Vector udotActuatorsCombination = si.getUDot();
    
    /////////////////////////////////////////////
    // DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
    /////////////////////////////////////////////
        
    IDmanager = new OpenSim::Manager(IDModel);
    IDmanager->setIntegratorAccuracy(timestep*1e-3);

    for (int i = 0; i < IDshoulderJoint.numCoordinates(); i++) { //locks all coordinates of the joint
        IDshoulderJoint.get_coordinates(i).setLocked(si,true);
    }

    IDModel.print("OS3Model.osim");
    IDModel.printDetailedInfo(si, std::cout);
    
    si.setTime(initialTime);
    IDmanager->initialize(si);
    SimTK::Integrator* integrator_ = &IDmanager->getIntegrator();
    std::cout << "using the " << integrator_->getMethodName() << " integration method\n"; //TODO compare performance of different integrators
    
    SimTK::State state_ =  integrator_->getAdvancedState();

    IDModel.getVisualizer().show(state_);
    IDModel.getMultibodySystem().realize(state_, Stage::Dynamics);
    
    SimTK::Vector residualMobilityForces;
    double simTime = 0;

    SimTK::Vec3 reverseDirection = endEffector.get_direction() * -1;
    bool reversed = false;

    OpenSim::InverseDynamicsSolver* solver = new OpenSim::InverseDynamicsSolver(IDModel);
    idSolver = solver;
    idSolver->setName("id_solver");

    return true;
}

void OS3Engine::step(void) {
    /**
     * Solves a single inverse dynamics problem
    */
    auto startStepClock = std::chrono::steady_clock::now();

    OS3ROS::ProblemOutput IDout = inverseD();

    auto finishStepClock = std::chrono::steady_clock::now();
    std::chrono::duration<double> stepDuration = std::chrono::duration_cast<std::chrono::duration<double>>(finishStepClock - startStepClock);
}


OS3ROS::ProblemOutput OS3Engine::inverseD(void) {
    /**
     * Inverse Dynamics function using OpenSim. Receives the latest joint angles and end effector wrench from OS3ROS, 
     * and processes it to receive joint torques for each joint.
    */
    #undef LOGGING
    
    OS3ROS::ProblemInput input(OS3ROS::get_latest_problem()); //need to make this threadsafe eventually
    OS3ROS::ProblemOutput output;

    if (input.timestamp <= prevSimTime) {
        return output;
    }
    
    #ifdef LOGGING
        logger << "  " << input.timestamp << "  " << input.forceMag << " " << input.forceDirection << " ";
    #endif

    SimTK::Integrator* integrator_ = &IDmanager->getIntegrator();

    SimTK::State state_ = integrator_->getAdvancedState();

    const OpenSim::JointSet& IDjointset = IDModel.get_JointSet();
    const OpenSim::Joint& IDshoulderJoint = IDjointset.get("r_shoulder");
    

    const OpenSim::Joint& IDelbowJoint = IDjointset.get("r_elbow");

    IDelbowJoint.getCoordinate().setLocked(state_, false);
    IDelbowJoint.getCoordinate().getLocked(state_);
    #ifdef LOGGING
        std::cout << "Before: " << IDelbowJoint.getCoordinate().getValue(state_) << std::endl;
    #endif
    
    // r_elbow_flex
    const OpenSim::Coordinate& IDElbowFlex = IDelbowJoint.getCoordinate();
    // r_shoulder_elev
    const OpenSim::Coordinate& IDShoulderFlex = IDjointset.get("r_shoulder1").get_coordinates(0);
    // r_shoulder_rot
    const OpenSim::Coordinate& IDShoulderRot = IDjointset.get("r_shoulder2").get_coordinates(0);
    // r_shoulder_add
    const OpenSim::Coordinate& IDShoulderAdd = IDshoulderJoint.get_coordinates(0);



    IDShoulderFlex.setLocked(state_, false);
    IDShoulderRot.setLocked(state_, false);
    IDShoulderAdd.setLocked(state_, false);
    IDElbowFlex.setLocked(state_, false);

    
    // IDElbowFlex.setValue(state_, IDElbowFlex.getValue(state_) + convertDegreesToRadians(5));

    IDShoulderFlex.setValue(state_, input.angles[0]);
    IDShoulderRot.setValue(state_, input.angles[1]);
    IDShoulderAdd.setValue(state_, input.angles[2]);
    IDElbowFlex.setValue(state_, input.angles[3]);
    
    #ifdef LOGGING
        std::cout << "After: " << IDelbowJoint.getCoordinate().getValue(state_) << std::endl;
    #endif

    IDShoulderFlex.setLocked(state_, true);
    IDShoulderRot.setLocked(state_, true);
    IDShoulderAdd.setLocked(state_, true);
    IDElbowFlex.setLocked(state_, true);
    
    IDmanager = new OpenSim::Manager(IDModel);
    IDmanager->initialize(state_);

    integrator_ = &IDmanager->getIntegrator();

    state_ = integrator_->getAdvancedState();

    IDModel.getMultibodySystem().realize(state_, Stage::Dynamics); //just for adding in controls (will do this again)
    IDModel.getVisualizer().show(state_);

    Vector controls(1);
    controls(0) = input.forceMag / endEffector.get_optimal_force();

    SimTK::Vec3 direction = input.forceDirection;
    endEffector.set_direction(direction);

    Vector modelControls = IDModel.getDefaultControls();
    endEffector.addInControls(controls, modelControls);

    IDModel.setControls(state_,modelControls);



    IDModel.getMultibodySystem().realize(state_, Stage::Dynamics);

    const Vector& appliedMobilityForces = 
                IDModel.getMultibodySystem().getMobilityForces(state_, Stage::Dynamics);

    const SimTK::Vector Udot = Test::randVector(state_.getNU())*0; //zero for isometric

    SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces(IDModel.getMultibodySystem().getRigidBodyForces(state_, SimTK::Stage::Dynamics));

    SimTK::Vector residualMobilityForces = idSolver->solve(state_,Udot,appliedMobilityForces,appliedBodyForces);
    
    for (auto i = 0; i < residualMobilityForces.size(); i++) {
        output.torques.push_back(residualMobilityForces[i]);
    }
    for (auto i = 0; i < input.names.size(); i++) {
        output.names.push_back(input.names[i].c_str());
    }
    for (auto i = 0; i < input.angles.size(); i++) {
        output.angles.push_back(input.angles[i]);
    }
    for (auto i = 0; i < input.velocities.size(); i++) {
        output.velocities.push_back(input.velocities[i]);
    }
    
    #ifdef LOGGING
        std::cout << "residualmob: " << output.residualMobilityForces << std::endl << "magnitude: " << input.forceMag << std::endl  << "direction: " << input.forceDirection << std::endl;
    #endif

    prevSimTime = input.timestamp;
    
    OS3ROS::set_latest_solution(output);
    return output;

}

OS3Engine::~OS3Engine() {
    /**
     * Destructor for the OS3Engine class instance
     * 
    */

    delete[] idSolver;
    delete[] IDmanager;

}