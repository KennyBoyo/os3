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

    std::cout << "Finished Initialising OS3ROS" << std::endl;
    generateIDModel(); //import/configure models


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

//NOT THREADSAFE (yet)
void OS3Engine::set_state(int state) {
    programState = state;
}


void OS3Engine::loop(void) {
    while(1) {
        step();
    }
}

/*******PRIVATE***************/

bool OS3Engine::generateIDModel(void) {

    // Define the initial and final simulation times //SHOULD BECOME OBSELETE IN REALTIME
    double initialTime = 0.0;
    double finalTime = 30.00 / 30;
    const double timestep = 1e-3;// * 100; //TODO fix this
    IDtimestep = timestep; //this should be received from control input
    prevSimTime = 0; //can be overriden by control

    //import model
    IDModel =  OpenSim::Model("Models/arm26-mod.osim"); std::cout << "loaded model from arm26-mod" << std::endl;
    std::cout << IDModel.getName() << "<----------------model name\n\n";
    //setup everything
    Vec3 grav = IDModel.get_gravity()*0; //end effector force is the net force (so includes gravity)
    IDModel.set_gravity(grav); //redundant unless we change grav above
    std::cout << "MARKERS!" << std::endl;
    std::cout << IDModel.getBodySet() << std::endl;
    std::cout << IDModel.getJointSet() << std::endl;
    std::cout << IDModel.getMarkerSet() << std::endl;

    //get joints
    const OpenSim::JointSet& IDjointset = IDModel.get_JointSet();
    const OpenSim::Joint& IDshoulderJoint = IDjointset.get("r_shoulder");

    const OpenSim::Joint& IDelbowJoint = IDjointset.get("r_elbow");
    const OpenSim::Coordinate& IDelbowflex = IDelbowJoint.getCoordinate();

    // elbowFlexRange = {IDshoulderelevcoord.getRangeMin(), IDshoulderelevcoord.getRangeMax()};

    std::cout << IDelbowflex.getName() << "<----- elbowflex name \n";



    //get bods
    const OpenSim::BodySet& IDbodyset = IDModel.get_BodySet();
    const OpenSim::Body& humerusbod = IDbodyset.get("r_humerus");
    const OpenSim::Body& radiusbod = IDbodyset.get("r_ulna_radius_hand");
    std::cout << humerusbod.getName() << "<---name of humerusbod\n";
    std::cout << radiusbod.getName() << "<---name of radiusbod\n";
    IDbodyset.print("bodyset.bods");

    //get muscles and disable
    const OpenSim::Set<OpenSim::Muscle>& muscleSet =  IDModel.getMuscles();
    for (int i=0; i < muscleSet.getSize(); i++) {
        muscleSet.get(i).set_appliesForce(false);
    }


    //get wrist point
    SimTK::Vec3 wristPointLoc = IDModel.getMarkerSet().get("r_radius_styloid").get_location();

    //init system (for end effector's sake)
    /*SimTK::State& si = */IDModel.initSystem();

    endEffector.set_body(radiusbod.getName());
    endEffector.setName("end_effector");
    std::cout << endEffector.get_body() << "  <-- name of body for end effector \n";
    endEffector.set_point(wristPointLoc);
    endEffector.set_point_is_global(false); //point coordinates are relative to radius
    endEffector.set_force_is_global(true); //force coordinates will be in ground frame
    endEffector.set_direction(SimTK::Vec3(1,0,0));   //x is front, y is up, 
    std::cout << endEffector.get_direction() << "<-- eE dir\n";//endEffector.set_direction(SimTK::Vec3(1,1,1));
    double optimalEndForce = 100; //Newtons (should be maximum force)
    endEffector.setOptimalForce(optimalEndForce);
    
    std::cout << endEffector.getOptimalForce() << "  <-- optimal force from endeffector\n";

    IDModel.addForce(&endEffector);
    IDModel.finalizeConnections();
    IDModel.setUseVisualizer(true);
    SimTK::State& si = IDModel.initSystem(); std::cout << "hello there\n";
    std::cout << endEffector.get_appliesForce() << "<-- that end effector is applying force---------------------\n";

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
    // IDmanager->setIntegratorMethod(OpenSim::Manager::IntegratorMethod::RungeKuttaFeldberg);
    IDmanager->setIntegratorAccuracy(timestep*1e-3); //TODO: Change this to match whatever I choose for FD (not too important as we don't integrate ID)


    // // Get the pelvis body by its name
    // OpenSim::Body& pelvis = IDModel.updBodySet().get("r_ulna_radius_hand");

    // // Set the position of the pelvis origin to (0.1, 0.2, 0.3)
    // SimTK::Vec3 position(0.1, 0.2, 0.3);
    // pelvis.set(state, position);

    // Marker& marker = IDModel.updMarkerSet().get("r_radius_styloid");
    // SimTK::Vec3 location(0.1, 0.2, 0.3);
    // marker.setLocation(location);


    // IDelbowJoint.getCoordinate().setValue(state, position);
    //lock joints - Only if we're performing ID first, not necessary for the direct FD method
    // IDelbowJoint.getCoordinate().setValue(si,convertDegreesToRadians(90));
    std::cout << IDshoulderJoint.get_coordinates(0).getName() << "<--name of shoulder coord 0. Let's set it to pi radians\n";
    
    #define IDFD
    #ifdef IDFD
    IDelbowJoint.getCoordinate().setLocked(si, true);
    
    std::cout << IDshoulderJoint.numCoordinates() << "<-- number of state var\n";
    for (int i = 0; i < IDshoulderJoint.numCoordinates(); i++) { //locks all coordinates of the joint
        IDshoulderJoint.get_coordinates(i).setLocked(si,true);
    }
    
    // IDshoulderJoint.getCoordinate().setLocked(si,true);
    #else  //NOT IDFD -- rather than doing first ID then FD, we do FD directly (not properly tested)
    IDelbowJoint.getCoordinate().setClamped(si,true);
    IDshoulderJoint.getCoordinate().setClamped(si,true);
    std::cout << "shoulder coord free to satisfy constraints" << IDshoulderJoint.getCoordinate().get_is_free_to_satisfy_constraints() << std::endl;
    std::cout << "shoulder coord prescribed function" << IDshoulderJoint.getCoordinate().get_prescribed() << std::endl;
    std::cout << "shouldercoord clamped in si " << IDshoulderJoint.getCoordinate().getClamped(si) << std::endl;
    #endif

    IDModel.print("OS3Model.osim");
    // Print out details of the model
    IDModel.printDetailedInfo(si, std::cout);

    #define TEST_ID
    #ifdef TEST_ID 
    #define REALTIME_TEST
    #ifdef REALTIME_TEST
    //REALTIME STUFF BELOW
    std::cout << "--------------------------ENTER REALTIME TEST MODE--------------------\n";
    si.setTime(initialTime);
    IDmanager->initialize(si);
    //get engine properties (just for our while loop here)
    SimTK::Integrator* integrator_ = &IDmanager->getIntegrator();
    std::cout << "using the " << integrator_->getMethodName() << " integration method\n"; //TODO compare performance of different integrators
    
    SimTK::State state_ =  integrator_->getAdvancedState();

    IDModel.getVisualizer().show(state_);
    IDModel.getMultibodySystem().realize(state_, Stage::Dynamics);
    std::cout << "----------------IMPORTANT-------------------\n";
    std::cout << "how many nq are there?\n";
    std::cout << "and what coordinates do they represent?\n";
    std::cout << "we shall find out\n";
    std::cout << "there are ....  " << state_.getNQ() << " NQ in state_\n";
    std::cout << "the current Q are " << state_.getQ() << " at initialisation\n";
    std::cout << "but there are more subsystems\n";
    std::cout << state_.getNumSubsystems() << " subsystems actually\n";
    std::cout << "so let's try something different\n";
    for (SimTK::SubsystemIndex subsysI(0); subsysI < state_.getNumSubsystems(); subsysI++) {
        std::cout << state_.getNQ(subsysI) << " NQ in subsys " << subsysI << "\n";
           
    }
    std::cout << "what does this mean?\n";
    

    SimTK::Vector residualMobilityForces;
    double simTime = 0;

    SimTK::Vec3 reverseDirection = endEffector.get_direction() * -1;
    bool reversed = false;

    OpenSim::InverseDynamicsSolver* solver = new OpenSim::InverseDynamicsSolver(IDModel);
    idSolver = solver;
    idSolver->setName("id_solver");

   
    
    #else //NOT REALTIME
    // Integrate from initial time to final time
    std::cout << "commencing non-realtime simulation\n";
    si.setTime(initialTime);
    std::cout << "time is set, let's initialise the manager\n";
    IDmanager->initialize(si);

    std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
    IDmanager->integrate(finalTime);
     
    auto forcesTable = forceRep->getForcesTable();
    forceRep->getForceStorage().print("forcestorage.mot");

    // std::string forceFilename
    OpenSim::STOFileAdapter::write(forcesTable, "forces.sto");


    std::cout << IDmanager->getIntegrator().getAdvancedState().getQ() << "<--Q??\n";
    while(1) {
        IDModel.getVisualizer().show(IDmanager->getIntegrator().getAdvancedState());
    }
    #endif


    #endif //TEST_ID
    


    return true;
}

void OS3Engine::step(void) {
    auto startStepClock = std::chrono::steady_clock::now();

    //step id first using OS3Engine::inverseD
    OS3ROS::ProblemOutput IDout = inverseD();
    // OS3ROS::set_latest_solution(IDout);
        
    // prevSimTime = IDout.timestamp; //shouldn't be necessary with control input //TODO this
    


    auto finishStepClock = std::chrono::steady_clock::now();
    std::chrono::duration<double> stepDuration = std::chrono::duration_cast<std::chrono::duration<double>>(finishStepClock - startStepClock);
}


OS3ROS::ProblemOutput OS3Engine::inverseD(void) {
    #undef LOGGING
    
    OS3ROS::ProblemInput input(OS3ROS::get_latest_problem()); //need to make this threadsafe eventually
    OS3ROS::ProblemOutput output;

    // OS3::ID_Output output;


    // std::cout << "Here: " << input.forceDirection << std::endl;
    // std::cout << "Time: " << input.timestamp << std::endl;

    // for (auto i = 0; i < input.names.size(); i++) {
    //     std::cout << input.names[i].c_str() << std::endl;
    // }
    // for (auto i = 0; i < input.angles.size(); i++) {
    //     std::cout << "angles" << input.angles[i] << std::endl;
    // }
    // for (auto i = 0; i < input.velocities.size(); i++) {
    //     std::cout << "velocities" << input.velocities[i] << std::endl;
    // }
    // for (auto i = 0; i < input.wrench.size(); i++) {
    //     std::cout << "wrench" << input.wrench[i] << std::endl;
    // }

    // output.timestamp = input.timestamp;

    if (input.timestamp <= prevSimTime) {
        // std::cout << "time stayed at " << output.timestamp << std::endl;
        //we haven't progressed
        return output; //output set to invalid
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

    // TESTING Setting joint angles at each time step
    // IDelbowJoint.getCoordinate().setValue(state_, IDelbowJoint.getCoordinate().getValue(state_) + convertDegreesToRadians(5));
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
    // IDelbowJoint.getCoordinate().setLocked(state_, true);
    
    IDmanager = new OpenSim::Manager(IDModel);
    IDmanager->initialize(state_);

    // Before: 25.5691
    // After: 25.6563
    // ~[2.39636 0.201409 0.889342]
    // or 0.0331805
    
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

    // for (auto i = 0; i < output.names.size(); i++) {
    //     std::cout << "name" << output.names[i] << std::endl;
    // }
    // for (auto i = 0; i < output.angles.size(); i++) {
    //     std::cout << "angles" << output.angles[i] << std::endl;
    // }
    // for (auto i = 0; i < output.velocities.size(); i++) {
    //     std::cout << "velocities" << output.velocities[i] << std::endl;
    // }
    // for (auto i = 0; i < output.torques.size(); i++) {
    //     std::cout << "wrench" << output.torques[i] << std::endl;
    // }

    // std::cout << output.names[0] << std::endl;
    // output.names = input.names;
    // output.angles = input.angles;
    // output.velocities = input.velocities;
    // std::cout << "residualmob: " << residualMobilityForces << std::endl;

    #ifdef LOGGING
        std::cout << "residualmob: " << output.residualMobilityForces << std::endl << "magnitude: " << input.forceMag << std::endl  << "direction: " << input.forceDirection << std::endl;
    #endif

    prevSimTime = input.timestamp;
    
    OS3ROS::set_latest_solution(output);
    return output;

}

OS3Engine::~OS3Engine() {

    delete[] idSolver;
    delete[] IDmanager;

}

//PRIVATE HELPER FUNCTIONS

/*converts clock ticks into seconds */
double _clock_secs(clock_t ctim) {
    double tim = (double) ctim;
    double cps =  CLOCKS_PER_SEC;
    double res = tim/cps;
    

    return res;
    
}