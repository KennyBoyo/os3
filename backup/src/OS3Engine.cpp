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
    commsClient.init();

    generateIDModel(); //import/configure models

    generateFDModel(); //


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

    while(1) { //TODO: check control logic here
        
        
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


    // if (EXTRA_ROT) {
        
    //     const OpenSim::Coordinate& IDshoulderelevcoord = IDshoulderJoint.get_coordinates(0);
    //     const OpenSim::Coordinate& IDshoulderRot2Coord = IDshoulderJoint.get_coordinates(1);
        
    // } else {
    //     const OpenSim::Coordinate& IDshoulderelevcoord = IDshoulderJoint.getCoordinate();
    //     // shoulderElevRange = {IDshoulderelevcoord.getRangeMin(), IDshoulderelevcoord.getRangeMax()};
    // }
    

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

    //OS3Engine::endEffector
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
    IDelbowJoint.getCoordinate().setValue(si,convertDegreesToRadians(90));
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

bool OS3Engine::generateFDModel(void) {

    std::cout << "initialising FD model\n";
    //import model
    FDModel =  OpenSim::Model("Models/arm26-mod.osim");
    //setup everything
    try {
        FDtimestep = IDtimestep;
    } catch (...) {
        std::cerr << "ID model not initialised\n";
        std::cerr << "Exiting\n";
        return false;
    }
    double initialTime = 0;

    //set gravity (for debugging)
    Vec3 grav = FDModel.get_gravity()*0;
    FDModel.set_gravity(grav); //redundant unless we change grav above

    //get bodies
    const OpenSim::BodySet& FDbodyset = FDModel.get_BodySet();
    const OpenSim::Body& FDbasebod = FDbodyset.get("base");
    const OpenSim::Body& FDhumerusbod = FDbodyset.get("r_humerus");
    const OpenSim::Body& FDradiusbod = FDbodyset.get("r_ulna_radius_hand"); //not used here

    //get joints

    const OpenSim::JointSet& FDjointset = FDModel.get_JointSet();

    FDModel.initSystem(); //just so we can get the right component
    const OpenSim::CustomJoint& FDshoulderCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_shoulder");
    const OpenSim::CustomJoint& FDelbowCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_elbow");
    // const OpenSim::Joint& FDelbowJoint = FDjointset.get("r_elbow");
    
    //get muscles and disable
    const OpenSim::Set<OpenSim::Muscle>& muscleSet =  FDModel.getMuscles();
    for (int i=0; i < muscleSet.getSize(); i++) {
        muscleSet.get(i).set_appliesForce(false);
    }

    

    //add torque actuators (to be controlled by ID input)
    SimTK::Vec3 FDshoulderAxis1 = FDshoulderCustomJoint.getSpatialTransform().get_rotation1().getAxis();
    SimTK::Vec3 FDshoulderAxis2 = FDshoulderCustomJoint.getSpatialTransform().get_rotation2().getAxis();
    std::cout << FDshoulderAxis1 << "<-- this is the shoulder axis rot1\n";
    std::cout << FDshoulderAxis2 << "<-- this is the shoulder axis rot2\n";
    FDshoulderTorque1.setName("shoulder_elev_T");
    FDshoulderTorque1.set_bodyA("base");
    FDshoulderTorque1.set_bodyB("r_humerus");
    FDshoulderTorque1.set_torque_is_global(false);
    FDshoulderTorque1.setAxis(FDshoulderAxis1);
    FDshoulderTorque1.setOptimalForce(OPTIMAL_TORQUE) ; // N.m (maximum torque supposedly)

    FDshoulderTorque2.setName("shoulder_rot2_T");
    FDshoulderTorque2.set_bodyA("base");
    FDshoulderTorque2.set_bodyB("r_humerus");
    FDshoulderTorque2.set_torque_is_global(false);
    FDshoulderTorque2.setAxis(FDshoulderAxis2);
    FDshoulderTorque2.setOptimalForce(OPTIMAL_TORQUE) ; // N.m (maximum torque supposedly)


    SimTK::Vec3 FDelbowFlexAxis = FDelbowCustomJoint.getSpatialTransform().get_rotation1().getAxis();
    FDelbowTorque.set_bodyA("r_humerus");
    FDelbowTorque.set_bodyB("r_ulna_radius_hand");
    FDelbowTorque.set_torque_is_global(false);
    FDelbowTorque.setAxis(FDelbowFlexAxis);
    FDelbowTorque.setOptimalForce(OPTIMAL_TORQUE);

    FDModel.addForce(&FDshoulderTorque1);
    FDModel.addForce(&FDshoulderTorque2);
    FDModel.addForce(&FDelbowTorque);
    FDModel.finalizeConnections();
    
    //set default shoulder torque to zero
    SimTK::Vector shoulderControls1(1);
    shoulderControls1(0) = 0;
    SimTK::Vector shoulderControls2(1);
    shoulderControls2(0) = 0;

    SimTK::Vector modelControls = FDModel.getDefaultControls();
    FDshoulderTorque1.addInControls(shoulderControls1,modelControls);
    FDshoulderTorque2.addInControls(shoulderControls2,modelControls);

    //set default elbow torque
    SimTK::Vector elbowControls(1);
    elbowControls(0) = 0;

    modelControls = FDModel.getDefaultControls();
    FDelbowTorque.addInControls(elbowControls, modelControls);

    //initialise model and get state
    FDModel.setUseVisualizer(true); std::cout << "using FD visualiser\n";
    SimTK::State& si = FDModel.initSystem();

    
    //create manager and save it to class variable
    FDmanager = new OpenSim::Manager(FDModel);
    FDmanager->setIntegratorMethod(OpenSim::Manager::IntegratorMethod::RungeKutta2);//TODO here
    FDmanager->setIntegratorAccuracy(FDtimestep*1e-3); //TODO: play around with this. If I reduce it, it might be faster

    

    FDelbowCustomJoint.getCoordinate().setValue(si,convertDegreesToRadians(90));

    //clamp joints to within limits
    FDelbowCustomJoint.getCoordinate().setClamped(si,true);
    for (int i=0; i<FDshoulderCustomJoint.numCoordinates(); i++) {
        
        FDshoulderCustomJoint.get_coordinates(i).setClamped(si,true);

    }
    
    
    
    FDModel.print("FDOS3Model.osim");
    FDModel.printDetailedInfo(si, std::cout);

   
    si.setTime(initialTime);

    FDmanager->initialize(si);
    
    _FDintegr.reset(new SimTK::SemiExplicitEulerIntegrator(FDModel.getMultibodySystem(), FDtimestep));
    _FDintegr->setAccuracy(1e-3);
    _FDstepper.reset(new SimTK::TimeStepper(FDModel.getMultibodySystem(),*_FDintegr));
    _FDstepper->initialize(si);

    
    FDModel.getVisualizer().show(si);

    OpenSim::Logger::setLevel(OpenSim::Logger::Level::Off); //Not Useful
    
    std::cout << "shoulder loc: " << FDshoulderCustomJoint.getParentFrame().getPositionInGround(si) << std::endl;
    FDshoulderPosG = FDshoulderCustomJoint.getParentFrame().getPositionInGround(si);
    std::cout << "FDmodel generated\n";

    return true;
}




void OS3Engine::step(void) {

    clock_t initTime = std::clock();
    auto startStepClock = std::chrono::steady_clock::now();

    //step id first using OS3Engine::inverseD
    OS3Engine::ID_Output IDout = inverseD();
    
    if (IDout.valid) {

        #ifdef LOGGING          //order is time, mobilityforces, torqueactuator, wristpos, elbowpos
            // logger << " " << IDout.timestamp <<  "    " << IDout.residualMobilityForces << " ";
            logger <<  "    " << IDout.residualMobilityForces << " ";
        #endif

        
        //get output torques in a form that is usable to the FD engine

        //step fd using OS3Engine::forwardD
        OS3Engine::FD_Output FDout = forwardD(IDout);
        
        //publish using comms::publisher
        commsClient.setPositionToPublish(FDoutputToOS3Data(FDout));
        

        #ifdef LOGGING
            // logger << FDout.timestamp << "   " ;//<< FDout.wristPos << std::endl;
            logger << " " << FDout.wristPos << " " << FDout.elbowPos << std::endl;
        #endif
        
        prevSimTime = IDout.timestamp; //shouldn't be necessary with control input //TODO this

        auto finishStepClock = std::chrono::steady_clock::now();
        std::chrono::duration<double> stepDuration = std::chrono::duration_cast<std::chrono::duration<double>>(finishStepClock - startStepClock);
        // std::cerr << " in " << ((double)(std::clock() - initTime) / CLOCKS_PER_SEC) << "clocl secs ";
        std::cout << " or " << stepDuration.count() << std::endl;

    }
    // double stepTime = ((double) (std::clock() - initTime))/CLOCKS_PER_SEC;
    // std::cout << "STEPPED IN " << stepTime << " seconds! that's " << (std::clock() - initTime) << " clocks \n";
}


OS3Engine::ID_Output OS3Engine::inverseD(void) {

    
    OS3Engine::ID_Input input(forceVecToInput(commsClient.get_latest_force())); //need to make this threadsafe eventually
    OS3Engine::ID_Output output;


    // IDModel.setUseVisualizer(false);
    // SimTK::State& si = IDModel.initSystem();
    // //get joints
    // const OpenSim::JointSet& IDjointset = IDModel.get_JointSet();
    // const OpenSim::Joint& IDshoulderJoint = IDjointset.get("r_shoulder");
    

    // const OpenSim::Joint& IDelbowJoint = IDjointset.get("r_elbow");
    // std::cout << "Before: " << IDelbowJoint.getCoordinate().getValue(si) << std::endl;
    // IDelbowJoint.getCoordinate().setValue(si, IDelbowJoint.getCoordinate().getValue(si) + convertDegreesToRadians(5));
    // std::cout << "After: " << IDelbowJoint.getCoordinate().getValue(si) << std::endl;

    // IDModel.getMultibodySystem().realize(si, Stage::Position);

    // IDModel.setUseVisualizer(true);
    // si = IDModel.initSystem();
    // IDModel.getVisualizer().show(si);


    output.valid = false; //change to true later if data
    output.timestamp = input.timestamp;

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
    std::cout << "Before: " << IDelbowJoint.getCoordinate().getValue(state_) << std::endl;
    IDelbowJoint.getCoordinate().setValue(state_, IDelbowJoint.getCoordinate().getValue(state_) + convertDegreesToRadians(5));
    std::cout << "After: " << IDelbowJoint.getCoordinate().getValue(state_) << std::endl;

    IDelbowJoint.getCoordinate().setLocked(state_, true);
    
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

    output.residualMobilityForces = idSolver->solve(state_,Udot,appliedMobilityForces,appliedBodyForces);

    // std::cout << "residualmob: " << output.residualMobilityForces << " from forcevec: " <<  latestForce << "\n magnitude: " << input.forceMag << " and direction: " << input.forceDirection << std::endl;
    output.valid = true;
    cout << output.residualMobilityForces << endl;


    
    return output;

}


OS3Engine::ID_Input OS3Engine::forceVecToInput(OS3ROS::ForceInput forceInput) {

    OS3Engine::ID_Input input;
    input.timestamp = forceInput.time;

    input.forceMag = sqrt(~forceInput.force * forceInput.force);
    if (input.forceMag == 0) {
        input.forceDirection = {1,1,1}; //no force anyway
    } else {
        input.forceDirection = forceInput.force / input.forceMag;
    }

    return input;

}



OS3Engine::FD_Output OS3Engine::forwardD(OS3Engine::ID_Output input) {

    auto tim0 = std::chrono::steady_clock::now();

    clock_t FDstartTime = std::clock();

    OS3Engine::FD_Output output;
    output.valid = false; //we'll change this to true once we have data
    output.timestamp = input.timestamp;

    if (input.valid == false) {
        std::cout << "invalid ID output provided as input to FD\n";
        return output;
    }

    
 

    // std::cout << "INPUT PROPERTIES\n resmob: " << input.residualMobilityForces << "timestamp: " << input.timestamp << "valid?: " << input.valid << std::endl;

    // SimTK::Integrator* integrator_ = &FDmanager->getIntegrator(); //when using manager (no stepper)
    SimTK::Integrator* integrator_ = _FDintegr.get(); //when using stepper
    

    const SimTK::State& state_ = integrator_->getAdvancedState(); //this needs to be a pointer, since we plan on doing fun things to it
    
    const OpenSim::CustomJoint& FDshoulderCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_shoulder");
    const OpenSim::CustomJoint& FDelbowCustomJoint =  FDModel.getComponent<OpenSim::CustomJoint>("/jointset/r_elbow");
    
    
    // double shoulderMaxElev =  FDshoulderCustomJoint.getCoordinate().getRangeMax();
    // double shoulderMinElev =  FDshoulderCustomJoint.getCoordinate().getRangeMin();
    double elbowMaxFlex = FDelbowCustomJoint.getCoordinate().getRangeMax();
    double elbowMinFlex = FDelbowCustomJoint.getCoordinate().getRangeMin();


    
    FDModel.getMultibodySystem().realize(state_, Stage::Acceleration);

    Vector shoulderControls1_(1);
    Vector shoulderControls2_(1);
    Vector elbowControls_(1);
    
    
    // shoulderControls1_(0) = 0;//input.residualMobilityForces(SHOULDER_ELEV_QNUM) / FDshoulderTorque1.getOptimalForce();
    // shoulderControls2_(0) = 0;//input.residualMobilityForces(SHOULDER_ROT_QNUM) / FDshoulderTorque2.getOptimalForce();
    // elbowControls_(0) = input.residualMobilityForces(ELBOW_QNUM) / FDelbowTorque.getOptimalForce();

    SimTK::Vector qVec_ = state_.getQ();
    SimTK::Vector uVec_ = state_.getU();
    SimTK::Vector uDVec_ = state_.getUDot();


    shoulderControls1_(0) = torqueSpring(qVec_(SHOULDER_ELEV_QNUM), uVec_(SHOULDER_ELEV_QNUM), uDVec_(SHOULDER_ELEV_QNUM),
            input.residualMobilityForces(SHOULDER_ELEV_QNUM), &FDshoulderCustomJoint.get_coordinates(0))
            / FDshoulderTorque1.getOptimalForce();
    shoulderControls2_(0) = torqueSpring(qVec_(SHOULDER_ROT_QNUM), uVec_(SHOULDER_ROT_QNUM), uDVec_(SHOULDER_ROT_QNUM),
            input.residualMobilityForces(SHOULDER_ROT_QNUM), &FDshoulderCustomJoint.get_coordinates(1))
            / FDshoulderTorque2.getOptimalForce();
    elbowControls_(0) = torqueSpring(qVec_(ELBOW_QNUM), uVec_(ELBOW_QNUM), uDVec_(ELBOW_QNUM),
            input.residualMobilityForces(ELBOW_QNUM), &FDelbowCustomJoint.getCoordinate() )
            / FDelbowTorque.getOptimalForce();
    

    

    Vector modelControls_ = FDModel.getDefaultControls();
    FDshoulderTorque1.addInControls(shoulderControls1_,modelControls_);
    FDshoulderTorque2.addInControls(shoulderControls2_,modelControls_);

    FDelbowTorque.addInControls(elbowControls_,modelControls_); //TODO: does this work or do I need to split in two steps?

    FDModel.setControls(state_,modelControls_);

    auto tim22 = std::chrono::steady_clock::now();
    


    FDModel.getMultibodySystem().realizeTopology();



    double step0 = state_.getTime();
    double intTime  = integrator_->getAdvancedTime() - step0;
    if (intTime != 0) {
        std::cout << "NOT SAME " << intTime << "\n ";
    }

    //step simulation
    clock_t steppingTime = std::clock();
    auto tim1 =  std::chrono::steady_clock::now();
    // SimTK::Integrator::SuccessfulStepStatus result = integrator_->stepBy(FDtimestep); //eventually will need stepTo() because we might miss timesteps
    int numreturns = 0;
    
    SimTK::Integrator::SuccessfulStepStatus result = integrator_->stepTo(input.timestamp);
    
    
    auto tim2 = std::chrono::steady_clock::now();
    
    SimTK::State newState_ = integrator_->getAdvancedState(); //TODO: should this work with the State& state_ now that I changed it to a pointer?
                                                            //TODO: actually, didn't I change it to a pointer??? I thought I called updAdvancedState().... not sure what's happening here

    double stepped1 = newState_.getTime() - step0;
    if (newState_.getTime() < input.timestamp) {
        std::cout << "integrator returned before complete timestep. Stepped: " << stepped1;
    } else if (newState_.getTime() > (input.timestamp + FDtimestep*10)  ) {
        std::cerr << "We stepped more than 10 timesteps more thatn we should have\n";
        // std::cerr << "stepped " << stepped1;
    }
    // std::cout << "stepped " << stepped1 << " ";
    #ifdef LOGGING
        logger << " " << stepped1 << " ";
        logger << "  " << newState_.getTime() << "  ";
        logger << "      " << newState_.getQ() << "  ";
    #endif
    FDModel.getVisualizer().show(newState_);

    //realize again so we can get state variables 
    FDModel.getMultibodySystem().realize(newState_, Stage::Position);
    

    //find positions in ground frame
        // SimTK::Vec3 humerusPos = FDModel.getBodySet().get("r_humerus").getPositionInGround(newState_); //humerus pos doesn't change (shoulder is fixed location)
    output.elbowPos = FDModel.getBodySet().get("r_ulna_radius_hand").getPositionInGround(newState_) - FDshoulderPosG; //position is sent as distance from shoulder
    output.wristPos = FDModel.getMarkerSet().get("r_radius_styloid").getLocationInGround(newState_) - FDshoulderPosG;
    if (output.elbowPos.size() > 0) {
        output.valid = true;
    }

    

    return output;

    



}


/* 
Enforces joint limits by adding a spring when the given coordinate approaches its limit
    Damped spring of the following form:
        T = K * x + B * u
@param: q,u  - state variables for the given coordinate
        torque - joint torque received from ID
        coord - the coordinate concerned
@returns: the new torque after spring applied
*/
double OS3Engine::torqueSpring(double q, double u, double udot, double torque, const OpenSim::Coordinate* coord) {
    
    double tol = (coord->getRangeMax() - coord->getRangeMin())*0.25; // the upper and lower __% of the range will have a spring
    // std::cout << coord->getName() << tol << std::endl; return torque;
    // std::cout << "T_in " << torque << " ";
    if ( q > (coord->getRangeMax() - tol) ) {
        // std::cout << "T_in " << torque << " ";
        double x = q - (coord->getRangeMax() - tol); //displacement from upper equilibrium point
        torque +=  Kspring * x + Bspring * u;
        // std::cout << "OVER: " << coord->getName() << " q " << q << " x " << x << " u " << u << " torque " << torque << "\n";
    } else if (q < (coord->getRangeMin() + tol) ) {
        // std::cout << "T_in " << torque << " ";
        double x = q - (coord->getRangeMin() + tol); //displacement from lower equilibrium point (will be negative)
        torque +=  Kspring * x + Bspring * u;
        // std::cout << "UNDER: " << coord->getName() << " q " << q << " x " << x << " u " << u << " torque " << torque << "\n";

    } else {
        // std::cout << "T_in " << torque << " ";
        //motion is within permitted range, we will just give it a small damping term to stop it from oscillating
        torque += Bfree * u;
        // std::cout << "GOOD: " << coord->getName() << " q " << q << " u " << u << " torque " << torque <<  "\n";
    }
    #define MAX_ABS_TORQUE 20
    if (torque > MAX_ABS_TORQUE) { //clamp torque
        torque = MAX_ABS_TORQUE;
    } else if (torque < -MAX_ABS_TORQUE) {
        torque = -MAX_ABS_TORQUE;
    }

    #ifdef LOGGING
        // logger << q << " " << u  << " " << torque ;//<< std::endl;
    #endif

    return torque;
}


OS3ROS::OS3Data OS3Engine::FDoutputToOS3Data(OS3Engine::FD_Output calculatedState) {


    OS3ROS::OS3Data outputToRos;
    if (calculatedState.valid == true) {
        outputToRos.timestamp = calculatedState.timestamp;
        // outputToRos.q = calculatedState.q;
        // outputToRos.u = calculatedState.u;
        // outputToRos.uDot = calculatedState.uDot;
        outputToRos.wristPos = calculatedState.wristPos;
        outputToRos.elbowPos = calculatedState.elbowPos;
        outputToRos.valid = true;
    } else {
        std::cout << "INVALID OUTPUT FROM FD";
        outputToRos.valid = false;
    }
    return outputToRos;
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