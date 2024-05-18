#include "KEDemoMachine.h"

using namespace std;

bool endCalib(StateMachine & sm) {
    return (sm.state<KECalibState>("CalibState2"))->isCalibDone();
}

bool goToNextState(StateMachine & SM) {
    KEDemoMachine & sm = static_cast<KEDemoMachine &>(SM); //Cast to specific StateMachine type

    //keyboard or joystick press
    if ( sm.robot()->keyboard->getX()==1) 
        return true;

    //Otherwise false
    return false;
}

bool standby(StateMachine & SM) {
    KEDemoMachine & sm = (KEDemoMachine &)SM; //Cast to specific StateMachine type

    if (sm.robot()->keyboard->getQ()==1) {
        return true;
    }
    return false;
}


KEDemoMachine::KEDemoMachine() {
    //Create a Robot and set it to generic state machine
    setRobot(std::make_unique<RobotKE>("KNEE_EXO", "KE_params.yaml"));

    //Create state instances and add to the State Machine
    addState("TestState", std::make_shared<KEDemoState>(robot()));
    addState("CalibState", std::make_shared<KECalibState>(robot()));
    addState("CalibState2", std::make_shared<KECalibState2>(robot()));

    addState("StandbyState", std::make_shared<KEMassCompensation>(robot()));
    addState("PosControlState", std::make_shared<KEPosControlDemo>(robot()));
    addState("VelControlState", std::make_shared<KEVelControlDemo>(robot()));
    addState("TorControlState", std::make_shared<KETorControlDemo>(robot()));
    addState("TorControlForPosState", std::make_shared<KETorControlForPosDemo>(robot()));
    addState("TorControlForSpringState", std::make_shared<KETorControlForSpring>(robot()));

    //Define transitions between states
    addTransition("CalibState", &goToNextState, "CalibState2");
    addTransition("CalibState2", &goToNextState, "TorControlForSpringState");
    addTransitionFromLast(&goToNextState, "TorControlState");
    addTransitionFromLast(&goToNextState, "StandbyState");
    addTransitionFromAny(&standby, "StandbyState");

    //Initialize the state machine with first state of the designed state machine
    setInitState("TorControlState");
}
KEDemoMachine::~KEDemoMachine() {
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void KEDemoMachine::init() {
    spdlog::debug("KEDemoMachine::init()");
    if(robot()->initialise()) {
        spdlog::debug("Successfully initialised Step 1");
        //std::string filename = "logs/xxx.csv"; // grab time and rename file afterwards
        logHelper.initLogger("KEDemoMachineLog", "logs/KEDemoMachine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getPosition(), "q");
        logHelper.add(robot()->getVelocity(), "dq");
        logHelper.add(robot()->getTorque(), "tau");
        logHelper.add(robot()->getSpringPosition(), "springQ");

        //logHelper.add(robot()->getEndEffPosition(), "X");
        //logHelper.add(robot()->getEndEffVelocity(), "dX");
        //UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");
        spdlog::debug("Successfully initialised Step 2");
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}


