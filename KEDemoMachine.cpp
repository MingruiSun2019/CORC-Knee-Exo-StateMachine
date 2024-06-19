#include "KEDemoMachine.h"

using namespace std;

bool endCalib(StateMachine & sm) {
    return (sm.state<KECalibState>("CalibState2"))->isCalibDone();
}

bool e_stop_handler(StateMachine & sm){
    const char *const gpio_estop_pin = "/sys/class/gpio/PQ.05/value";
    //const char *const gpio_button_pin = "/sys/class/gpio/PAC.06/value";
    std::ifstream e_stop_gpio_file(gpio_estop_pin);
    //std::ifstream button_gpio_file(gpio_button_pin);

    if (e_stop_gpio_file.is_open())
    {
        int gpio_estop_status; // or perhaps a string?
        //int gpio_button_status; // or perhaps a string?

        e_stop_gpio_file >> gpio_estop_status;
        //button_gpio_file >> gpio_button_status;
        //spdlog::debug("estop: {}, button: {}", gpio_estop_status, gpio_button_status);

        if (gpio_estop_status == 0){
            //spdlog::debug("estop: {}, button: {}", gpio_estop_status, gpio_button_status);
            spdlog::error("E-stop pressed");
            std::raise(SIGINT);
        }
        return false;
    }
    return false;
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

bool rotateExtension(StateMachine & SM) {
    KEDemoMachine & sm = (KEDemoMachine &)SM; //Cast to specific StateMachine type
    if (sm.robot()->keyboard->getW()==1) {
        return true;
    }
    return false;
}

bool rotateFlexion(StateMachine & SM) {
    KEDemoMachine & sm = (KEDemoMachine &)SM; //Cast to specific StateMachine type
    if (sm.robot()->keyboard->getS()==1) {
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
    addState("ZeroLevelForceState", std::make_shared<KEZeroLevelForce>(robot()));

    addState("StandbyState", std::make_shared<KEStandByState>(robot()));
    addState("CalibExerRobDriveState", std::make_shared<KECalibExerRobDriveState>(robot()));
    addState("CalibExerHumDriveState", std::make_shared<KECalibExerHumDriveState>(robot()));
    addState("ActualExerRobDriveState", std::make_shared<KEActualExerRobDriveState>(robot()));

    addState("PosControlState", std::make_shared<KEPosControlDemo>(robot()));
    addState("VelControlState", std::make_shared<KEVelControlDemo>(robot()));
    addState("TorControlState", std::make_shared<KETorControlDemo>(robot()));
    addState("TorControlForPosState", std::make_shared<KETorControlForPosDemo>(robot()));
    addState("TorControlForSpringState", std::make_shared<KETorControlForSpring>(robot()));
    addState("Sit2StandState", std::make_shared<KETorCtrlSit2Stand>(robot()));

    //Define transitions between states
    addTransition("CalibState", &goToNextState, "CalibState2");
    addTransition("CalibState2", &goToNextState, "ZeroLevelForceState");
    addTransition("ZeroLevelForceState", &goToNextState, "CalibExerRobDriveState");
    addTransition("CalibExerRobDriveState", &goToNextState, "CalibExerHumDriveState");
    addTransition("CalibExerHumDriveState", &goToNextState, "ActualExerRobDriveState");
    addTransition("ActualExerRobDriveState", &goToNextState, "Sit2StandState");

    addTransitionFromLast(&goToNextState, "TorControlState");
    addTransitionFromLast(&goToNextState, "StandbyState");
    addTransitionFromAny(&standby, "StandbyState");
    addTransitionFromAny(&e_stop_handler, "StandbyState");

    // State machine for experiment
    /*
    addTransition("CalibState", &goToNextState, "CalibState2");
    addTransition("CalibState2", &goToNextState, "TorControlForSpringState");
    addTransitionFromLast(&goToNextState, "PosControlState");
    addTransitionFromLast(&goToNextState, "ParameterIdState");
    addTransitionFromLast(&goToNextState, "SlowSineRIC");
    addTransitionFromLast(&goToNextState, "FastSineRIC");

    addTransition("StandbyState", &rotateExtension, "IncrePosFlexState");
    addTransition("StandbyState", &rotateFlexion, "IncrePosExtState");
    */



    //Initialize the state machine with first state of the designed state machine
    setInitState("CalibState");
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
        //spdlog::debug("Successfully initialised Step 1");
        //std::string filename = "logs/xxx.csv"; // grab time and rename file afterwards

        /*
        logHelper.initLogger("KEDemoMachineLog", "logs/ABC.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(), "Time (s)");
        logHelper.add(robot()->getPosition(), "q");
        logHelper.add(robot()->getVelocity(), "dq");
        logHelper.add(robot()->getTorque(), "tau");
        logHelper.add(robot()->getSpringPosition(), "springQ");
        */

        //logHelper.add(robot()->getEndEffPosition(), "X");
        //logHelper.add(robot()->getEndEffVelocity(), "dX");
        //UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.7.2");
        spdlog::debug("Successfully initialised");
    }
    else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
}


