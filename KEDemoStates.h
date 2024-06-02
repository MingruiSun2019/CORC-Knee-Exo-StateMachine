/**
 * \file KEDemoState.h
 * \author Vincent Crocher
 * \version 0.3
 * \date 2020-07-27
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef KEDemoSTATES_H_DEF
#define KEDemoSTATES_H_DEF

#include "State.h"
#include "RobotKE.h"
#include "StateMachine.h"



class KEDemoMachine;

/**
 * \brief Generic state type for used with KEDemoMachine, providing running time and iterations number: been superseeded by default state.
 *
 */
class KETimedState : public State {
   protected:
    RobotKE * robot;                               //!< Pointer to state machines robot object

    KETimedState(RobotKE* KE, const char *name = NULL): State(name), robot(KE){spdlog::debug("Created KETimedState {}", name);};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};
};


class KEDemoState : public KETimedState {

   public:
    KEDemoState(RobotKE * KE, const char *name = "KE Test State"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    VM3 qi, Xi;
    double tau;
    bool lock;
};

class KEStandByState : public KETimedState {

   public:
    KEStandByState(RobotKE * KE, const char *name = "KE Test State"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    int keyPressed;
    LogHelper logHelperSB;
};


/**
 * \brief Position calibration of KE. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class KECalibState : public KETimedState {

   public:
    KECalibState(RobotKE * KE, const char *name = "KE Calib State"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     int calib_step = 0;
     double tau = -6.;
     bool calibDone = false;

};

class KECalibState2 : public KETimedState {

   public:
    KECalibState2(RobotKE * KE, const char *name = "KE Calib 2 State"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     double stop_reached_time;
     bool at_stop_step_2 = false;
     double caliStep1MotorPosScalar = 0.;
     int targetPosStep2Set = 0;
     int calib_step = 0;
     double tau = -6.;
     bool calibDone = false;
     motorProfile controlMotorProfile = {1000, 10000, 10000};
};

/**
 * \brief Provide end-effector mass compensation on KE. Mass is controllable through keyboard inputs.
 *
 */
class KEMassCompensation : public KETimedState {

   public:
    KEMassCompensation(RobotKE * KE, const char *name = "MKEMass Compensation"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
     double mass = 0;
};


/**
 * \brief Position control Demo.
 *
 */
class KEPosControlDemo : public KETimedState {

   public:
    KEPosControlDemo(RobotKE * KE, const char *name = "Position Control Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     double stop_reached_time;
     double step2_t = 0.0;
     double targetSpringPos = -0.0;
     motorProfile controlMotorProfile = {1000, 10000, 10000};
};

class KEVelControlDemo : public KETimedState {

   public:
    KEVelControlDemo(RobotKE * KE, const char *name = "Velocity Control Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     double stop_reached_time;
     double step2_t = 0.0;
     double targetMotorVel = -0.0;
     motorProfile controlMotorProfile = {1000, 1000, 1000};
};

class KETorControlDemo : public KETimedState {

   public:
    KETorControlDemo(RobotKE * KE, const char *name = "Torque Control Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     double stop_reached_time;
     double step2_t = 0.0;
     double targetMotorTor = -0.0;
     motorProfile controlMotorProfile = {1000, 1000, 1000};
};

class KETorControlForPosDemo : public KETimedState {

   public:
    KETorControlForPosDemo(RobotKE * KE, const char *name = "Torque Control for Pos Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     double stop_reached_time;
     double step2_t = 0.0;
     double targetMotorPos = -0.0;
     double targetMotorVel = -0.0;
     double targetMotorAcc = -0.0;

     motorProfile controlMotorProfile = {1000, 1000, 1000};
};

class KETorControlForSpring : public KETimedState {

   public:
    KETorControlForSpring(RobotKE * KE, const char *name = "Spring Torque Control Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     double state_t = 0.0;
     double springK = 5.44;
     double targetSpringTor = 0.;
     double targetMotorPos = 0.;
     double targetMotorPosFilt = 0.;
     double decay = 0.75;

     motorProfile controlMotorProfile = {1000, 1000, 1000};
};

class KECalibExerRobDriveState : public KETimedState {

   public:
    KECalibExerRobDriveState(RobotKE * KE, const char *name = "Sit to Stand Torque Control Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     int logger_count = 1;
     double action_t = 0.0;
     double springK = 5.44 * 180 / M_PI;
     double targetSpringTor = 0.;
     double targetMotorPos = 0.;
     double targetMotorPosFilt = 0.;
     double decay = 0.75;
     bool actionFlag = false;
     double user_butt_time = 0.;
     bool user_butt_pressed = false;
     bool button_released = true;
     std::string testCondition = "init";
     int angleFlag = 0;
     int emgFlag = 0;
     double test_duration = 5.;
     std::vector<std::string> actionText{"No Action", "In Action"};
     std::vector<std::string> angleText{"LowAngle", "HighAngle"};
     std::vector<std::string> emgText{"Relax", "CoContract"};

     LogHelper logHelperSB;

     motorProfile controlMotorProfile = {1000, 1000, 1000};    

     void initLoggerStandard(std::string testCondition); 
};

class KECalibExerHumDriveState : public KETimedState {

   public:
    KECalibExerHumDriveState(RobotKE * KE, const char *name = "Sit to Stand Torque Control Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     int logger_count = 1;
     double action_t = 0.0;
     double springK = 5.44 * 180 / M_PI;
     double springVel = 0.;
     double targetTorFilt = 0.;
     double targetSpringTor = 0.;
     double targetMotorPos = 0.;
     double targetMotorPosFilt = 0.;
     double decay = 0.75;  // the higher the decay, the lower the cut off frequency
     bool actionFlag = false;
     double user_butt_time = 0.;
     bool user_butt_pressed = false;
     bool button_released = true;
     int inFlexing = 1;
     std::string testCondition = "init";
     int resistanceFlag = 1;
     std::vector<std::string> actionText{"No Action", "In Action"};
     std::vector<std::string> resistanceText{"ForceLevel5Nm", "ForceLevel10Nm","ForceLevel15Nm","ForceLevel20Nm"};
     std::vector<std::string> modeText{"Extending", "Flexing"};

     LogHelper logHelperSB;

     motorProfile controlMotorProfile = {1000, 1000, 1000};    

     void initLoggerStandard(std::string testCondition); 
};


class KEActualExerRobDriveState : public KETimedState {

   public:
    KEActualExerRobDriveState(RobotKE * KE, const char *name = "Actual Exercise Robot in Charge"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     int logger_count = 1;
     double action_t = 0.0;
     double springK = 5.44 * 180 / M_PI;
     double springVel = 0.;
     double targetTorFilt = 0.;
     double targetSpringTor = 0.;
     double targetMotorPos = 0.;
     double targetMotorPosFilt = 0.;
     double decay = 0.75;  // the higher the decay, the lower the cut off frequency
     bool actionFlag = false;
     double user_butt_time = 0.;
     bool user_butt_pressed = false;
     bool button_released = true;
     double sineFreq = 0.;
     bool ready_to_test = false;
     int getting_ready = 0;
     double test_duration = 0.;
     std::string testCondition = "init";
     int freqFlag = 0;
     int resistanceFlag = 0;
     std::vector<double> freqTable{0.3, 0.7};
     std::vector<std::string> freqText{"SlowMove 0.3Hz", "FastMove 0.7Hz"};
     std::vector<std::string> actionText{"No Action", "In Action"};
     std::vector<std::string> resistanceText{"Relax", "ResistFlexion", "ResistExtension", "Co-Contraction"};

     LogHelper logHelperSB;

     motorProfile controlMotorProfile = {1000, 1000, 1000};    

     void initLoggerStandard(std::string testCondition); 
};

class KETorCtrlSit2Stand : public KETimedState {

   public:
    KETorCtrlSit2Stand(RobotKE * KE, const char *name = "Sit to Stand Torque Control Demo"):KETimedState(KE, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    private:
     int logger_count = 1;
     double action_t = 0.0;
     double springK = 5.44 * 180 / M_PI;
     double targetSpringTor = 0.;
     double targetTorFilt = 0.;
     double targetMotorPos = 0.;
     double targetMotorPosFilt = 0.;
     double decay = 0.75;
     double user_butt_time = 0.;
     int actionFlag = 0;
     bool user_butt_pressed = false;
     bool button_released = true;
     std::string testCondition = "init";
     int assistanceFlag = 0;

     std::vector<double> assistanceLevel{10., 20.};
     std::vector<std::string> actionText{"No Action", "In Action"};
     std::vector<std::string> assistanceText{"AssistLevel 10Nm", "AssistLevel 20Nm"};

     LogHelper logHelperSB;

     motorProfile controlMotorProfile = {1000, 1000, 1000};     
     std::vector<double> sit2stdAngSmp{-110., -100., -80., -60., -40., -20., 0.};
     std::vector<double> sit2stdTorProfile_40{0.07,0.1,0.28,0.24,0.1,0.02,0.02};

     void initLoggerStandard(std::string testCondition); 
};

#endif
