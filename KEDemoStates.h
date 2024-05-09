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
     double stop_reached_time;
     bool at_stop_step_1 = false;
     bool at_stop_step_2 = false;
     bool calibDone=false;
     bool calibStep1Done = false;
     bool calibStep2Done = false;
     int calib_step = 0;
     double step2_t = 0.;
     double targetSpringPos = -110.;
     double targetSpringVel = 0.;
     motorProfile controlMotorProfile = {1000, 10000, 10000};
    int targetPosStep2Set = 1;
    double caliStep1MotorPos = 0.0;
    double targetMotorPos = 0.;
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
    KETorControlDemo(RobotKE * KE, const char *name = "Velocity Control Demo"):KETimedState(KE, name){};

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
    KETorControlForPosDemo(RobotKE * KE, const char *name = "Velocity Control Demo"):KETimedState(KE, name){};

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

#endif
