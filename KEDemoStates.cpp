#include "KEDemoStates.h"
#include "KEDemoMachine.h"
#include "JetsonGPIO.h"


#define KNEE 0  // the second joint

using namespace std;

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM3 impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, VM3 X0, VM3 X, VM3 dX, VM3 dXd=VM3::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}



void KEDemoState::entryCode(void) {
    //robot->applyCalibration();
    //robot->initPositionControl();
    //robot->initVelocityControl();
    //robot->initTorqueControl();
    qi=robot->getPosition();
    Xi=robot->getEndEffPosition();
    //robot->setJointVelocity(VM3::Zero());
    //robot->setEndEffForceWithCompensation(VM3::Zero(), false);
    robot->printJointStatus();
    robot->initTorqueControl();
    robot->setJointTorque(0);
    tau = 0;
    lock = false;
}
void KEDemoState::duringCode(void) {
    // todo
    }
void KEDemoState::exitCode(void) {
    robot->setJointVelocity(0);
    //robot->setEndEffForceWithCompensation(VM3(0,0,0));
    robot->setJointTorque(0);
}



void KECalibState::entryCode(void) {
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();

    //spdlog::debug("Geting FT sensors ID: {}", sensorID);
    //spdlog::debug("Geting FT sensors Forces: {}", sensorForce[0]);


    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void KECalibState::duringCode(void) {
    tau = -6;
    robot->setJointTorque(tau);
    if(iterations()%100==1) {
        std::cout << "step 1 ..." << std::flush;
    }
}

void KECalibState::exitCode(void) {
    calib_step = 1;
    robot->applyCalibration(calib_step);
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    spdlog::debug("spring pos: {} Calibrated", springPos[0]);

    //Eigen::VectorXd& caliStep1MotorPosTemp = robot->getPosition();
    //caliStep1MotorPos = caliStep1MotorPosTemp[KNEE];
    std::cout << "Calibration step 1 OK." << std::endl;
    //spdlog::debug("Motor Pos Step 1: {}", caliStep1MotorPos);
}


void KECalibState2::entryCode(void) {
    stop_reached_time = .0;
    at_stop_step_2 = false;
    Eigen::VectorXd& caliStep1MotorPos = robot->getPosition();
    caliStep1MotorPosScalar = caliStep1MotorPos[KNEE];
    
    controlMotorProfile.profileVelocity = 400;
    robot->initProfilePositionControl(controlMotorProfile);
    std::cout << "Calibrating (keep clear)..." << std::flush;

}
//Move slowly on each joint until max force detected
void KECalibState2::duringCode(void) {

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    //Eigen::VectorXd& springPos=robot->getSpringPosition();
    //spdlog::debug("SpringPos: {}", springPos[KNEE]);
    Eigen::VectorXd& motorVel = robot->getVelocity();

    if (targetPosStep2Set <= 6) {
        if(iterations()%100==1) {
            robot->setJointPosition(caliStep1MotorPosScalar + (110+abs(tau)/5.44)*M_PI/180.);
            targetPosStep2Set++;
        }
    }

    if(abs(motorVel[KNEE])<0.01) {
        stop_reached_time += dt();
    }
    if(stop_reached_time > 1.) {
        calib_step = 2;
        robot->applyCalibration(calib_step);

        if(iterations()%100==1) {
            std::cout << "Calibration step 2 OK." << std::endl;
        }
    }
    if(robot->isCalibrated()) {
        calibDone=true; //Trigger event
    }
}

void KECalibState2::exitCode(void) {
    //robot->setEndEffForceWithCompensation(VM3(0,0,0));
    ;
}


void KEMassCompensation::entryCode(void) {
    robot->initTorqueControl();
    std::cout << "Press S to decrease mass (-100g), W to increase (+100g)." << std::endl;
}
void KEMassCompensation::duringCode(void) {

    //Smooth transition in case a mass is set at startup
    double settling_time = 3.0;
    double t=running()>settling_time?1.0:running()/settling_time;

    //Bound mass to +-5kg
    if(mass>15.0) {
        mass = 15.;
    }
    if(mass<-15) {
        mass = -15.;
    }

    //Apply corresponding force
    robot->setEndEffForceWithCompensation(VM3(0,0,t*mass*9.8), true);

    //Mass controllable through keyboard inputs
    if(robot->keyboard->getS()) {
        mass -=0.5;robot->printStatus();
        robot->printJointStatus();
        std::cout << "Mass: " << mass << std::endl;
    }
    if(robot->keyboard->getW()) {
        mass +=0.5;robot->printStatus();
        robot->printJointStatus();
        std::cout << "Mass: " << mass << std::endl;
    }
}
void KEMassCompensation::exitCode(void) {
    robot->setEndEffForceWithCompensation(VM3(0,0,0));
}


void KEPosControlDemo::entryCode(void) {
    robot->applyCalibration(1);
    robot->applyCalibration(2);


    robot->initCyclicPositionControl(controlMotorProfile);
    double step2_t = 0;

}
void KEPosControlDemo::duringCode(void) {
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    Eigen::VectorXd& motorPos=robot->getPosition();

    // Calcualte joint reference postion
    step2_t += dt();
    targetSpringPos = (20.*cos(0.4*step2_t) - 20.) * M_PI / 180.0;
    //spdlog::debug("spring pos: {}, motor pos: {}, targetPos: {}", springPos[0], motorPos[0], targetSpringPos);


    // Set position
    robot->setJointPosition(targetSpringPos);

    //Eigen::VectorXd& motorVel=robot->getVelocity();
    //spdlog::debug("motorVel: {}", motorVel[KNEE]);

    if(iterations()%100==1) {
        std::cout << "springPOS: " << springPos[KNEE] << std::endl;
        robot->printStatus();
    }
}
void KEPosControlDemo::exitCode(void) {
    ;
    //robot->setPosition(std::vector<double> 0.0);
}

void KEVelControlDemo::entryCode(void) {
    robot->applyCalibration(1);
    robot->applyCalibration(2);

    robot->initProfileVelocityControl(controlMotorProfile);
    usleep(10000);
    double step2_t = 0;

}
void KEVelControlDemo::duringCode(void) {

    // Calcualte joint reference postion
    step2_t += dt();
    targetMotorVel = 20.*sin(0.3*step2_t) * M_PI / 180.0;

    // Set position
    robot->setJointVelocity(targetMotorVel);

    Eigen::VectorXd& motorVel=robot->getVelocity();
    spdlog::debug("motorVel: {}", motorVel[KNEE]);

    if(iterations()%100==1) {
        std::cout << "refVel: " << targetMotorVel << std::endl;
        robot->printStatus();
    }
}
void KEVelControlDemo::exitCode(void) {
    ;
    //robot->setPosition(std::vector<double> 0.0);
}


void KETorControlDemo::entryCode(void) {
    robot->applyCalibration(1);
    robot->applyCalibration(2);

    robot->initTorqueControl();
    usleep(10000);
    robot->stopSensorStreaming();
    usleep(10000);

    robot->startSensorStreaming();

    robot->sensorCalibration();

    double step2_t = 0;

    int but_pin = 12;
    //GPIO::setmode(GPIO::BOARD);
    //GPIO::setup(but_pin, GPIO::IN);


}
void KETorControlDemo::duringCode(void) {

    // Calcualte joint reference postion
    step2_t += dt();
    targetMotorTor = 5.*sin(0.8*step2_t);

    // Set position
    robot->setJointTorque(targetMotorTor);

    const char *const gpio_estop_pin = "/sys/class/gpio/PQ.05/value";
    std::ifstream amp(gpio_estop_pin);
    if (amp.is_open())
    {
        int gpio_estop_status; // or perhaps a string?
        amp >> gpio_estop_status;
        spdlog::debug("gpio_estop_status: {}", gpio_estop_status);
    }
    //int gpio_estop_status = open("/sys/class/gpio/PQ.05/value", O_RDONLY);

    //Eigen::VectorXd& motorVel=robot->getVelocity();
    Eigen::VectorXd& ft1_force=robot->getForces(1,1);
    Eigen::VectorXd& ft1_torque=robot->getForces(1,2);
    Eigen::VectorXd& ft2_force=robot->getForces(2,1);
    Eigen::VectorXd& ft2_torque=robot->getForces(2,2);

    //spdlog::debug("ftForce: {}", ftForce[0]);

    if(iterations()%10==1) {
        //std::cout << "refVel: " << targetMotorTor << std::endl;
        spdlog::debug("ft1_force: {}", ft1_force[0]);
        spdlog::debug("ft1_torque: {}", ft1_torque[0]);
        spdlog::debug("ft2_force: {}", ft2_force[0]);
        spdlog::debug("ft2_torque: {}", ft2_torque[0]);

        robot->printStatus();
    }
}
void KETorControlDemo::exitCode(void) {
    ;
    //robot->setPosition(std::vector<double> 0.0);
}

void KETorControlForPosDemo::entryCode(void) {
    robot->applyCalibration(1);
    robot->applyCalibration(2);

    robot->initTorqueControl();
    usleep(10000);
    double step2_t = 0;

}
void KETorControlForPosDemo::duringCode(void) {

    // Calcualte joint reference postion
    step2_t += dt();
    targetMotorPos = 10.*sin(0.5*step2_t) * M_PI / 180.0;
    targetMotorVel = 5.*cos(0.5*step2_t) * M_PI / 180.0;
    targetMotorAcc = -2.5*sin(0.5*step2_t) * M_PI / 180.0;

    // Set position
    robot->applySpringPositionTorControl(targetMotorPos, targetMotorVel, targetMotorAcc);

    Eigen::VectorXd& motorVel=robot->getVelocity();
    spdlog::debug("motorVel: {}", motorVel[KNEE]);

    if(iterations()%100==1) {
        std::cout << "refVel: " << targetMotorPos << std::endl;
        robot->printStatus();
    }
}
void KETorControlForPosDemo::exitCode(void) {
    ;
    //robot->setPosition(std::vector<double> 0.0);
}


void KETorControlForSpring::entryCode(void) {
    robot->initCyclicPositionControl(controlMotorProfile);
    usleep(10000);
    state_t = 0.;
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    Eigen::VectorXd& motorPos=robot->getPosition();

    // Calcualte joint reference postion
    spdlog::debug("spring pos: {}, motor pos: {}", springPos[0], motorPos[0]);

}
void KETorControlForSpring::duringCode(void) {

    // Calcualte joint reference postion
    state_t += dt();
    targetSpringTor = 3.*sin(0.5*state_t) * M_PI / 180.0;
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    Eigen::VectorXd& motorPos=robot->getPosition();

    targetMotorPos = springPos[KNEE] + targetSpringTor / springK;
    targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);

    // Set position
    robot->setJointPosition(targetMotorPosFilt);

    if(iterations()%100==1) {
        std::cout << "refVel: " << targetMotorPosFilt << std::endl;
        robot->printStatus();
    }
}
void KETorControlForSpring::exitCode(void) {
    ;
    //robot->setPosition(std::vector<double> 0.0);
}