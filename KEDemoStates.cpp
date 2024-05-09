#include "KEDemoStates.h"
#include "KEDemoMachine.h"

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
    calibDone=false;
    calibStep1Done = false;
    calibStep2Done = false;

    stop_reached_time = .0;
    at_stop_step_1 = false;
    at_stop_step_2 = false;
    calib_step = 0;
    
    robot->decalibrate();
    robot->initTorqueControl();
    robot->printJointStatus();
    std::cout << "Calibrating (keep clear)..." << std::flush;
}
//Move slowly on each joint until max force detected
void KECalibState::duringCode(void) {
    double tau = -6;  // the torque to drive the output link to the first stop

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    Eigen::VectorXd& springVel=robot->getSpringVelocity();
    Eigen::VectorXd& motorVel=robot->getVelocity();
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    spdlog::debug("SpringPos: {}", springPos[KNEE]);

    //double b = 7.;
    //tau(KNEE) = std::min(std::max(8 - b * vel(i), .0), 8.);

    if (calibStep1Done==false && calibStep2Done==false){
        // Calibration step 1:
        // drive the output link to the safety stop
        robot->setJointTorque(tau);
        if(iterations()%100==1) {
            std::cout << "step 1 ..." << std::flush;
        }

        if(abs(springVel[KNEE])<0.01) {
            stop_reached_time += dt();
        }
        if(stop_reached_time>1.0) {
            at_stop_step_1=true;
            stop_reached_time = 0;
            calib_step = 1;
            robot->applyCalibration(calib_step);
            calibStep1Done = true;
            Eigen::VectorXd& caliStep1MotorPosTemp = robot->getPosition();
            caliStep1MotorPos = caliStep1MotorPosTemp[KNEE];
            std::cout << "Calibration step 1 OK." << std::endl;
            spdlog::debug("Motor Pos Step 1: {}", caliStep1MotorPos);

            controlMotorProfile.profileVelocity = 400;
            robot->initProfilePositionControl(controlMotorProfile);

            step2_t = 0.0;

        }
    }
    else if (calibStep1Done==true && calibStep2Done==false){
        // Calibration step 2:
        // drive the output link to the vertical position
        //spdlog::debug("Step 2 time 1: {}", step2_t);
        //float actualSpringPos = ((JointKE *)p)->getSpringPosition();

        if (targetPosStep2Set <= 4) {
            targetMotorPos = caliStep1MotorPos;
            robot->setJointPosition(targetMotorPos + (110+abs(tau)/5.44)*M_PI/180.);
            targetPosStep2Set++;
        }

        if(abs(motorVel[KNEE])<0.01) {
            stop_reached_time += dt();
        }
        if(stop_reached_time > 1.) {
            at_stop_step_2=true;
            calib_step = 2;
            robot->applyCalibration(calib_step);
            calibStep2Done = true;
            std::cout << "Calibration step 2 OK." << std::endl;
        }
    }
    else if (calibStep1Done==true && calibStep2Done==true){
        //Switch to gravity contro when done
        if(robot->isCalibrated()) {
            //robot->setEndEffForceWithCompensation(VM3(0,0,0));
            calibDone=true; //Trigger event
        }
    }
    else {
        spdlog::debug("Calibration fail, restart.");
        calibStep1Done = false;
        calibStep2Done = false;
    }
}

void KECalibState::exitCode(void) {
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


    robot->initProfilePositionControl(controlMotorProfile);
    double step2_t = 0;

}
void KEPosControlDemo::duringCode(void) {

    // Calcualte joint reference postion
    step2_t += dt();
    targetSpringPos = 0.2*sin(0.4*step2_t) * M_PI / 180.0;

    // Set position
    robot->setJointPosition(targetSpringPos);

    Eigen::VectorXd& motorVel=robot->getVelocity();
    spdlog::debug("motorVel: {}", motorVel[KNEE]);

    if(iterations()%100==1) {
        std::cout << "refPos: " << targetSpringPos << std::endl;
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
    double step2_t = 0;

}
void KETorControlDemo::duringCode(void) {

    // Calcualte joint reference postion
    step2_t += dt();
    targetMotorTor = 5.*sin(0.8*step2_t);

    // Set position
    robot->setJointTorque(targetMotorTor);

    Eigen::VectorXd& motorVel=robot->getVelocity();
    spdlog::debug("motorVel: {}", motorVel[KNEE]);

    if(iterations()%100==1) {
        std::cout << "refVel: " << targetMotorTor << std::endl;
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
