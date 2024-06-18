#include "KEDemoStates.h"
#include "KEDemoMachine.h"
#include "StateMachine.h"
#include <algorithm>
#include <iostream>
#include <ctime>


#define KNEE 0  // the second joint
#define IMU_ID0 0
#define IMU_ID1 1
#define IMU_ACCL 0
#define IMU_GYRO 1
#define IMU_ORIEN 2

#define CALIB_ANGLE_1 110

using namespace std;

double timeval_to_sec(struct timespec *ts)
{
    return (double)(ts->tv_sec + ts->tv_nsec / 1000000000.0);
}

VM3 impedance(Eigen::Matrix3d K, Eigen::Matrix3d D, VM3 X0, VM3 X, VM3 dX, VM3 dXd=VM3::Zero()) {
    return K*(X0-X) + D*(dXd-dX);
}

void toEulerAngles(double w, double x, double y, double z, double eularAngles[]) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 2 * (w * w + z * z) - 1;
    eularAngles[0] = std::atan2(sinr_cosp, cosr_cosp);
    // Pitch (y-axis rotation)
    double sinp = 2 * (x * z - w * y);
    eularAngles[1] = -std::asin(sinp);
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 2 * (w * w + x * x) - 1;
    eularAngles[2] = std::atan2(siny_cosp, cosy_cosp);
}

void KEDemoState::entryCode(void) {
    spdlog::info("In Joystick Test State");
}
void KEDemoState::duringCode(void) {

    //Test Acc datas
    //Eigen::VectorXd& acclA = robot->getTeensyData(1,1);
    Eigen::VectorXd& tsy1_sens1_acc = robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ACCL);
    Eigen::VectorXd& tsy1_sens1_gyro = robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_GYRO);
    Eigen::VectorXd& tsy1_sens1_orien = robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ORIEN);
    Eigen::VectorXd& tsy1_sens2_acc = robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ACCL);
    Eigen::VectorXd& tsy1_sens2_gyro = robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_GYRO);
    Eigen::VectorXd& tsy1_sens2_orien = robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ORIEN);
    // for teensy 2
    Eigen::VectorXd& tsy2_sens1_acc = robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ACCL);
    Eigen::VectorXd& tsy2_sens1_gyro = robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_GYRO);
    Eigen::VectorXd& tsy2_sens1_orien = robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ORIEN);
    Eigen::VectorXd& tsy2_sens2_acc = robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ACCL);
    Eigen::VectorXd& tsy2_sens2_gyro = robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_GYRO);
    Eigen::VectorXd& tsy2_sens2_orien = robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ORIEN);

    double quat_euler_t1_s1[3] = {0}, quat_euler_t1_s2[3] = {0}, quat_euler_t2_s1[3] = {0}, quat_euler_t2_s2[3] = {0};
    toEulerAngles(tsy1_sens1_orien[0], tsy1_sens1_orien[1], tsy1_sens1_orien[2], tsy1_sens1_orien[3], quat_euler_t1_s1); 
    toEulerAngles(tsy1_sens2_orien[0], tsy1_sens2_orien[1], tsy1_sens2_orien[2], tsy1_sens2_orien[3], quat_euler_t1_s2); 
    toEulerAngles(tsy2_sens1_orien[0], tsy2_sens1_orien[1], tsy2_sens1_orien[2], tsy2_sens1_orien[3], quat_euler_t2_s1); 
    toEulerAngles(tsy2_sens2_orien[0], tsy2_sens2_orien[1], tsy2_sens2_orien[2], tsy2_sens2_orien[3], quat_euler_t2_s2); 

    std::time_t raw_time_now = std::time(nullptr);
    struct tm* time_info = std::localtime(&raw_time_now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%m_%d_%H_%M_%S", time_info);
    std::string time_now(buffer);

    if(iterations()%3==1) {
        std::cout << "Time: " << time_now << std::endl;

        std::cout << "Teensy 1, Senosor 1 Acc: " << tsy1_sens1_acc[0] << ", " << tsy1_sens1_acc[1]  << ", " << tsy1_sens1_acc[2] << std::endl;
        std::cout << "Teensy 1, Senosor 1 Gyro: " << tsy1_sens1_gyro[0] << ", " << tsy1_sens1_gyro[1] << ", " << tsy1_sens1_gyro[2] << std::endl;
        std::cout << "Teensy 1, Senosor 1 Orien: " << tsy1_sens1_orien[0] << ", " << tsy1_sens1_orien[1] << ", " << tsy1_sens1_orien[2] << ", " << tsy1_sens1_orien[3] << std::endl;
        std::cout << "Teensy 1, Senosor 1 Angle: " << quat_euler_t1_s1[0]*180./3.14 << ", " << quat_euler_t1_s1[1]*180./3.14 << ", " << quat_euler_t1_s1[2]*180./3.14 << std::endl;

        std::cout << "Teensy 1, Senosor 2 Acc: " << tsy1_sens2_acc[0] << ", " << tsy1_sens2_acc[1] << ", " << tsy1_sens2_acc[2] << std::endl;
        std::cout << "Teensy 1, Senosor 2 Gyro: " << tsy1_sens2_gyro[0] << ", " << tsy1_sens2_gyro[1] << ", " << tsy1_sens2_gyro[2] << std::endl;
        std::cout << "Teensy 1, Senosor 2 Orien: " << tsy1_sens2_orien[0] << ", " << tsy1_sens2_orien[1] << ", " << tsy1_sens2_orien[2] << ", " << tsy1_sens2_orien[3] << std::endl;
        std::cout << "Teensy 1, Senosor 2 Angle: " << quat_euler_t1_s2[0]*180./3.14 << ", " << quat_euler_t1_s2[1]*180./3.14 << ", " << quat_euler_t1_s2[2]*180./3.14 << std::endl;

        std::cout << "--------" << std::endl;
        std::cout << "Teensy 2, Senosor 1 Acc: " << tsy2_sens1_acc[0] << ", " << tsy2_sens1_acc[1] << ", " << tsy2_sens1_acc[2] << std::endl;
        std::cout << "Teensy 2, Senosor 1 Gyro: " << tsy2_sens1_gyro[0] << ", " << tsy2_sens1_gyro[1] << ", " << tsy2_sens1_gyro[2] << std::endl;
        std::cout << "Teensy 2, Senosor 1 Orien: " << tsy2_sens1_orien[0] << ", " << tsy2_sens1_orien[1] << ", " << tsy2_sens1_orien[2] << ", " << tsy2_sens1_orien[3] << std::endl;
        std::cout << "Teensy 2, Senosor 1 Angle: " << quat_euler_t2_s1[0]*180./3.14 << ", " << quat_euler_t2_s1[1]*180./3.14 << ", " << quat_euler_t2_s1[2]*180./3.14 << std::endl;

        std::cout << "Teensy 2, Senosor 2 Acc: " << tsy2_sens2_acc[0] << ", " << tsy2_sens2_acc[1] << ", " << tsy2_sens2_acc[2] << std::endl;
        std::cout << "Teensy 2, Senosor 2 Gyro: " << tsy2_sens2_gyro[0] << ", " << tsy2_sens2_gyro[1] << ", " << tsy2_sens2_gyro[2] << std::endl;
        std::cout << "Teensy 2, Senosor 2 Orien: " << tsy2_sens2_orien[0] << ", " << tsy2_sens2_orien[1] << ", " << tsy2_sens2_orien[2] << ", " << tsy2_sens2_orien[3] << std::endl;
        std::cout << "Teensy 2, Senosor 2 Angle: " << quat_euler_t2_s2[0]*180./3.14 << ", " << quat_euler_t2_s2[1]*180./3.14 << ", " << quat_euler_t2_s2[2]*180./3.14 << std::endl;

        std::cout << "==================" << std::endl;
    }
}
void KEDemoState::exitCode(void) {
    spdlog::info("In Joystick Test State");
}


void KEStandByState::entryCode(void) {
    spdlog::debug("In StandBy State");
    logHelperSB.initLogger("KEStandByStateLog", "logs/Standby.csv", LogFormat::CSV, true);
    logHelperSB.add(running(), "Time (s)");
    logHelperSB.add(robot->getPosition(), "q");
    logHelperSB.add(robot->getVelocity(), "dq");
    logHelperSB.add(robot->getTorque(), "tau");
    logHelperSB.add(robot->getSpringPosition(), "springQ");
    spdlog::debug("In StandBy State: logger created");

    if(logHelperSB.isInitialised()) {
            logHelperSB.startLogger();
        }
    spdlog::debug("In StandBy State: logger started");

}
void KEStandByState::duringCode(void) {
    if (robot->keyboard->getQ()==1){
        spdlog::debug("Key pressed: Q");
    }
    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }
}
void KEStandByState::exitCode(void) {
        logHelperSB.endLog();
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
    tau = -12;
    robot->setJointTorque(tau);
    if(iterations()%100==1) {
        std::cout << "step 1 ..." << std::flush;
    }
}

void KECalibState::exitCode(void) {
    calib_step = 1;
    robot->applyCalibrationSpring();
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
        if(iterations()%10==1) {
            robot->setJointPosition(caliStep1MotorPosScalar + (CALIB_ANGLE_1+abs(tau)/5.44)*M_PI/180.);
            targetPosStep2Set++;
        }
    }

    if(abs(motorVel[KNEE])<0.01) {
        stop_reached_time += dt();
    }
    if(stop_reached_time > 1.) {
        Eigen::VectorXd& springPos=robot->getSpringPosition();
        calib_step = 2;
        robot->applyCalibrationMotor(springPos[KNEE]);

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
    robot->applyCalibrationSpring();
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    robot->applyCalibrationMotor(springPos[KNEE]);


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
    robot->applyCalibrationSpring();
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    robot->applyCalibrationMotor(springPos[KNEE]);

    robot->initCyclicVelocityControl(controlMotorProfile);
    usleep(10000);
    double step2_t = 0;
    spdlog::debug("Vel control init done");

}
void KEVelControlDemo::duringCode(void) {

    // Calcualte joint reference postion
    step2_t += dt();
    targetMotorVel = 200.*sin(1*step2_t) * M_PI / 180.0;

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
    robot->applyCalibrationSpring();
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    robot->applyCalibrationMotor(springPos[KNEE]);

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

    //const char *const gpio_estop_pin = "/sys/class/gpio/PQ.05/value";
    //std::ifstream amp(gpio_estop_pin);
    //if (amp.is_open())
    //{
    //    int gpio_estop_status; // or perhaps a string?
    //    amp >> gpio_estop_status;
    //    spdlog::debug("gpio_estop_status: {}", gpio_estop_status);
   // }
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
    robot->applyCalibrationSpring();
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    robot->applyCalibrationMotor(springPos[KNEE]);

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


/*=============================
Calibration Exercise Robot Drive
=============================*/
void KECalibExerRobDriveState::entryCode(void) {
    robot->stopSensorStreaming();
    usleep(10000);
    robot->startSensorStreaming();
    robot->sensorCalibration();

    robot->initCyclicPositionControl(controlMotorProfile);
    usleep(1000);
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    targetMotorPosFilt = springPos[KNEE];
    robot->setJointPosition(targetMotorPos);
    action_t = 0.;
}
void KECalibExerRobDriveState::duringCode(void) {

    Eigen::VectorXd& springPos=robot->getSpringPosition();
    Eigen::VectorXd& motorPos=robot->getPosition();

    const char *const gpio_button_pin = "/sys/class/gpio/PAC.06/value";  // user button
    std::ifstream button_gpio_file(gpio_button_pin);
    if (button_gpio_file.is_open())
    {
        int gpio_button_status;
        button_gpio_file >> gpio_button_status;
        if (gpio_button_status == 0 && user_butt_pressed == false){
            user_butt_time += dt();
            if (user_butt_time > 0.1) {user_butt_pressed = true; user_butt_time = 0.0;}
        }
        else if (gpio_button_status == 0 && user_butt_pressed == true && button_released == true) 
        {actionFlag = 1 - actionFlag; action_t = 0.; user_butt_pressed = false; button_released = false;}
        else if (gpio_button_status == 1) {button_released = true;}
        else ;
    }

    if (robot->keyboard->getW()==1) angleFlag = 1 - angleFlag;  // toggle between 1 and 2
    if (robot->keyboard->getD()==1) emgFlag = 1 - emgFlag;  // toggle between 1 and 2
    testCondition = angleText[angleFlag] + emgText[emgFlag];

    if (robot->keyboard->getA()==1) initLoggerStandard(testCondition);  // toggle between 1 and 2
    if (robot->keyboard->getS()==1) logHelperSB.endLog();  // toggle between 1 and 2

    double kneeAngle = springPos[KNEE]*180./M_PI;
    double kneeAngleMot = motorPos[KNEE]*180./M_PI;

    if (actionFlag == false) {targetSpringTor = 0;}
    else if (action_t < test_duration) {
        action_t += dt();
        targetSpringTor = 20.*sin(0.2*2*M_PI*action_t);
    }
    else {targetSpringTor = 0; actionFlag=false;}

    targetMotorPos = springPos[KNEE] + targetSpringTor / springK;
    targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);

    // Set position
    robot->setJointPosition(targetMotorPosFilt);

    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    std::time_t result = std::time(nullptr);

    if(iterations()%5==1) {
        //std::cout << "action Flag: " << actionFlag << "action_time: " << action_t << std::endl;
        //std::cout << "targetMotorPos: " << targetMotorPos*180./M_PI << "  targetMotorPosFilt: " << targetMotorPosFilt*180./M_PI << std::endl;
        //std::cout << "Knee Angle: " << kneeAngle << "  Knee Angle Mot: " << kneeAngleMot << std::endl;
        //std::cout << "Target Torque: " << targetSpringTor << std::endl;
        //std::cout << std::asctime(std::localtime(&result));
        if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
            std::cout << "RECORDING ";
        }
        std::cout << "Calib Rob Drive " << angleText[angleFlag] << ", " << emgText[emgFlag] << ", "
          << actionText[actionFlag] << " Time: 0s ||";
        for (int i=1; i<=50; i++) {
            if (action_t*10 < i) std::cout << "+";
            else std::cout << "-";
        }
        std::cout << "|| 5s" << std::endl;
    }
}
void KECalibExerRobDriveState::exitCode(void) {
    ;
    //Eigen::VectorXd& springPos=robot->getSpringPosition();
    //double targetMotorPos = springPos[KNEE];
    //robot->setJointPosition(targetMotorPos);
    //robot->initProfilePositionControl(controlMotorProfile);
    //usleep(10000);
    //robot->setPosition(std::vector<double> 0.0);
}

/*=============================
Calibration Exercise Human Drive - Robot in Viscous field
=============================*/
void KECalibExerHumDriveState::entryCode(void) {
        robot->stopSensorStreaming();
    usleep(10000);
    robot->startSensorStreaming();
    robot->sensorCalibration();
    //robot->initCyclicPositionControl(controlMotorProfile);
    //usleep(100);
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    targetMotorPosFilt = springPos[KNEE];
    robot->setJointPosition(targetMotorPos);
    action_t = 0.;
}
void KECalibExerHumDriveState::duringCode(void) {

    Eigen::VectorXd& springPos=robot->getSpringPosition();
    Eigen::VectorXd& motorPos=robot->getPosition();
    Eigen::VectorXd& springVelVec=robot->getSpringVelocity();

    const char *const gpio_button_pin = "/sys/class/gpio/PAC.06/value";  // user button
    std::ifstream button_gpio_file(gpio_button_pin);
    if (button_gpio_file.is_open())
    {
        int gpio_button_status;
        button_gpio_file >> gpio_button_status;
        if (gpio_button_status == 0 && user_butt_pressed == false){
            user_butt_time += dt();
            if (user_butt_time > 0.1) {user_butt_pressed = true; user_butt_time = 0.0;}
        }
        else if (gpio_button_status == 0 && user_butt_pressed == true && button_released == true) 
        {actionFlag = 1 - actionFlag; action_t = 0.; user_butt_pressed = false; button_released = false;}
        else if (gpio_button_status == 1) {button_released = true;}
        else ;
    }

    if (robot->keyboard->getW()==1) {
        resistanceFlag++; 
        if (resistanceFlag >= 5) resistanceFlag = 1;  // toggle between 1,2,3,4
    }
    testCondition = resistanceText[resistanceFlag-1];

    if (robot->keyboard->getA()==1) initLoggerStandard(testCondition);  // toggle between 1 and 2
    if (robot->keyboard->getS()==1) logHelperSB.endLog();  // toggle between 1 and 2

    double kneeAngle = springPos[KNEE]*180./M_PI;
    double kneeAngleMot = motorPos[KNEE]*180./M_PI;

    //springVelFilt = springVelFilt + (1.0-decay) * (springVelVec[KNEE] - springVelFilt);
    if (actionFlag == false) {targetSpringTor = 0;}
    else {
        if (inFlexing == 1 && kneeAngle < -78) inFlexing = 0;
        else if (inFlexing == 0 && kneeAngle > -20) inFlexing = 1;

        if (inFlexing == 1) targetSpringTor = 5 * resistanceFlag;
        else targetSpringTor = -5 * resistanceFlag;
    }

    targetTorFilt = targetTorFilt + 0.03 * (targetSpringTor - targetTorFilt);  // smooth the torque in transition
    targetMotorPos = springPos[KNEE] + targetTorFilt / springK;
    targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);

    // Set position
    robot->setJointPosition(targetMotorPosFilt);

    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    if(iterations()%5==1) {
        //std::cout << "action Flag: " << actionFlag << "action_time: " << action_t << std::endl;
        //std::cout << "springVel: " << springVel << "  targetMotorPosFilt: " << targetMotorPosFilt*180./M_PI << std::endl;
        //std::cout << "Knee Angle: " << kneeAngle << "  Knee Angle Mot: " << kneeAngleMot << std::endl;
        //std::cout << "Target Torque: " << targetTorFilt << std::endl;
        //std::cout << "Mode: " << modeText[inFlexing] << "||";
        if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
            std::cout << "RECORDING ";
        }
        std::cout << "Calib Hum Drive " << resistanceText[resistanceFlag-1] << ", " << modeText[inFlexing] << ", "
          << actionText[actionFlag] << " -78deg ||";
        if (inFlexing) {
            for (int i=-78; i<-18; i++) {
                if (kneeAngle > i) std::cout << "-";
                else std::cout << "+";
            }
        }
        else {
            std::cout << "|";
            for (int i=-78; i<-18; i++) {
                if (kneeAngle > i) std::cout << "+";
                else std::cout << "-";
            }
        }
        std::cout << "|| -18deg" << std::endl;
    }
}
void KECalibExerHumDriveState::exitCode(void) {
    ;
    //robot->initProfilePositionControl(controlMotorProfile);
    //usleep(10000);
    //robot->setPosition(std::vector<double> 0.0);
}

/*=============================
Actual Exercise Robot in Charge - Robot in Position Control
=============================*/
void KEActualExerRobDriveState::entryCode(void) {
    robot->stopSensorStreaming();
    usleep(10000);
    robot->startSensorStreaming();
    robot->sensorCalibration();
    //robot->initProfilePositionControl(controlMotorProfile);
    //usleep(1000);
    //robot->initCyclicPositionControl(controlMotorProfile);
    //usleep(1000);
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    targetMotorPosFilt = springPos[KNEE];
    robot->setJointPosition(targetMotorPos);
    action_t = 0.;
}
void KEActualExerRobDriveState::duringCode(void) {

    Eigen::VectorXd& springPos=robot->getSpringPosition();
    Eigen::VectorXd& motorPos=robot->getPosition();
    Eigen::VectorXd& springVelVec=robot->getSpringVelocity();

    const char *const gpio_button_pin = "/sys/class/gpio/PAC.06/value";  // user button
    std::ifstream button_gpio_file(gpio_button_pin);
    if (button_gpio_file.is_open())
    {
        int gpio_button_status;
        button_gpio_file >> gpio_button_status;
        if (gpio_button_status == 0 && user_butt_pressed == false){
            user_butt_time += dt();
            if (user_butt_time > 0.1) {user_butt_pressed = true; user_butt_time = 0.0;}
        }
        else if (gpio_button_status == 0 && user_butt_pressed == true && button_released == true) 
        {actionFlag = 1 - actionFlag; action_t = 0.; user_butt_pressed = false; button_released = false;}
        else if (gpio_button_status == 1) {button_released = true;}
        else ;
    }

    if (robot->keyboard->getW()==1) {
        resistanceFlag++; 
        if (resistanceFlag >= 4) resistanceFlag = 0;  // toggle between 1,2,3,4
    }
    if (robot->keyboard->getD()==1) {
        freqFlag = 1 - freqFlag;  // toggle between 0 and 1
    }
    
    test_duration = 5 / sineFreq;  // 5 cycles
    sineFreq = freqTable[freqFlag];
    testCondition = freqText[freqFlag] + resistanceText[resistanceFlag];

    if (robot->keyboard->getA()==1) initLoggerStandard(testCondition);  // toggle between 1 and 2
    if (robot->keyboard->getS()==1) logHelperSB.endLog();  // toggle between 1 and 2

    double kneeAngle = springPos[KNEE]*180./M_PI;
    double kneeAngleMot = motorPos[KNEE]*180./M_PI;

    //springVelFilt = springVelFilt + (1.0-decay) * (springVelVec[KNEE] - springVelFilt);
    if (actionFlag == false) {
        double tarTorq = -78. - kneeAngle;  // go to 78 with propotional torque control
        tarTorq = std::min(std::max(tarTorq, -10.), 10.);
        targetTorFilt = targetTorFilt + 0.5 * (tarTorq - targetTorFilt);  // smooth the torque in transition
        targetMotorPos = springPos[KNEE] + targetTorFilt / springK;
    }
    else if (action_t < test_duration) {
        action_t += dt();
        targetMotorPos = (-48-30*cos(2*M_PI*sineFreq*action_t)) * M_PI / 180;
    }
    else targetMotorPos = -78.* M_PI / 180;


    targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);

    // Set position
    robot->setJointPosition(targetMotorPosFilt);
    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    double measuredTorq =  (motorPos[KNEE] - springPos[KNEE]) * springK;

    if(iterations()%5==1) {
        //std::cout << "action Flag: " << actionFlag << "action_time: " << action_t << std::endl;
        //std::cout << "test_duration: " << test_duration << "  action_t: " << action_t << std::endl;
        //std::cout << "Knee Angle: " << kneeAngle << "  Knee Angle Mot: " << kneeAngleMot << std::endl;
        //std::cout << "Target Torque: " << targetTorFilt << std::endl;
        if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
            std::cout << "RECORDING ";
        }
        std::cout << "Exercise Robot in Charge " << freqText[freqFlag] << ", " << resistanceText[resistanceFlag] << ", "
          << actionText[actionFlag] << "Torque -20Nm ||";
        if (measuredTorq > 0) {
            for (int i=40; i>0; i--) {
                std::cout << "-";
            }
            std::cout << "|";
            for (int i=0; i<40; i++) {
                if ((measuredTorq*2) > i) std::cout << "+";
                else std::cout << "-";
            }
        }
        else {
            for (int i=-40; i<0; i++) {
                if ((measuredTorq*2) > i) std::cout << "-";
                else std::cout << "+";
            }
            std::cout << "|";
            for (int i=40; i>0; i--) {
                std::cout << "-";
            }
        }
        std::cout << "|| 20Nm" << std::endl;

    }
}
void KEActualExerRobDriveState::exitCode(void) {
    ;
    //robot->setPosition(std::vector<double> 0.0);
}


/*=============================
Actual Exercise Sit to Stand
=============================*/
void KETorCtrlSit2Stand::entryCode(void) {
        robot->stopSensorStreaming();
    usleep(10000);
    robot->startSensorStreaming();
    robot->sensorCalibration();
    Eigen::VectorXd& springPos=robot->getSpringPosition();
    targetMotorPosFilt = springPos[KNEE];
    robot->setJointPosition(targetMotorPos);
    action_t = 0.;
}
void KETorCtrlSit2Stand::duringCode(void) {

    Eigen::VectorXd& springPos=robot->getSpringPosition();
    Eigen::VectorXd& motorPos=robot->getPosition();

    const char *const gpio_button_pin = "/sys/class/gpio/PAC.06/value";  // user button
    std::ifstream button_gpio_file(gpio_button_pin);
    if (button_gpio_file.is_open())
    {
        int gpio_button_status;
        button_gpio_file >> gpio_button_status;
        if (gpio_button_status == 0 && user_butt_pressed == false){
            user_butt_time += dt();
            if (user_butt_time > 0.1) {user_butt_pressed = true; user_butt_time = 0.0;}
        }
        else if (gpio_button_status == 0 && user_butt_pressed == true && button_released == true) 
        {actionFlag = 1 - actionFlag; action_t = 0.; user_butt_pressed = false; button_released = false;}
        else if (gpio_button_status == 1) {button_released = true;}
        else ;
    }

    if (robot->keyboard->getW()==1) {
        assistanceFlag++; 
        if (assistanceFlag >= 2) assistanceFlag = 0;  // toggle between 1,2,3,4
    }
    if (robot->keyboard->getA()==1) initLoggerStandard(testCondition);  // toggle between 1 and 2
    if (robot->keyboard->getS()==1) logHelperSB.endLog();  // toggle between 1 and 2


    int i = 0;
    int arrayLen = sit2stdAngSmp.size();
    double t = 0.;

    double kneeAngle = springPos[KNEE]*180./M_PI;
    double kneeAngleMot = motorPos[KNEE]*180./M_PI;

    if (actionFlag == true) {
        while (i < arrayLen - 2 && sit2stdAngSmp[i+1] < kneeAngle) {
            i++;
        }

            // Linear interpolation formula
        if (kneeAngle >= sit2stdAngSmp[arrayLen-1]) {
            targetSpringTor = sit2stdTorProfile_40[arrayLen-1];
        }
        else if (kneeAngle <= sit2stdAngSmp[0]) {
            targetSpringTor = sit2stdTorProfile_40[0];
        }
        else {
            t = (kneeAngle - sit2stdAngSmp[i]) / (sit2stdAngSmp[i+1] - sit2stdAngSmp[i]);
            targetSpringTor = sit2stdTorProfile_40[i] + t * (sit2stdTorProfile_40[i+1] - sit2stdTorProfile_40[i]);
        }
        targetSpringTor = targetSpringTor * 3.6 * assistanceLevel[assistanceFlag];  // change to body weight x assistance level
    }
    else {
        targetSpringTor = 0;
    }

    targetTorFilt = targetTorFilt + 0.2 * (targetSpringTor - targetTorFilt);  // smooth the torque in transition
    targetMotorPos = springPos[KNEE] + targetTorFilt / springK;
    targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);

    // Set position
    robot->setJointPosition(targetMotorPosFilt);
    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    if(iterations()%10==1) {
        //std::cout << "Stand flag : " << actionFlag << std::endl;
        //std::cout << "targetMotorPos: " << targetMotorPos*180./M_PI << "  targetMotorPosFilt: " << targetMotorPosFilt*180./M_PI << std::endl;
        //std::cout << "Knee Angle: " << kneeAngle << "  Knee Angle Mot: " << kneeAngleMot << std::endl;
        //std::cout << "Target Torque: " << targetSpringTor << std::endl;
        if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
            std::cout << "RECORDING ";
        }
        std::cout << "Exercise STS " << assistanceText[assistanceFlag] << ", " << actionText[actionFlag] << ", "
          << actionText[actionFlag] << " -110deg ||";
        for (int i=-110; i<0; i++) {
            if (kneeAngle > i) std::cout << "+";
            else std::cout << "-";
        }
        std::cout << "|| 0deg" << std::endl;
    }
}
void KETorCtrlSit2Stand::exitCode(void) {
    logHelperSB.endLog();
    //robot->setPosition(std::vector<double> 0.0);
}

void KECalibExerRobDriveState::initLoggerStandard(std::string testConsiditon) {
    logger_count++;
    std::time_t raw_time_now = std::time(nullptr);
    struct tm* time_info = std::localtime(&raw_time_now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%m_%d_%H_%M_%S", time_info);
    std::string time_now(buffer);

    std::string filename = "logs/CalibExerRobDrive_" + testConsiditon + "_test" + std::to_string(logger_count) + "_" + time_now + ".csv";
    logHelperSB.initLogger("KECalibExerRobDriveStateLog", filename, LogFormat::CSV, true);
    logHelperSB.add(running(), "Time (s)");
    logHelperSB.add(actionFlag, "actionFlag");
    logHelperSB.add(robot->getPosition(), "MotorPos");
    logHelperSB.add(robot->getVelocity(), "MotorVel");
    logHelperSB.add(robot->getTorque(), "MotorTorq");
    logHelperSB.add(robot->getSpringPosition(), "springPos");
    logHelperSB.add(robot->getSpringVelocity(), "springVel");
    logHelperSB.add(robot->getForces(1, 1), "Thigh_Forces");
    logHelperSB.add(robot->getForces(1, 2), "Thigh_Torques");
    logHelperSB.add(robot->getForces(2, 1), "Shank_Forces");
    logHelperSB.add(robot->getForces(2, 2), "Shank_Torques");

    // for teensy 1
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ACCL), "THIGH_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_GYRO), "THIGH_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ORIEN), "THIGH_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ACCL), "THIGH_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_GYRO), "THIGH_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ORIEN), "THIGH_IMU1_OREN");
    // for teensy 2
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ACCL), "SHANK_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_GYRO), "SHANK_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ORIEN), "SHANK_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ACCL), "SHANK_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_GYRO), "SHANK_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ORIEN), "SHANK_IMU1_OREN");

    spdlog::debug("In Calibration Exercise Robot Drive State: logger created");

    if(logHelperSB.isInitialised()) {
            logHelperSB.startLogger();
        }
    spdlog::debug("In Calibration Exercise Robot Drive State State: logger started");
}

void KECalibExerHumDriveState::initLoggerStandard(std::string testConsiditon) {
    logger_count++;
    std::time_t raw_time_now = std::time(nullptr);
    struct tm* time_info = std::localtime(&raw_time_now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%m_%d_%H_%M_%S", time_info);
    std::string time_now(buffer);

    std::string filename= "logs/CalibExerHumDrive_" + testConsiditon + "_test" + std::to_string(logger_count) + "_" + time_now + ".csv";
    logHelperSB.initLogger("KECalibExerHumDriveStateLog", filename, LogFormat::CSV, true);
    logHelperSB.add(running(), "Time (s)");
    logHelperSB.add(actionFlag, "actionFlag");
    logHelperSB.add(robot->getPosition(), "MotorPos");
    logHelperSB.add(robot->getVelocity(), "MotorVel");
    logHelperSB.add(robot->getTorque(), "MotorTorq");
    logHelperSB.add(robot->getSpringPosition(), "springPos");
    logHelperSB.add(robot->getSpringVelocity(), "springVel");
    logHelperSB.add(robot->getForces(1, 1), "Thigh_Forces");
    logHelperSB.add(robot->getForces(1, 2), "Thigh_Torques");
    logHelperSB.add(robot->getForces(2, 1), "Shank_Forces");
    logHelperSB.add(robot->getForces(2, 2), "Shank_Torques");
    // for teensy 1
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ACCL), "THIGH_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_GYRO), "THIGH_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ORIEN), "THIGH_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ACCL), "THIGH_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_GYRO), "THIGH_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ORIEN), "THIGH_IMU1_OREN");
    // for teensy 2
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ACCL), "SHANK_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_GYRO), "SHANK_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ORIEN), "SHANK_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ACCL), "SHANK_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_GYRO), "SHANK_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ORIEN), "SHANK_IMU1_OREN");
    spdlog::debug("In Calibration Exercise Human Drive State: logger created");

    if(logHelperSB.isInitialised()) {
            logHelperSB.startLogger();
        }
    spdlog::debug("In Calibration Exercise Human Drive State: logger started");
}

void KEActualExerRobDriveState::initLoggerStandard(std::string testConsiditon) {
    logger_count++;
    std::time_t raw_time_now = std::time(nullptr);
    struct tm* time_info = std::localtime(&raw_time_now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%m_%d_%H_%M_%S", time_info);
    std::string time_now(buffer);

    std::string filename= "logs/ActualExerRobDrive_" + testConsiditon + "_test" + std::to_string(logger_count) + "_" + time_now + ".csv";
    logHelperSB.initLogger("KEActualExerRobDriveState", filename, LogFormat::CSV, true);
    logHelperSB.add(running(), "Time (s)");
    logHelperSB.add(actionFlag, "actionFlag");
    logHelperSB.add(robot->getPosition(), "MotorPos");
    logHelperSB.add(robot->getVelocity(), "MotorVel");
    logHelperSB.add(robot->getTorque(), "MotorTorq");
    logHelperSB.add(robot->getSpringPosition(), "springPos");
    logHelperSB.add(robot->getSpringVelocity(), "springVel");
    logHelperSB.add(robot->getForces(1, 1), "Thigh_Forces");
    logHelperSB.add(robot->getForces(1, 2), "Thigh_Torques");
    logHelperSB.add(robot->getForces(2, 1), "Shank_Forces");
    logHelperSB.add(robot->getForces(2, 2), "Shank_Torques");
    // for teensy 1
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ACCL), "THIGH_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_GYRO), "THIGH_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ORIEN), "THIGH_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ACCL), "THIGH_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_GYRO), "THIGH_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ORIEN), "THIGH_IMU1_OREN");
    // for teensy 2
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ACCL), "SHANK_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_GYRO), "SHANK_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ORIEN), "SHANK_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ACCL), "SHANK_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_GYRO), "SHANK_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ORIEN), "SHANK_IMU1_OREN");

    spdlog::debug("In Actual Exercise Robot in Charge State: logger created");

    if(logHelperSB.isInitialised()) {
            logHelperSB.startLogger();
        }
    spdlog::debug("In Actual Exercise Robot in Charge State: logger started");
}

void KETorCtrlSit2Stand::initLoggerStandard(std::string testConsiditon) {
    logger_count++;
    std::time_t raw_time_now = std::time(nullptr);
    struct tm* time_info = std::localtime(&raw_time_now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%m_%d_%H_%M_%S", time_info);
    std::string time_now(buffer);

    std::string filename= "logs/ActualExerSTS_" + testConsiditon + "_test" + std::to_string(logger_count) + "_" + time_now + ".csv";
    logHelperSB.initLogger("KETorCtrlSit2Stand", filename, LogFormat::CSV, true);
    logHelperSB.add(running(), "Time (s)");
    logHelperSB.add(actionFlag, "actionFlag");
    logHelperSB.add(robot->getPosition(), "MotorPos");
    logHelperSB.add(robot->getVelocity(), "MotorVel");
    logHelperSB.add(robot->getTorque(), "MotorTorq");
    logHelperSB.add(robot->getSpringPosition(), "springPos");
    logHelperSB.add(robot->getSpringVelocity(), "springVel");
    logHelperSB.add(robot->getForces(1, 1), "Thigh_Forces");
    logHelperSB.add(robot->getForces(1, 2), "Thigh_Torques");
    logHelperSB.add(robot->getForces(2, 1), "Shank_Forces");
    logHelperSB.add(robot->getForces(2, 2), "Shank_Torques");
    // for teensy 1
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ACCL), "THIGH_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_GYRO), "THIGH_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID0, IMU_ORIEN), "THIGH_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ACCL), "THIGH_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_GYRO), "THIGH_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy1, IMU_ID1, IMU_ORIEN), "THIGH_IMU1_OREN");
    // for teensy 2
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ACCL), "SHANK_IMU0_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_GYRO), "SHANK_IMU0_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID0, IMU_ORIEN), "SHANK_IMU0_OREN");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ACCL), "SHANK_IMU1_ACCL");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_GYRO), "SHANK_IMU1_GYRO");
    logHelperSB.add(robot->getTeensyData(robot->tsy2, IMU_ID1, IMU_ORIEN), "SHANK_IMU1_OREN");
    spdlog::debug("In Actual Exercise STS State: logger created");

    if(logHelperSB.isInitialised()) {
            logHelperSB.startLogger();
        }
    spdlog::debug("In Actual Exercise STS State: logger started");
}