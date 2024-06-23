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

#define THIGH_FT 1
#define SHANK_FT 2
#define FT_FORCE 1
#define FT_TORQUE 2

#define MAX_TORQUE_LEVEL 24
#define UPPER_ANGLE_LIMIT -23
#define LOWER_ANGLE_LIMIT -73
#define DISPLAY_DOWNSAMPLE_FACTOR 10
#define PROGRESS_BAR_WIDTH 50

#define KEYCODE_J 106
#define KEYCODE_K 107

#define POS_INCRE_MAX 0.4  //  higher than this will cause overvoltage, 0.4 is roughly 4000inc at the motor with 144 ratio and 4096 motor encoder resolution

#define CALIB_ANGLE_1 110

using namespace std;

double motorPosForTrans = 0.;


void saveValues(float values[], int num_values, const std::string& filename) {
    std::cout << "saving parameters to file..." << std::endl;
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        for (int i = 0; i < num_values; i++) {
            if (i == num_values - 1) {
                outFile << values[i];
            } else {
                outFile << values[i] << ",";
            }
        }
        outFile << std::endl;
        outFile.close();
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
    std::cout << "saving parameters to file finished." << std::endl;
}

void sav_temp_params_to_file(RobotKE* robot) {
    int num_params = 14;
    float temp_vals[num_params] = {0};

    Eigen::VectorXd& motorPos = robot->getPosition();
    Eigen::VectorXd& springPos= robot->getSpringPosition();
    temp_vals[0] = motorPos[KNEE];
    temp_vals[1] = springPos[KNEE];

    float ft1_forceOffsets[3] = {0};
    float ft1_torqueOffsets[3] = {0};
    float ft2_forceOffsets[3] = {0};
    float ft2_torqueOffsets[3] = {0};

    robot->ftsensor1->getOffsets(ft1_forceOffsets, ft1_torqueOffsets);
    robot->ftsensor2->getOffsets(ft2_forceOffsets, ft2_torqueOffsets);

    temp_vals[2] = ft1_forceOffsets[0];
    temp_vals[3] = ft1_forceOffsets[1];
    temp_vals[4] = ft1_forceOffsets[2];
    temp_vals[5] = ft1_torqueOffsets[0];
    temp_vals[6] = ft1_torqueOffsets[1];
    temp_vals[7] = ft1_torqueOffsets[2];
    temp_vals[8] = ft2_forceOffsets[0];
    temp_vals[9] = ft2_forceOffsets[1];
    temp_vals[10] = ft2_forceOffsets[2];
    temp_vals[11] = ft2_torqueOffsets[0];
    temp_vals[12] = ft2_torqueOffsets[1];
    temp_vals[13] = ft2_torqueOffsets[2];

    std::string filename = "logs/onRunningParams.csv";
    saveValues(temp_vals, num_params, filename);
}

double get_mean(double arr[], int n){
      double sum = 0;
      for (int i = 0; i < n; i++){
         sum += arr[i];
      }
      return sum / n;
}

void draw_progress_bar(std::string left_text, std::string right_text, float value, float value_min, float value_max, int bar_width) {
    std::cout << left_text;
    int progress = (value * (float)bar_width) / (value_max - value_min);
    for (int i = 1; i <= bar_width; i++) {
        if (i <= progress) std::cout << "+";
        else std::cout << "-";
    }
    std::cout << right_text<< std::endl;
}


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
    
    controlMotorProfile.profileVelocity = 600;
    robot->initProfilePositionControl(controlMotorProfile);
    std::cout << "Calibrating (keep clear)..." << std::flush;

}
//Move slowly on each joint until max force detected
void KECalibState2::duringCode(void) {

    //Apply constant torque (with damping) unless stop has been detected for more than 0.5s
    //Eigen::VectorXd& springPos=robot->getSpringPosition();
    //spdlog::debug("SpringPos: {}", springPos[KNEE]);
    Eigen::VectorXd& motorVel = robot->getVelocity();

    if (targetPosStep2Set <= 4) {
        if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
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

void KEZeroLevelForce::entryCode(void) {
    robot->startSensorStreaming();
    std::cout << "Entering Zero level force state, sit with leg straight..." << std::endl;
}
//Move slowly on each joint until max force detected
void KEZeroLevelForce::duringCode(void) {
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
        {actionFlag = 1 - actionFlag; user_butt_pressed = false; button_released = false;}
        else if (gpio_button_status == 1) {button_released = true;}
        else ;
    }


    if (actionFlag == true) {
        Eigen::VectorXd& shankfForces=robot->getForces(SHANK_FT, FT_FORCE);
        Eigen::VectorXd& shankTorques=robot->getForces(SHANK_FT, FT_TORQUE);
        Eigen::VectorXd& thighfForces=robot->getForces(THIGH_FT, FT_FORCE);
        Eigen::VectorXd& thighTorques=robot->getForces(THIGH_FT, FT_TORQUE);

        thigh_Fx_recording[recording_count] = thighfForces[0];
        thigh_Fy_recording[recording_count] = thighfForces[1];
        thigh_Fz_recording[recording_count] = thighfForces[2];
        thigh_Mx_recording[recording_count] = thighTorques[0];
        thigh_My_recording[recording_count] = thighTorques[1];
        thigh_Mz_recording[recording_count] = thighTorques[2];

        shank_Fx_recording[recording_count] = shankfForces[0];
        shank_Fy_recording[recording_count] = shankfForces[1];
        shank_Fz_recording[recording_count] = shankfForces[2];
        shank_Mx_recording[recording_count] = shankTorques[0];
        shank_My_recording[recording_count] = shankTorques[1];
        shank_Mz_recording[recording_count] = shankTorques[2];

        //std::cout << "Thigh Force: " << thighfForces[0] << " Shank Force: " << shankfForces[0] << std::endl;
        if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
            std::cout << "Zero leveling..." << std::endl;
        }

        recording_count++;
        if (recording_count == 100) {
            actionFlag = false;
            zeroleveled = false;
        }
    }
    if (actionFlag == false) {
        if (zeroleveled == false) {
            thigh_Fx_Calib = get_mean(thigh_Fx_recording, recording_count) + thigh_Fx_Init;
            thigh_Fy_Calib = get_mean(thigh_Fy_recording, recording_count) + thigh_Fy_Init;
            thigh_Fz_Calib = get_mean(thigh_Fz_recording, recording_count) + thigh_Fz_Init;
            thigh_Mx_Calib = get_mean(thigh_Mx_recording, recording_count) + thigh_Mx_Init;
            thigh_My_Calib = get_mean(thigh_My_recording, recording_count) + thigh_My_Init;
            thigh_Mz_Calib = get_mean(thigh_Mz_recording, recording_count) + thigh_Mz_Init;

            shank_Fx_Calib = get_mean(shank_Fx_recording, recording_count) + shank_Fx_Init;
            shank_Fy_Calib = get_mean(shank_Fy_recording, recording_count) + shank_Fy_Init;
            shank_Fz_Calib = get_mean(shank_Fz_recording, recording_count) + shank_Fz_Init;
            shank_Mx_Calib = get_mean(shank_Mx_recording, recording_count) + shank_Mx_Init;
            shank_My_Calib = get_mean(shank_My_recording, recording_count) + shank_My_Init;
            shank_Mz_Calib = get_mean(shank_Mz_recording, recording_count) + shank_Mz_Init;

            thigh_force_offsets << thigh_Fx_Calib, thigh_Fy_Calib, thigh_Fz_Calib;
            thigh_torque_offsets << thigh_Mx_Calib, thigh_My_Calib, thigh_Mz_Calib;
            shank_force_offsets << shank_Fx_Calib, shank_Fy_Calib, shank_Fz_Calib;
            shank_torque_offsets << shank_Mx_Calib, shank_My_Calib, shank_Mz_Calib;

            robot->ftsensor1->setOffsets(thigh_force_offsets, thigh_torque_offsets);
            robot->ftsensor2->setOffsets(shank_force_offsets, shank_torque_offsets);
            recording_count = 0;
            std::cout << "Zero Level Finished." << std::endl;  
            zeroleveled = true;  
        }
        else {
            if(iterations()%100==1) {
                std::cout << "Stand with leg straight to get ready for force sensor zero level..." << std::endl;
            }
        }
    }
}

void KEZeroLevelForce::exitCode(void) {
    //robot->setEndEffForceWithCompensation(VM3(0,0,0));
    Eigen::VectorXd& motorPos=robot->getPosition();
    motorPosForTrans = motorPos[KNEE];
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

    if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
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

        double targetMotorPosFiltTemp = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    targetMotorPosFilt = targetMotorPosFilt + std::min(std::max(targetMotorPosFiltTemp - targetMotorPosFilt, -POS_INCRE_MAX), POS_INCRE_MAX);
    //targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);
        //spdlog::debug("motor pos: {}, spring pos: {}", targetMotorPosFilt, springPos[KNEE]);


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
    //robot->stopSensorStreaming();
    //usleep(10000);
    //robot->startSensorStreaming();
    //robot->sensorCalibration();

    robot->initCyclicPositionControl(controlMotorProfile);
    usleep(1000);
    targetMotorPosFilt = motorPosForTrans;
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
    if (robot->keyboard->getD()==1) {
        emgFlag++; 
        if (emgFlag >= 4) emgFlag = 0;  // toggle between 0,1,2,3
    }
    if (robot->keyboard->getKeyCode() == KEYCODE_J) {
        sav_temp_params_to_file(robot);
    }


    testCondition = angleText[angleFlag] + emgText[emgFlag];

    if (robot->keyboard->getA()==1) initLoggerStandard(testCondition);  // toggle between 1 and 2
    if (robot->keyboard->getS()==1) logHelperSB.endLog();  // toggle between 1 and 2

    double kneeAngle = springPos[KNEE]*180./M_PI;
    double kneeAngleMot = motorPos[KNEE]*180./M_PI;

    if (actionFlag == false) {targetSpringTor = 0;}
    else if (action_t < test_duration) {
        action_t += dt();
        targetSpringTor = MAX_TORQUE_LEVEL*sin(0.2*2*M_PI*action_t);
    }
    else {targetSpringTor = 0; actionFlag=false;}

    targetMotorPos = springPos[KNEE] + targetSpringTor / springK;

    double targetMotorPosFiltTemp = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    targetMotorPosFilt = targetMotorPosFilt + std::min(std::max(targetMotorPosFiltTemp - targetMotorPosFilt, -POS_INCRE_MAX), POS_INCRE_MAX);
    //targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", targetMotorPosFilt, springPos[KNEE]);

    // Set position
    robot->setJointPosition(targetMotorPosFilt);

    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    std::time_t result = std::time(nullptr);

    if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
        if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
            std::cout << "RECORDING ";
        }
        std::string left_str = "Calib Rob Drive " + angleText[angleFlag] + ", " + emgText[emgFlag] + ", " + actionText[actionFlag] + " Time: 0s ||";
        std::string right_str = "|| 5s";
        float text_duration = 5;
        draw_progress_bar(left_str, right_str, action_t, 0, test_duration, PROGRESS_BAR_WIDTH);
    }
}
void KECalibExerRobDriveState::exitCode(void) {
    Eigen::VectorXd& motorPos=robot->getPosition();
    motorPosForTrans = motorPos[KNEE];
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
    //robot->stopSensorStreaming();
    //usleep(10000);
    //robot->startSensorStreaming();
    //robot->sensorCalibration();
    //robot->initCyclicPositionControl(controlMotorProfile);
    //usleep(100);
    targetMotorPosFilt = motorPosForTrans;

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
        else targetSpringTor = -MAX_TORQUE_LEVEL/4. * resistanceFlag;
    }

    targetTorFilt = targetTorFilt + 0.03 * (targetSpringTor - targetTorFilt);  // smooth the torque in transition
    targetMotorPos = springPos[KNEE] + targetTorFilt / springK;

    double targetMotorPosFiltTemp = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    targetMotorPosFilt = targetMotorPosFilt + std::min(std::max(targetMotorPosFiltTemp - targetMotorPosFilt, -POS_INCRE_MAX), POS_INCRE_MAX);
    //targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);
    //spdlog::debug("motor pos: {}, spring pos: {}", targetMotorPosFilt, springPos[KNEE]);


    // Set position
    robot->setJointPosition(targetMotorPosFilt);

    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
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
        
        std::string left_str = "Calib Hum Drive " + resistanceText[resistanceFlag-1] + ", " + modeText[inFlexing] + ", " + actionText[actionFlag] + " -78deg ||";
        std::string right_str = "|| " + std::to_string(UPPER_ANGLE_LIMIT) + "deg";
        draw_progress_bar("||", "||", kneeAngle, LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT, PROGRESS_BAR_WIDTH);
        if (inFlexing) {
            draw_progress_bar(left_str, right_str, kneeAngle, LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT, PROGRESS_BAR_WIDTH);
            /*
            for (int i=-78; i<-18; i++) {
                if (kneeAngle > i) std::cout << "-";
                else std::cout << "+";
            }
            */
        }
        else {
            draw_progress_bar(left_str, right_str, kneeAngle, LOWER_ANGLE_LIMIT, UPPER_ANGLE_LIMIT, PROGRESS_BAR_WIDTH);
            /*
            std::cout << "|";
            for (int i=-78; i<-18; i++) {
                if (kneeAngle > i) std::cout << "+";
                else std::cout << "-";
            }
            */
        }
        //std::cout << "|| -18deg" << std::endl;
    }
}
void KECalibExerHumDriveState::exitCode(void) {
    Eigen::VectorXd& motorPos=robot->getPosition();
    motorPosForTrans = motorPos[KNEE];
    //robot->initProfilePositionControl(controlMotorProfile);
    //usleep(10000);
    //robot->setPosition(std::vector<double> 0.0);
}

/*=============================
Actual Exercise Robot in Charge - Robot in Position Control
=============================*/
void KEActualExerRobDriveState::entryCode(void) {
    //robot->stopSensorStreaming();
    //usleep(10000);
    //robot->startSensorStreaming();
    //robot->sensorCalibration();
    //robot->initCyclicVelocityControl(controlMotorProfile);
    //usleep(1000);
    robot->initProfilePositionControl(controlMotorProfile);
    usleep(1000);
    targetMotorPosFilt = motorPosForTrans;

    action_t = 0.;
}
void KEActualExerRobDriveState::duringCode(void) {

    sineFreq = freqTable[freqFlag];
    test_duration = num_cycles / sineFreq;  // 5 cycles

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
        {actionFlag = 1 - actionFlag; action_t = test_duration / num_cycles - 0.1; user_butt_pressed = false; button_released = false;}
        else if (gpio_button_status == 1) {button_released = true;}
        else ;
    }

    if (robot->keyboard->getW()==1) {
        resistanceFlag++; 
        if (resistanceFlag >= 4) resistanceFlag = 0;  // toggle between 1,2,3,4
    }
    if (robot->keyboard->getD()==1) {
        freqFlag = 1 - freqFlag;  // toggle between 0 and 1
        if (freqFlag == 0) {
            controlMotorProfile.profileVelocity = 200;
        }
        else {
            controlMotorProfile.profileVelocity = 1500;
        }
        robot->initProfilePositionControl(controlMotorProfile);
    }
    
    sineFreq = freqTable[freqFlag];
    test_duration = num_cycles / sineFreq;  // 5 cycles
    testCondition = freqText[freqFlag] + resistanceText[resistanceFlag];

    if (robot->keyboard->getA()==1) initLoggerStandard(testCondition);  // toggle between 1 and 2
    if (robot->keyboard->getS()==1) logHelperSB.endLog();  // toggle between 1 and 2

    double kneeAngle = springPos[KNEE]*180./M_PI;
    double kneeAngleMot = motorPos[KNEE]*180./M_PI;

    //springVelFilt = springVelFilt + (1.0-decay) * (springVelVec[KNEE] - springVelFilt);
    if (actionFlag == false) {
        cycle_count = 0;
        isSet = false;
        if (isSet == false) {
            if (targetPosStep1Set <= 4) {
                if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
                    robot->setJointPosition(-78* M_PI / 180);
                    targetPosStep1Set++;
                }
            }
            else {
                isSet = true;
                targetPosStep1Set = 0;
            }
            //targetMotorPos = -78.* M_PI / 180;
        }
        //double tarTorq = -78. - kneeAngle;  // go to 78 with propotional torque control
        //tarTorq = std::min(std::max(tarTorq, -10.), 10.);
        //targetTorFilt = targetTorFilt + 0.5 * (tarTorq - targetTorFilt);  // smooth the torque in transition
        //targetMotorPos = springPos[KNEE] + targetTorFilt / springK;
    }
    else if (cycle_count < num_cycles*2+2) {
        isSet = false;
        action_t += dt();
        if (action_t > (test_duration / num_cycles)){
            event_trigger = true;
            action_t = 0.0;
            cycle_count++;
            
        }
        //targetMotorPos = (-48-30*cos(2*M_PI*sineFreq*action_t)) * M_PI / 180;

        if (event_trigger == true){
            if (isSetHigh == false && isSetLow == true) {
                if (targetPosStep1Set <= 4) {
                    if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
                        robot->setJointPosition(-18* M_PI / 180);
                        targetPosStep1Set++;
                        spdlog::debug("Setting High: {}", cycle_count);
                    }
                }
                else {
                    isSetHigh = true;
                    isSetLow = false;
                    event_trigger = false;
                    targetPosStep1Set = 0;
                }
            }
            if (isSetHigh == true && isSetLow == false) {
                if (targetPosStep1Set <= 4) {
                    if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
                        robot->setJointPosition(-78* M_PI / 180);
                        targetPosStep1Set++;
                        spdlog::debug("Setting Low");
                    }
                }
                else {
                    isSetHigh = false;
                    isSetLow = true;
                    event_trigger = false;
                    targetPosStep1Set = 0;
                }
            }
        }
    }

        //double tarTorq = targetMotorPosTemp - kneeAngle;  // go to 78 with propotional torque control
        //tarTorq = std::min(std::max(tarTorq, -20.), 20.);
        //targetTorFilt = targetTorFilt + 0.5 * (tarTorq - targetTorFilt);  // smooth the torque in transition
        //targetMotorPos = springPos[KNEE] + targetTorFilt / springK;
    else 
    {
        isSet = false;
        if (isSet == false) {
            if (targetPosStep1Set <= 4) {
                if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
                    robot->setJointPosition(-78* M_PI / 180);
                    targetPosStep1Set++;
                }
            }
            else {
                isSet = true;
                targetPosStep1Set = 0;
            }
            //targetMotorPos = -78.* M_PI / 180;
        }
    }

    //double prev_tarPos = targetMotorPosFilt;
    //double targetMotorPosFiltTemp = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //targetMotorPosFilt = targetMotorPosFilt + std::min(std::max(targetMotorPosFiltTemp - targetMotorPosFilt, -POS_INCRE_MAX), POS_INCRE_MAX);
    
    //targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);
    //spdlog::debug("motor pos: {}, spring pos: {}", targetMotorPosFilt, springPos[KNEE]);


    // Set position
    //double targetVel = (targetMotorPosFilt - prev_tarPos) / dt();
    //robot->applyMotorPosition(targetMotorPosFilt, targetVel);
    //robot->setJointPosition(targetMotorPosFilt);
    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    double measuredTorq =  (motorPos[KNEE] - springPos[KNEE]) * springK;

    if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
        //std::cout << "action Flag: " << actionFlag << "action_time: " << action_t << std::endl;
        //std::cout << "test_duration: " << test_duration << "  action_t: " << action_t << std::endl;
        //std::cout << "Knee Angle: " << kneeAngle << "  Knee Angle Mot: " << kneeAngleMot << std::endl;
        //std::cout << "Target Torque: " << targetTorFilt << std::endl;
        if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
            std::cout << "RECORDING ";
        }
        std::cout << "Exercise Robot in Charge " << freqText[freqFlag] << ", " << resistanceText[resistanceFlag] << ", "
          << actionText[actionFlag] << " -24Nm ||";
        if (measuredTorq > 0) {
            for (int i=2*MAX_TORQUE_LEVEL; i>0; i--) {
                std::cout << "-";
            }
            std::cout << "|";
            for (int i=0; i<2*MAX_TORQUE_LEVEL; i++) {
                if ((measuredTorq*2) > i) std::cout << "+";
                else std::cout << "-";
            }
        }
        else {
            for (int i=-2*MAX_TORQUE_LEVEL; i<0; i++) {
                if ((measuredTorq*2) > i) std::cout << "-";
                else std::cout << "+";
            }
            std::cout << "|";
            for (int i=60; i>0; i--) {
                std::cout << "-";
            }
        }
        std::cout << "|| 24Nm" << std::endl;

    }
}
void KEActualExerRobDriveState::exitCode(void) {
    Eigen::VectorXd& motorPos=robot->getPosition();
    motorPosForTrans = motorPos[KNEE];
    //robot->setPosition(std::vector<double> 0.0);
}


/*=============================
Actual Exercise Sit to Stand
=============================*/
void KETorCtrlSit2Stand::entryCode(void) {
    //robot->stopSensorStreaming();
    //usleep(10000);
    //robot->startSensorStreaming();
    //robot->sensorCalibration();
    robot->initCyclicPositionControl(controlMotorProfile);
    usleep(10000);

    targetMotorPosFilt = motorPosForTrans;

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
        targetSpringTor = targetSpringTor * MAX_TORQUE_LEVEL /10. * assistanceLevel[assistanceFlag];  // change to body weight x assistance level
    }
    else {
        targetSpringTor = 0;
    }


    //targetTorFilt = targetTorFilt + 0.95 * (targetSpringTor - targetTorFilt);  // smooth the torque in transition
    targetTorFilt = targetSpringTor;
    targetMotorPos = springPos[KNEE] + targetTorFilt / springK;

    //double targetMotorPosFiltTemp = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //targetMotorPosFilt = targetMotorPosFilt + std::min(std::max(targetMotorPosFiltTemp - targetMotorPosFilt, -POS_INCRE_MAX), POS_INCRE_MAX);
    targetMotorPosFilt = targetMotorPosFilt + (1.0-decay) * (targetMotorPos - targetMotorPosFilt);
    //spdlog::debug("motor pos: {}, spring pos: {}", motorPos[KNEE], springPos[KNEE]);
    //spdlog::debug("motor pos: {}, spring pos: {}", targetMotorPosFilt, springPos[KNEE]);


    // Set position
    robot->setJointPosition(targetMotorPosFilt);
    if(logHelperSB.isStarted() && logHelperSB.isInitialised()) {
        logHelperSB.recordLogData();
    }

    if(iterations()%DISPLAY_DOWNSAMPLE_FACTOR==1) {
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
    Eigen::VectorXd& motorPos=robot->getPosition();
    motorPosForTrans = motorPos[KNEE];
    //logHelperSB.endLog();
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