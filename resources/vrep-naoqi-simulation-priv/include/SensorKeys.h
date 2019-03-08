/**
 * @file include/JointsEnum.h
 *
 * This file declares the enumeration ids for all the sensors and 
 * actuators available in the simulated model of Nao
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jun 2017
 */

#pragma once

#include <string>

using namespace std;

#define NUM_JOINTS 24
#define NUM_FSR_SENSORS 8
#define NUM_IMU_SENSORS 7
#define NUM_CAMERA_SENSORS 2

const char* vrepJoints[NUM_JOINTS] {
  "HeadYaw", 
  "HeadPitch", 
  "LShoulderPitch",
  "LShoulderRoll", 
  "LElbowYaw", 
  "LElbowRoll", 
  "LWristYaw", 
  "RShoulderPitch", 
  "RShoulderRoll", 
  "RElbowYaw", 
  "RElbowRoll", 
  "RWristYaw", 
  "LHipYawPitch", 
  "LHipRoll", 
  "LHipPitch", 
  "LKneePitch", 
  "LAnklePitch", 
  "LAnkleRoll",
  "RHipYawPitch", 
  "RHipRoll",
  "RHipPitch", 
  "RKneePitch", 
  "RAnklePitch", 
  "RAnkleRoll"
};

const char* vrepFsrs[NUM_FSR_SENSORS] {
	"LFsrFL_force",
	"LFsrFR_force",
	"LFsrRL_force",
	"LFsrRR_force",
	"RFsrFL_force", 
	"RFsrFR_force",
	"RFsrRL_force", 
	"RFsrRR_force"
};

const char* vrepImu[NUM_IMU_SENSORS] {
  "angleX",
  "angleY", 
  "accelerometerX", 
  "accelerometerY",
  "accelerometerZ",
  "gyroX", 
  "gyroY"
};

const char* vrepCameras[NUM_CAMERA_SENSORS] {
	"CameraTop",
	"CameraBottom",
};

const char* naoqiJoints[NUM_JOINTS] {
  "HeadYaw", 
  "HeadPitch", 
  "LShoulderPitch",
  "LShoulderRoll", 
  "LElbowYaw", 
  "LElbowRoll", 
  "LWristYaw", 
  "RShoulderPitch", 
  "RShoulderRoll", 
  "RElbowYaw", 
  "RElbowRoll", 
  "RWristYaw", 
  "LHipYawPitch", 
  "LHipRoll", 
  "LHipPitch", 
  "LKneePitch", 
  "LAnklePitch", 
  "LAnkleRoll",
  "RHipYawPitch", 
  "RHipRoll",
  "RHipPitch", 
  "RKneePitch", 
  "RAnklePitch", 
  "RAnkleRoll"
};

const char* naoqiFsrs[NUM_FSR_SENSORS] {
	"LFoot/FSR/FrontLeft",
	"LFoot/FSR/FrontRight",
	"LFoot/FSR/RearLeft",
	"LFoot/FSR/RearRight",
	"RFoot/FSR/FrontLeft", 
	"RFoot/FSR/FrontRight",
	"RFoot/FSR/RearLeft", 
	"RFoot/FSR/RearRight"
};

const char* naoqiCameras[NUM_CAMERA_SENSORS] {
	"CameraTop",
	"CameraBottom",
};


