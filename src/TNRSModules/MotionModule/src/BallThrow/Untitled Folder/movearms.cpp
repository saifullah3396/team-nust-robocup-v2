#include <iostream>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <qi/os.hpp>
#include <math.h>
#include <vector>
#include "NAOKinematics.h"
#include "KMat.hpp"

using namespace std;
using namespace KMath::KMat;
using namespace KDeviceLists;

int main(int argc, char **argv)
{
/*
float radian = M_PI/180.0;
float degree = 180.0 / M_PI;
NAOKinematics nkin;
	NAOKinematics::kmatTable output1, output2, output3, output4, output5, EETR, EETL,T1, T2;
	
	std::vector<float> joints(NUMOFJOINTS);
	double pi = KMath::KMat::transformations::PI;
        joints[L_ARM+SHOULDER_PITCH]=33.9f*radian;
	joints[L_ARM+SHOULDER_ROLL]=-3.0f*radian;
	joints[L_ARM+ELBOW_YAW]=-62.3f*radian;
	joints[L_ARM+ELBOW_ROLL]=-65.7f*radian;
	joints[L_ARM+WRIST_YAW]=-14.2f*radian;
	//Right Hand
	joints[R_ARM+SHOULDER_PITCH]=33.9f*radian;
	joints[R_ARM+SHOULDER_ROLL]=3.0f*radian;
	joints[R_ARM+ELBOW_YAW]=62.5f*radian;
	joints[R_ARM+ELBOW_ROLL]=65.7f*radian;
	joints[R_ARM+WRIST_YAW]=23.4f*radian;	


nkin.setJoints(joints);

output1 = nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_ARM);
	
	//Right Hand
output2 = nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_ARM);

	std::cout << "x = " << output1(0,3) << " y = " << output1(1,3) << " z = " << output1(2,3) <<  std::endl;
	std::cout << "x = " << output2(0,3) << " y = " << output2(1,3) << " z = " << output2(2,3) <<  std::endl;
	
double lxshoulder = output1(0,3) - 0.0;
double lyshoulder = output1(1,3) - 98.0;
double lzshoulder = output1(2,3) - 100.0;

std::cout << " Left hand : X co-ordinate from Centre of circle = " << lxshoulder << std::endl;
std::cout << " Left Hand : y co-ordinate from Centre of circle = " << lyshoulder << std::endl;
std::cout << " Left Hand : z co-ordinate from Centre of circle = " << lzshoulder << std::endl;

double rxshoulder = output2(0,3) - 0.0;
double ryshoulder = output2(1,3) + 98.0;
double rzshoulder = output2(2,3) - 100.0;

std::cout << " Right hand : X co-ordinate from Centre of circle = " << rxshoulder << std::endl;
std::cout << " Right Hand : y co-ordinate from Centre of circle = " << ryshoulder << std::endl;
std::cout << " Right Hand : z co-ordinate from Centre of circle = " << rzshoulder << std::endl;

double lradius = sqrt(pow(lxshoulder, 2.0) + pow(lzshoulder, 2.0));
double rradius = sqrt(pow(rxshoulder, 2.0) + pow(rzshoulder, 2.0));

std::cout << " Left Circle Radius is = " << lradius << std::endl;
std::cout << " Right Circle Radius is = " << rradius << std::endl;

double LDistance = lradius * joints[L_ARM+SHOULDER_PITCH];
std::cout << " Left : Distance covered on the circumference of circle is = " << LDistance << std::endl;

double RDistance = rradius * joints[R_ARM+SHOULDER_PITCH];
std::cout << " Right : Distance covered on the circumference of circle is = " << RDistance << std::endl;

output1.prettyPrint();
output2.prettyPrint();

double lr = atan2(output1(2, 1), output1(2, 2));
double lp = asin(-output1(2, 0));
double ly = atan2(output1(1, 0), output1(0, 0));
std::cout << "r= " << lr*degree << " p= " << lp*degree << " y= "<< ly*degree << std::endl;


double rr = atan2(output2(2, 1), output2(2, 2));
double rp = asin(-output2(2, 0));
double ry = atan2(output2(1, 0), output2(0, 0));
std::cout << "r= " << rr*degree << " p= " << rp*degree << " y= "<< ry*degree << std::endl;




/*

KMath::KMat::transformations::makeTranslation(EETL, 100.0, 50.0, 0.0);
KMath::KMat::transformations::makeTranslation(EETR, 100.0, -50.0, 0.0);
KMath::KMat::transformations::makeRotationXYZ(T1, -M_PI_2,0.0 , -M_PI_4);
KMath::KMat::transformations::makeRotationXYZ(T2, 0.0, 0.0, 0.0);
EETL = T1 * EETL;
EETR = T2 * EETR;

EETL.prettyPrint();
EETR.prettyPrint();

/*
EETL(0, 0) = 0.0;
EETL(0, 1) = 0.0;
EETL(0, 2) = 1.0;
EETL(0, 3) = 0.0;
EETL(1, 0) = 1.0;
EETL(1, 1) = 0.0;
EETL(1, 2) = 0.0;
EETL(1, 3) = 50.0;
EETL(2, 0) = 0.0;
EETL(2, 1) = 1.0;
EETL(2, 2) = 0.0;
EETL(2, 3) = 0.0;
EETL(3, 0) = 0.0;
EETL(3, 1) = 0.0;
EETL(3, 2) = 0.0;
EETL(3, 3) = 1.0;


  vector<vector<float> > result;
  result = nkin.inverseLeftHand(EETL);
  
  if(!result.empty()){
		cout << "--Solution exists 1" << endl;
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
                 }

		cout << endl;

}
else 
{
cout << "Result Doesn't Exist" << endl;
}

result = nkin.inverseRightHand(EETR);
	if(!result.empty()){
		cout << "--Solution exists 2" << endl;
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
}

else
{
cout << "Result Doesn't Exist" << endl;
}

*/

std::string robotIp = argv[1];

  if (argc < 2) {
    std::cerr << "Usage: almotion_setpositions robotIp "
              << "(optional default \"192.168.100.5\")."<< std::endl;
  }
  else {
    robotIp = argv[1];
  }
float radian = M_PI/180;

  AL::ALMotionProxy motion(robotIp);
  AL::ALRobotPostureProxy robotPosture(robotIp);
  robotPosture.goToPosture("StandInit", 0.5f);
  AL::ALValue stiffness = 1.0f;




 
AL::ALValue timeLists;
AL::ALValue angleLists;

std::string RhandName = "RHand";
motion.post.openHand(RhandName);
std::string LhandName = "LHand";
motion.openHand(LhandName);
AL::ALValue names = ("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");
bool isAbsolute        = true;

  names = AL::ALValue::array("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");

angleLists.clear();
angleLists.arraySetSize(10);
  angleLists[0] = AL::ALValue(-1.5f*radian);
  angleLists[1] = AL::ALValue(29.2f*radian);
  angleLists[2] = AL::ALValue(-45.5f*radian);
  angleLists[3] = AL::ALValue(-53.8f*radian);
  angleLists[4] = AL::ALValue(-35.1f*radian);
  angleLists[5] = AL::ALValue(2.1f*radian);
  angleLists[6] = AL::ALValue(34.2f*radian);
  angleLists[7] = AL::ALValue(51.2f*radian);
  angleLists[8] = AL::ALValue(59.1f*radian);
  angleLists[9] = AL::ALValue(24.8f*radian);
timeLists.arraySetSize(10);
for (int i=0; i<10; i++)
{
  timeLists[i] = AL::ALValue(1.0f);
  }


  isAbsolute = true;
  motion.angleInterpolation(names, angleLists, timeLists, isAbsolute);

AL::ALValue staticTime;
AL::ALValue staticAngle;

AL::ALValue sNames = ("LShoulderRoll", "RShoulderRoll");
sNames = AL::ALValue::array("LShoulderRoll", "RShoulderRoll");
staticAngle.clear();
staticAngle.arraySetSize(2);
staticAngle[0] = AL::ALValue(-24.95f*radian);
staticAngle[1] = AL::ALValue(24.95f*radian);

staticTime.arraySetSize(2);
for (int j=0; j<2; j++)
{
staticTime[j] = AL::ALValue(1.0f);
}

isAbsolute = true;

motion.angleInterpolation(sNames, staticAngle, staticTime, isAbsolute);

AL::ALValue lifttimeLists;
AL::ALValue liftangleLists;


AL::ALValue Lnames = ("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");


  Lnames = AL::ALValue::array("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");

liftangleLists.clear();
liftangleLists.arraySetSize(10);
  liftangleLists[0] = AL::ALValue(-5.1f*radian);
  liftangleLists[1] = AL::ALValue(-25.9f*radian);
  liftangleLists[2] = AL::ALValue(-63.7f*radian);
  liftangleLists[3] = AL::ALValue(-69.6f*radian);
  liftangleLists[4] = AL::ALValue(-10.8f*radian);
  liftangleLists[5] = AL::ALValue(17.7f*radian);
  liftangleLists[6] = AL::ALValue(-26.6f*radian);
  liftangleLists[7] = AL::ALValue(61.2f*radian);
  liftangleLists[8] = AL::ALValue(75.9f*radian);
  liftangleLists[9] = AL::ALValue(-3.8f*radian);
lifttimeLists.arraySetSize(10);
for (int i=0; i<10; i++)
{
  lifttimeLists[i] = AL::ALValue(2.0f);
  }


  isAbsolute = true;
  motion.angleInterpolation(Lnames, liftangleLists, lifttimeLists, isAbsolute);

AL::ALValue lbtimeLists;
AL::ALValue lbangleLists;


AL::ALValue Lbnames = ("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");


  Lbnames = AL::ALValue::array("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");

lbangleLists.clear();
lbangleLists.arraySetSize(10);
  lbangleLists[0] = AL::ALValue(-9.5f*radian);
  lbangleLists[1] = AL::ALValue(-94.3f*radian);
  lbangleLists[2] = AL::ALValue(-18.3f*radian);
  lbangleLists[3] = AL::ALValue(-28.9f*radian);
  lbangleLists[4] = AL::ALValue(-57.0f*radian);
  lbangleLists[5] = AL::ALValue(7.9*radian);
  lbangleLists[6] = AL::ALValue(-77.2f*radian);
  lbangleLists[7] = AL::ALValue(37.4f*radian);
  lbangleLists[8] = AL::ALValue(62.0f*radian);
  lbangleLists[9] = AL::ALValue(37.8f*radian);
  

lbtimeLists.arraySetSize(10);
for (int i=0; i<10; i++)
{
  lbtimeLists[i] = AL::ALValue(2.0f); 
  }

  isAbsolute = true;
  motion.angleInterpolation(Lbnames, lbangleLists, lbtimeLists, isAbsolute);

AL::ALValue vitimeLists;
AL::ALValue viangleLists;


AL::ALValue vinames = ("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");


  vinames = AL::ALValue::array("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");

viangleLists.clear();
viangleLists.arraySetSize(10);
  viangleLists[0] = AL::ALValue(-2.8f*radian);
  viangleLists[1] = AL::ALValue(-79.2f*radian);
  viangleLists[2] = AL::ALValue(-56.1f*radian);
  viangleLists[3] = AL::ALValue(-60.4f*radian);
  viangleLists[4] = AL::ALValue(-20.8f*radian);
  viangleLists[5] = AL::ALValue(-0.5*radian);
  viangleLists[6] = AL::ALValue(-83.8f*radian);
  viangleLists[7] = AL::ALValue(55.5f*radian);
  viangleLists[8] = AL::ALValue(51.3f*radian);
  viangleLists[9] = AL::ALValue(32.2f*radian);
  

vitimeLists.arraySetSize(10);
for (int i=0; i<10; i++)
{
  vitimeLists[i] = AL::ALValue(2.0); 
  }

  isAbsolute = true;
  motion.angleInterpolation(vinames, viangleLists, vitimeLists, isAbsolute);


AL::ALValue viitimeLists;
AL::ALValue viiangleLists;


AL::ALValue viinames = ("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");


  viinames = AL::ALValue::array("LShoulderRoll", "LShoulderPitch", "LElbowRoll", "LElbowYaw", "LWristYaw", "RShoulderRoll", "RShoulderPitch", "RElbowRoll", "RElbowYaw", "RWristYaw");

viiangleLists.clear();
viiangleLists.arraySetSize(10);
  viiangleLists[0] = AL::ALValue(-3.0f*radian);
  viiangleLists[1] = AL::ALValue(-69.5f*radian);
  viiangleLists[2] = AL::ALValue(-65.7f*radian);
  viiangleLists[3] = AL::ALValue(-62.3f*radian);
  viiangleLists[4] = AL::ALValue(-14.2f*radian);
  viiangleLists[5] = AL::ALValue(3.0f*radian);
  viiangleLists[6] = AL::ALValue(-68.5f*radian);
  viiangleLists[7] = AL::ALValue(63.5f*radian);
  viiangleLists[8] = AL::ALValue(62.5f*radian);
  viiangleLists[9] = AL::ALValue(23.4f*radian);
  

viitimeLists.arraySetSize(10);
for (int i=0; i<10; i++)
{
  viitimeLists[i] = AL::ALValue(2.0); 
  }

  isAbsolute = true;
  motion.angleInterpolation(viinames, viiangleLists, viitimeLists, isAbsolute);


AL::ALValue itimeLists;
AL::ALValue iangleLists;


AL::ALValue inames = ("LShoulderPitch", "RShoulderPitch", "LShoulderRoll", "RShoulderRoll");


  inames = AL::ALValue::array("LShoulderPitch", "RShoulderPitch", "LShoulderRoll", "RShoulderRoll", "LWristYaw", "RWristYaw");

iangleLists.clear();
iangleLists.arraySetSize(6);
  iangleLists[0] = AL::ALValue::array(-54.4f*radian, -42.8f*radian, -26.7f*radian, 2.6f*radian, 33.9f*radian);
  iangleLists[1] = AL::ALValue::array(-54.4f*radian, -42.8f*radian, -26.7f*radian, 2.6f*radian, 33.9f*radian);
  iangleLists[2] = AL::ALValue::array(3.0f*radian, 25.0f*radian, 8.0f*radian, 6.0f*radian, 3.0f*radian);
  iangleLists[3] = AL::ALValue::array(-3.0f*radian, -25.0f*radian, -8.0f*radian, -6.0f*radian, -3.0f*radian);
  iangleLists[4] = AL::ALValue::array(-30.0f*radian, -25.0f*radian, -20.0f*radian, -15.0f*radian, -14.0f*radian);
  iangleLists[5] = AL::ALValue::array(30.0f*radian, 25.0f*radian, 20.0f*radian, -15.0f*radian, -14.0f*radian);
itimeLists.arraySetSize(6);
for (int i=0; i<6; i++)
{
  itimeLists[i] = AL::ALValue::array(0.1, 0.2, 0.3, 0.4, 0.5); 
  }

  isAbsolute = true;
  motion.angleInterpolation(inames, iangleLists, itimeLists, isAbsolute);



/*

AL::ALValue htimeLists;
AL::ALValue hangleLists;


AL::ALValue hnames = ("HeadPitch");


  hnames = AL::ALValue("HeadPitch");

hangleLists.clear();
hangleLists.arraySetSize(1);
  hangleLists[0] = AL::ALValue::array(-20.0*radian, 4.7f*radian);

htimeLists.arraySetSize(1);
for (int i=0; i<1; i++)
{
  htimeLists[i] = AL::ALValue::array(0.5f, 2.0f); 
  }


  isAbsolute = true;
  motion.angleInterpolation(hnames, hangleLists, htimeLists, isAbsolute);
/*






AL::ALValue leftTime;
AL::ALValue leftAngle;

AL::ALValue leftNames = ("LElbowRoll", "RElbowRoll");
leftNames = AL::ALValue::array("LElbowRoll", "RElbowRoll");
leftAngle.clear();
leftAngle.arraySetSize(2);
leftAngle[0] = AL::ALValue(-37.0f*radian);
leftAngle[1] = AL::ALValue(37.0f*radian);

leftTime.arraySetSize(2);
for (int j=0; j<2; j++)
{
leftTime[j] = AL::ALValue(0.1f);
}

isAbsolute = true;
motion.angleInterpolation(leftNames, leftAngle, leftTime, isAbsolute);


/*

AL::ALValue legTime;
AL::ALValue legAngle;

AL::ALValue lNames = ("RHipYawPitch", "RHipPitch", "RKneePitch", "RAnklePitch", "RHipRoll", "LHipYawPitch", "LHipPitch", "LKneePitch", "LAnklePitch", "LHipRoll");
lNames = AL::ALValue::array("RHipYawPitch", "RHipPitch", "RKneePitch", "RAnklePitch", "RHipRoll", "LHipYawPitch", "LHipPitch", "LKneePitch", "LAnklePitch", "LHipRoll");
legAngle.clear();
legAngle.arraySetSize(10);
legAngle[0] = AL::ALValue::array(-14.0f*radian, -0.8f*radian);
legAngle[1] = AL::ALValue::array(-39.6f*radian, -26.5f*radian);
legAngle[2] = AL::ALValue::array(120.6f*radian, 40.1f*radian);
legAngle[3] = AL::ALValue::array(-67.7f*radian, -21.1f*radian);
legAngle[4] = AL::ALValue::array(4.7f*radian, -0.1*radian);
legAngle[5] = AL::ALValue::array(-14.0f*radian, -0.8*radian);
legAngle[6] = AL::ALValue::array(-39.6f*radian, -25.7f*radian);
legAngle[7] = AL::ALValue::array(120.7f*radian, 39.6f*radian);
legAngle[8] = AL::ALValue::array(-68.1f*radian, -21.1f*radian);
legAngle[9] = AL::ALValue::array(-4.4f*radian, 2.0f*radian);

legTime.arraySetSize(10);
for (int j=0; j<10; j++)
{
legTime[j] = AL::ALValue::array(4.0f, 7.0f);
}

isAbsolute = true;

motion.angleInterpolation(lNames, legAngle, legTime, isAbsolute);

*/
  return 0;
    
}

