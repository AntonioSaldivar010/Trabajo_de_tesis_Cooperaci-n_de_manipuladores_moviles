#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include <SerialStream.h>
#include <SerialPort.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <math.h>
#include <chrono>
#include "../include/ClientComm.h"

using namespace youbot;
using namespace std;
using namespace Eigen;
const int buff_size=5;
char CMD[buff_size];
double numberreceived;

const double pi = M_PI;

int main(int argc, char *argv[]) {
	JointSensedTorque torque;
	JointSensedVelocity velocity;
	JointSensedAngle angle;
	GripperBarSpacingSetPoint gripperSetPoint;
	ClientComm clientcommunication;
	VectorXd q_r(8);
	VectorXd dq_r(8);
	VectorXd dq(8), p(3), a1(3), a2(3), XD(3), XDpp(3), u(3);
	double est_error, time_r;

	/* configuration flags for different system configuration (e.g. base without arm)*/
	bool youBotHasBase = false;
	bool youBotHasArm = false;

	/* create handles for youBot base and manipulator (if available) */
	YouBotBase* myYouBotBase = 0;
	YouBotManipulator* myYouBotManipulator = 0;

	try {
		myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotBase->doJointCommutation();

		youBotHasBase = true;
	} catch (exception& e) {
		LOG(warning) << e.what();
		youBotHasBase = false;
	}

	try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		myYouBotManipulator->doJointCommutation();
		myYouBotManipulator->calibrateManipulator();

		youBotHasArm = true;
	} catch (exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
	}

	try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);    
		// calibrate the reference position of the gripper
		myYouBotManipulator->calibrateGripper();
		
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		} catch (...) {
		std::cout << "unhandled exception" << std::endl;
		}
	/*
	* Variable for the base.
	* Here "boost units" is used to set values in OODL, that means you have to set a value and a unit.
	*/
	quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
	quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;
	
	quantity<si::length> longitudinalPosition = 0 * meter;
	quantity<si::length> transversalPosition = 0 * meter;
	quantity<si::plane_angle> orientation = 0 * radian;

	/* Variable for the arm. */
	JointAngleSetpoint desiredJointAngle;
	JointVelocitySetpoint desiredVelocity;

	try {
		/*
		 * Simple sequence of commands to the youBot:
		 */		
		if (youBotHasArm && youBotHasBase) {
			double translationalVelocity; //meter_per_second
			double rotationalVelocity;  //radian_per_second

            // open the gripper 2 cm
			gripperSetPoint.barSpacing = 0.02 * meter;
			myYouBotManipulator->getArmGripper().setData(gripperSetPoint);

            //  			double q1 = (5.83 - 0.11)/2; // 0.11 - 5.83
            // 			double q3 = - 0.02; // -5.02 - -0.02
			double q1 = 0; // 0.11 - 5.83
			double q2 = 0; // 0.11 - 2.65
			double q3 = 0;
			double q4 = 0; // 0.12 - 3.59
			double q5 = 0;
			
			desiredJointAngle.angle = (q1+2.91) * radian;
			myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);
			desiredJointAngle.angle = (q2+1.2) * radian;
			myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);
			desiredJointAngle.angle = (q3-2.59) * radian;
			myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);
			desiredJointAngle.angle = (q4+1.8) * radian;
			myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);
			desiredJointAngle.angle = (q5+2.9) * radian;
			myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);;
			
			SLEEP_MILLISEC(5000);

			q_r(0) = 0;
			q_r(1) = 0;
			q_r(2) = 0;
			myYouBotManipulator->getArmJoint(1).getData(angle);
			q_r(3) = angle.angle.value() - 2.91;
			myYouBotManipulator->getArmJoint(2).getData(angle);
			q_r(4) = angle.angle.value() - 1.2;
			myYouBotManipulator->getArmJoint(3).getData(angle);
			q_r(5) = angle.angle.value() + 2.59;
			myYouBotManipulator->getArmJoint(4).getData(angle);
			q_r(6) = angle.angle.value() - 1.8;
			myYouBotManipulator->getArmJoint(5).getData(angle);
			q_r(7) = angle.angle.value() - 2.9;
			
			
			myYouBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			
			dq_r(2) = angularVelocity.value();
			dq_r(1) = longitudinalVelocity.value();
			dq_r(0) = transversalVelocity.value();
			myYouBotManipulator->getArmJoint(1).getData(velocity);
			dq_r(3) = velocity.angularVelocity.value();
			myYouBotManipulator->getArmJoint(2).getData(velocity);
			dq_r(4) = velocity.angularVelocity.value();
			myYouBotManipulator->getArmJoint(3).getData(velocity);
			dq_r(5) = velocity.angularVelocity.value();
			myYouBotManipulator->getArmJoint(4).getData(velocity);
			dq_r(6) = velocity.angularVelocity.value();
			myYouBotManipulator->getArmJoint(5).getData(velocity);
			dq_r(7) = velocity.angularVelocity.value();

			cout << endl << endl << "Condiciones iniciales brazo" << endl;
			cout << "+-----------------------------------------" << endl
			     << "| variable" << "\t" << "deseada " << "\t" << "real" << endl
			     << "| q1" << "\t\t" << q1 << "\t\t" << q_r(3) << endl
			     << "| q2" << "\t\t" << q2 << "\t\t" << q_r(4) << endl
			     << "| q3" << "\t\t" << q3 << "\t\t" << q_r(5) << endl
			     << "| q4" << "\t\t" << q4 << "\t\t" << q_r(6) << endl
			     << "| q5" << "\t\t" << q5 << "\t\t" << q_r(7) << endl
			     << "+-----------------------------------------" << endl << endl;
			
			int warn=0;
							
			int connfd;
 			connfd=clientcommunication.socket_client(argc, argv);
			cout<<"Communication stablished"<<endl;
			
			data_package data_r;
			data_package data_s;
			
			data_r=clientcommunication.data_receptor(connfd);
			
			// Initial joints configuration
			double q1r = data_r.q4; // 0.11 - 5.83
			double q2r = data_r.q5; // 0.11 - 2.65
			double q3r = data_r.q6;
			double q4r = data_r.q7; // 0.12 - 3.59
			double q5r = data_r.q8; // 0.12 - 5.6

			VectorXd qr(8);
			VectorXd dqr(8);
			double dt=0.01;
			     
			data_s.q1 = q_r(0);
			data_s.q2 = q_r(1);
			data_s.q3 = q_r(2);
			data_s.q4 = q_r(3);
			data_s.q5 = q_r(4);
			data_s.q6 = q_r(5);
			data_s.q7 = q_r(6);
			data_s.q8 = q_r(7);
			
			SLEEP_MILLISEC(5000);
			
				SerialPort serial("/dev/ttyUSB0");
    
    serial.Open(SerialPort::BAUD_115200,SerialPort::CHAR_SIZE_8,SerialPort::SerialPort::PARITY_NONE,SerialPort::STOP_BITS_1,SerialPort::FLOW_CONTROL_NONE);

    std::cout<<"Funcionó"<<std::endl;
    SerialPort::DataBuffer buffer;
//    std::cout<<CMD.size();
    int salida1=0;
    int byte_sel=0;
			
			ofstream q_csv("qReal1.csv");
			ofstream dq_csv("dqReal1.csv");
			ofstream E_csv("EReal1.csv");
			ofstream Eest_csv("EestReal1.csv");
			ofstream dP_csv("dPReal1.csv");
			ofstream V_csv("VReal1.csv");
			ofstream time_csv("time1.csv");
			ofstream utemp_csv("utemp1.csv");
			ofstream pose_csv("poseReal1.csv");
			ofstream alpha1xyz_csv("alpha1xyzReal1.csv");
			ofstream alpha2xyz_csv("alpha2xyzReal1.csv");
			ofstream referencia_csv("referenciaReal1.csv");
			ofstream referenciadpp_csv("referenciadppReal1.csv");
			
			clientcommunication.data_transmission(connfd,data_s);
			
			auto t_init = std::chrono::high_resolution_clock::now();

			while (1) {
			  
				myYouBotBase->getBasePosition(longitudinalPosition,transversalPosition,orientation);
				q_r(1)=longitudinalPosition.value();
				q_r(0)=transversalPosition.value();
				q_r(2)=orientation.value();
				myYouBotManipulator->getArmJoint(1).getData(angle);
				q_r(3) = angle.angle.value()-2.91;
				myYouBotManipulator->getArmJoint(2).getData(angle);
				q_r(4) = angle.angle.value()-1.2;
				myYouBotManipulator->getArmJoint(3).getData(angle);
				q_r(5) = angle.angle.value()+2.59;
				myYouBotManipulator->getArmJoint(4).getData(angle);
				q_r(6) = angle.angle.value()-1.8;
				myYouBotManipulator->getArmJoint(5).getData(angle);
				q_r(7) = angle.angle.value()-2.9;
				
				auto t_start = std::chrono::high_resolution_clock::now();

				serial.Read(buffer,buff_size,100);
	
				try{

				  numberreceived = double((int(buffer[1])-48)*1000 + (int(buffer[2])-48)*100 + (int(buffer[3])-48)*10 + (int(buffer[4])-48)*1)*.01;
				  std::cout << numberreceived<<std::endl;

				  buffer.clear();

				}
				catch(SerialPort::ReadTimeout E){
				  std::cout<<"TIMEOUT";
				}
				
				data_s.q1=q_r(0);
				data_s.q2=q_r(1);
				data_s.q3=q_r(2);
				data_s.q4=q_r(3);
				data_s.q5=q_r(4);
				data_s.q6=q_r(5);
				data_s.q7=q_r(6);
				data_s.q8=q_r(7);
				data_s.f=numberreceived;
				
				cout<<"Fuerza recibida: "<<numberreceived<<endl;
				
				clientcommunication.data_transmission(connfd,data_s);
				
				data_r=clientcommunication.data_receptor(connfd);
				
				dq(0)=data_r.dq1;
				dq(1)=data_r.dq2;
				dq(2)=data_r.dq3;
				dq(3)=data_r.dq4;
				dq(4)=data_r.dq5;
				dq(5)=data_r.dq6;
				dq(6)=data_r.dq7;
				dq(7)=data_r.dq8;
				p(0)=data_r.x;
				p(1)=data_r.y;
				p(2)=data_r.z;
				time_r=data_r.t;
				u(0)=data_r.ux;
				u(1)=data_r.uy;
				u(2)=data_r.uz;
				XD(0)=data_r.xr;
				XD(1)=data_r.yr;
				XD(2)=data_r.zr;
				XDpp(0)=data_r.xrpp;
				XDpp(1)=data_r.yrpp;
				XDpp(2)=data_r.zrpp;
				est_error=data_r.esterror;
				a1(0)=data_r.a1x;
				a1(1)=data_r.a1y;
				a1(2)=data_r.a1z;
				a2(0)=data_r.a2x;
				a2(1)=data_r.a2y;
				a2(2)=data_r.a2z;
				
				cout<<"q1 "<<q_r(0)<<endl;
				cout<<"q1 "<<q_r(1)<<endl;
				cout<<"q1 "<<q_r(2)<<endl;
				cout<<"q1 "<<q_r(3)<<endl;
				cout<<"q1 "<<q_r(4)<<endl;
				cout<<"q1 "<<q_r(5)<<endl;
				cout<<"q1 "<<q_r(6)<<endl;
				cout<<"q1 "<<q_r(7)<<endl<<endl;
				cout<<"dq1 "<<dq(0)<<endl;
				cout<<"dq1 "<<dq(1)<<endl;
				cout<<"dq1 "<<dq(2)<<endl;
				cout<<"dq1 "<<dq(3)<<endl;
				cout<<"dq1 "<<dq(4)<<endl;
				cout<<"dq1 "<<dq(5)<<endl;
				cout<<"dq1 "<<dq(6)<<endl;
				cout<<"dq1 "<<dq(7)<<endl<<endl;
				
				cout << "esterror1= "<<est_error<<endl;
				
				dq_csv << dq(0) << "\t" << dq(1) << "\t" << dq(2) << "\t" << dq(3) << "\t" << dq(4) << "\t" << dq(5) << "\t" << dq(6) << "\t" << dq(7) << endl;
				pose_csv << p(0) << "\t" << p(1) << "\t" << p(2) << endl;
				alpha1xyz_csv << a1(0) << "\t" << a1(1) << "\t" << a1(2) << endl;
				alpha2xyz_csv<< a2(0) << "\t" << a2(1) << "\t" << a2(2) << endl;
				referencia_csv << XD(0) << "\t" << XD(1) << "\t" << XD(2) << endl;
				referenciadpp_csv << XDpp(0) << "\t" << XDpp(1) << "\t" << XDpp(2) << endl;
				utemp_csv << u(0) << "\t" << u(1) << "\t" << u(2) << endl;
				Eest_csv << est_error << endl;
				time_csv << time_r << endl;
               
				
				for(int i=0; i<8; i++)
				{
				  if(dq(i)!=dq(i) || q_r(i)!=q_r(i))
				  {
				    cout<<"Nan!!!"<<endl;
				    warn=1;
				  }
				}
				
				if(warn==1)
				  break;
								
//  				quantity<si::velocity> longitudinalVelocity = dq(1) * meter_per_second;
//  				quantity<si::velocity> transversalVelocity = dq(0) * meter_per_second;
//  				quantity<si::angular_velocity> angularVelocity = dq(2) * radian_per_second;
				
 				quantity<si::velocity> longitudinalVelocity = 0 * meter_per_second;
 				quantity<si::velocity> transversalVelocity = 0 * meter_per_second;
 				quantity<si::angular_velocity> angularVelocity = 0 * radian_per_second;
								
				myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
			
//  				desiredVelocity.angularVelocity = dq(3) * radian_per_second;
//  				myYouBotManipulator->getArmJoint(1).setData(desiredVelocity);
//  				desiredVelocity.angularVelocity = dq(4) * radian_per_second;
//  				myYouBotManipulator->getArmJoint(2).setData(desiredVelocity);
//  				desiredVelocity.angularVelocity = dq(5) * radian_per_second;
//  				myYouBotManipulator->getArmJoint(3).setData(desiredVelocity);
//  				desiredVelocity.angularVelocity = dq(6) * radian_per_second;
//  				myYouBotManipulator->getArmJoint(4).setData(desiredVelocity);
//  				desiredVelocity.angularVelocity = dq(7) * radian_per_second;
//  				myYouBotManipulator->getArmJoint(5).setData(desiredVelocity);
				
 				desiredVelocity.angularVelocity = 0 * radian_per_second;
 				myYouBotManipulator->getArmJoint(1).setData(desiredVelocity);
 				desiredVelocity.angularVelocity = 0 * radian_per_second;
 				myYouBotManipulator->getArmJoint(2).setData(desiredVelocity);
 				desiredVelocity.angularVelocity = 0 * radian_per_second;
 				myYouBotManipulator->getArmJoint(3).setData(desiredVelocity);
 				desiredVelocity.angularVelocity = 0 * radian_per_second;
 				myYouBotManipulator->getArmJoint(4).setData(desiredVelocity);
 				desiredVelocity.angularVelocity = 0 * radian_per_second;
 				myYouBotManipulator->getArmJoint(5).setData(desiredVelocity);
				
			}
			serial.Close();
			cout << "FINISHED..." << endl << endl;
		}

	} catch (exception& e) {
		cout << e.what() << endl;
		cout << "unhandled exception" << endl;
	}

	/* clean up */
	if (myYouBotBase) {
		delete myYouBotBase;
		myYouBotBase = 0;
	}
	if (myYouBotManipulator) {
		delete myYouBotManipulator;
		myYouBotManipulator = 0;
	}

	LOG(info) << "Done.";

	return 0;
}
