#include <stdio.h>
#include <stdlib.h>
#include "Aria.h"
#include <list>
#include <iostream>
#include <math.h>
#include <time.h>
#include "opencv2/opencv.hpp"
#define PI 3.14
ArRobot robot;
ArSensorReading *sonarReading;
ArLaser *myLaser;
ArGripper*myGripper = NULL;
double dist, angle;
FILE *fpData, *opData, *veldata, *postdata, *veldata2;
double Xp, Yp, Xr, Yr, X, Y, th;
float RadToDeg = PI / 180;
int a;
int sonarNum = 8;
int right_turnV = 280;
int left_turnV = 400;
int init_speed = 250;
int object_Dected = 0;
int right_turnV3 = 295;
int left_turnV3 = 460;
int finish = 0;
void update(void);
ArGlobalFunctor updateCB(&update);
int sonar_read[] = { 0,0,0,0,0,0,0,0 };
int laser_read[180][4] = {};
int son_po[8][3] = { { 0,0,90 },{ 0,0,50 },{ 0,0,30 },{ 0,0,10 },{ 0,0,-10 },{ 0,0,-30 },{ 0,0,-50 },{ 0,0,-90 } };
int enable = 0;
double velocity_left, velocity_right;
int ObjectInFront = 2000;
int ObjectInLeftFront = 480;
int ObjectInLeft = 800;
int ObjectInRight = 800;
int ObjectInRight2gap = 700;
int Stop_detect = 600;
int Reach_ball_range = 1700;
int ver_pass_gap = 525;
int max_degree = 91;
int min_degree = -90;
int first_object = 2000;
int max_range = 5000;
int min_range = -1000;
double x_coordinate;
double X_ver, Y_ver;
int ver_range = 50;
int sonar_min = 250;
int sonar_max = 5000;
int degree_incr = 10;
int slower_speed = 100;
int left = 10;
int right = -10;
int inital_X = 3800;
int inital_Y = 3500;
int inttal_Z = 270;
int stop = 0;
//count the time
time_t tm = time(0);
// The camera (Cannon VC-C4).
ArVCC4 vcc4(&robot);
cv::VideoCapture cap;
double getBallLocation(cv::VideoCapture cap)
{
	cv::Mat frame;
	double x_coordinate;
	// Capture frame-by-frame
	cap >> frame;  // read a new frame from video 
				   // If the frame is empty, break immediately
	if (frame.empty()) {
		std::cout << "Error: frame empty" << std::endl;
		return -1;
	}
	// Display the resulting frame
	cv::imshow("Original Frame", frame);

	// Resize frame to 160 * 120.
	cv::resize(frame, frame, cv::Size(160, 120));
	// Segment frame using orange color (B G R order).
	cv::inRange(frame, cv::Scalar(0, 60, 150), cv::Scalar(80, 165, 255), frame);
	//cv::inRange(frame, cv::Scalar(25, 60, 150), cv::Scalar(60, 120, 255), frame);
	// Set up the detector with parameters.
	cv::SimpleBlobDetector::Params params;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByCircularity = false;
	params.minThreshold = 0;
	params.maxThreshold = 255;
	params.filterByColor = true;
	params.blobColor = 255;
	params.filterByArea = true;
	params.minArea = 50;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	// Detect blobs.
	std::vector<cv::KeyPoint> keypoints;
	detector->detect(frame, keypoints);
	// shows the detected ball
	cv::imshow("Detected Ball", frame);
	cv::waitKey(20); // wait for the keyboard input

					 // Return X coordinate.
	if (keypoints.size() > 0)
	{
		x_coordinate = keypoints[0].pt.x;
		printf("x_coordinate:%f ", x_coordinate);
		return x_coordinate;
	}
	else {
		std::cout << "Can't get the ball's x coordinate " << std::endl;
		return 0;
	}
}

void chaseBall(double x_coordinate)
{
	double xRel;
	double width = 160.0; // Image size is 160 * 120
	if (x_coordinate > 0)
	{
		// Determine where the blob's center of gravity
		// is relative to the center of the camera.
		xRel = (x_coordinate - width / 2.0) / width - 0.15;

		// Set the heading and velocity for the robot.
		if (ArMath::fabs(xRel) < .1)
		{
			robot.setDeltaHeading(0);
		}
		else
		{
			if (ArMath::fabs(xRel) <= 1)
				robot.setDeltaHeading(-xRel * 5);
			else if (-xRel > 0)
				robot.setDeltaHeading(left);
			else
				robot.setDeltaHeading(right);
		}
		robot.setVel(slower_speed);
	}
}


void SonarToGobel()
{
	//convert local to gobel
	//get sonar reading
	sonarReading = robot.getSonarReading(a);
	//write into a loop
	sonar_read[a] = sonarReading->getRange();
	//get local X
	son_po[a][0] = sonarReading->getSensorX();
	//get local Y
	son_po[a][1] = sonarReading->getSensorY();
	//convert by formula
	Xp = sonar_read[a] * cos(son_po[a][2] * RadToDeg) + son_po[a][0];
	Yp = sonar_read[a] * sin(son_po[a][2] * RadToDeg) + son_po[a][1];
	Xr = Xp*cos(th*RadToDeg) - Yp*sin(th*RadToDeg);
	Yr = Xp*sin(th*RadToDeg) + Yp*cos(th*RadToDeg);
}
void LaserToGobel()
{
	myLaser = robot.findLaser(1);
	//convert local to gobel
	float xpos = robot.getPose().getX();
	float ypos = robot.getPose().getY();
	//get the angle where laser scan
	dist = myLaser->currentReadingPolar(a, a + 5, &angle);
	Xp = dist * cos(angle * RadToDeg) + myLaser->getSensorPositionX();
	Yp = dist * sin(angle * RadToDeg) + myLaser->getSensorPositionY();
	Xr = Xp*cos(th*RadToDeg) - Yp*sin(th*RadToDeg);
	Yr = Xp*sin(th*RadToDeg) + Yp*cos(th*RadToDeg);
	X = Xr + xpos;
	Y = Yr + ypos;
	//write into laser data
	if (Y>min_range && X>min_range && X<max_range  && Y<max_range)
	{
		fprintf(fpData, "%.1f   %.1f  \n", X, Y);
	}
}
void verify_sonar()
{
	myLaser = robot.findLaser(1);
	//convert local to gobel
	float xpos = robot.getPose().getX();
	float ypos = robot.getPose().getY();
	//get the angle where laser scan
	dist = myLaser->currentReadingPolar(son_po[a][2] - 5, son_po[a][2] + 5, &angle);
	Xp = dist * cos(angle * RadToDeg) + myLaser->getSensorPositionX();
	Yp = dist * sin(angle * RadToDeg) + myLaser->getSensorPositionY();
	Xr = Xp*cos(th*RadToDeg) - Yp*sin(th*RadToDeg);
	Yr = Xp*sin(th*RadToDeg) + Yp*cos(th*RadToDeg);
	X_ver = Xr + xpos;
	Y_ver = Yr + ypos;
}
void vel()
{
	//store time velocity data
	velocity_left = robot.getLeftVel();
	velocity_right = robot.getRightVel();
	//get the time difference from start
	double tm_pass = difftime(time(0), tm);
	//write velocity
	fprintf(veldata2, "%.1f  %.1f  \n", velocity_right, tm_pass - 40);
	fprintf(veldata, "%.1f  %.1f  \n", velocity_left, tm_pass - 40);
}
void turning()
{
	// start turning when dected first object and pass the first gap and act as sharp turning when pass last gap.
	if ((sonar_read[3]<ObjectInFront || sonar_read[4]<ObjectInFront) && object_Dected == 1)
	{
		//turn right
		robot.setVel2(left_turnV, right_turnV);
		vel();
	}
	//to pass second gap
	else if (sonar_read[7] < ObjectInRight && sonar_read[0] < ObjectInLeft && object_Dected == 1)
	{
		object_Dected = 2;
		vel();
	}
	// verify that passed second gap
	else if (sonar_read[7] > ver_pass_gap && object_Dected == 2)
	{
		object_Dected = 3;
		robot.setVel2(left_turnV, right_turnV);
		vel();
	}
	//to pass third gap.
	else if (object_Dected == 3)
	{
		//turn right
		robot.setVel2(left_turnV3, right_turnV3);
		vel();
		myGripper->gripOpen();
		if (sonar_read[1]<ObjectInRight2gap)
		{
			init_speed = slower_speed;
			object_Dected = 4;
			finish = 1;
			vel();
		}
	}
	// after pass third gap stop
	else if (finish == 1)
	{
		if (myGripper->getGripState() == 1) {
			x_coordinate = getBallLocation(cap);
			chaseBall(x_coordinate);
			vel();
			if (x_coordinate == 0 && (myLaser->currentReadingPolar(-10, 10, &angle) < Reach_ball_range)) {
				myGripper->gripClose();
				myGripper->liftUp();
				robot.setDeltaHeading(45);
			}
		}
		if (myLaser->currentReadingPolar(-10, 10, &angle) < Stop_detect) {
			finish = 2;
		}
	}
	else if (finish == 2) {
		robot.setVel2(stop, stop);
		vel();
		myGripper->gripOpen();
	}
}
void update(void)
{
	printf("state %d", object_Dected);
	// Set the velocity of the wheels.
	robot.setVel2(init_speed, init_speed);
	vel();
	//get theta
	th = robot.getPose().getTh();
	//get robot current position X and Y
	float xpos = robot.getPose().getX();
	float ypos = robot.getPose().getY();
	fprintf(postdata, "%f  %f  \n", xpos, ypos);
	// Get laser reading.
	for (a = min_degree; a<max_degree; a += degree_incr)
	{
		LaserToGobel();
	}
	// Get sonar reading.
	for (a = 0; a < sonarNum; a++)
	{
		verify_sonar();
		SonarToGobel();
		//to get gobel X and Y pose.
		X = Xr + xpos;
		Y = Yr + ypos;
		// open and write sonar data into txt
		if ((X - X_ver > ver_range) && (Y - Y_ver > ver_range)) {
			if (Y_ver>-sonar_min && X_ver>-sonar_min && X_ver<max_range  && Y_ver<max_range)
			{
				fprintf(opData, "%.1f   %.1f  \n", X_ver, Y_ver);
			}
		}
		else if (Y > -sonar_min && X > -sonar_min && X < sonar_max  && Y < sonar_max) {
			fprintf(opData, "%.1f   %.1f  \n", X, Y);
		}
		std::cout << "Sonar " << a << ": " << sonar_read[a] << "\n ";
	}

	// when detect first object on left flag change
	if (sonar_read[7] < first_object && object_Dected == 0)
	{
		object_Dected = 1;
	}
	turning();
}
int main(int argc, char **argv)
{
	myGripper = new ArGripper(&robot);
	// Initialisation
	Aria::init();
	// Initialize the camera.
	vcc4.init();
	// Wait for a while.
	ArUtil::sleep(2000);
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

	// Always try to connect to the first laser:
	argParser.addDefaultArgument("-connectLaser");

	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	// Add sonar.
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	// Connect laser.
	if (!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured laser.");
		Aria::logOptions();
		Aria::exit(1);
	}
	if (vcc4.getTilt() > -40) {
		std::cout << "Camera current tilt position: " << vcc4.getTilt()
			<< "\n Start tilting down" << std::endl;
		// Tilt the camera down 45 degrees to make it find the ball easier.
		vcc4.tilt(-45); // tilt down for detecting the ball
		ArUtil::sleep(1000);
	}

	ArPose space(inital_X, inital_Y, inital_Z); // Initial robot's odometry.
	robot.moveTo(space); //Moves the robot's idea of its position to this position.
						 // Tilt the camera down 45 degrees to make it find the ball easier.
	ArUtil::sleep(1000);
	// turn on the motors, turn off amigobot sounds
	// Load camera in OpenCV.
	if (!cap.open(0))
		Aria::exit(0);
	robot.enableMotors();
	myGripper->liftDown();
	//open file
	opData = fopen("sonar.txt", "a");
	fpData = fopen("laser.txt", "a");
	postdata = fopen("post.txt", "a");
	veldata = fopen("velocity_L.txt", "a");
	veldata2 = fopen("velocity_R.txt", "a");
	myGripper->gripClose();
	robot.addUserTask("update", 50, &updateCB);
	//robot.setCycleTime(100);
	robot.runAsync(true);
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();
	//close file
	fclose(opData);
	fclose(veldata);
	fclose(postdata);
	fclose(opData);
	Aria::exit(0);
}
