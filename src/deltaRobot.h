/*
 *  deltaKinematics.h
 *  Delta Sim
 *
 *  Created by Zak Henry on 8/04/11.
 *  Copyright 2011 Zak Henry. All rights reserved.
 *
 */

#ifndef _DELTA_ROBOT // if this class hasn't been defined, the program can define it
#define _DELTA_ROBOT // by using this if statement you prevent the class to be called more 
// than once which would confuse the compiler

#include "ofMain.h"
#include "pathLoader.h"
#include "serialConnection.h"

class SerialConnection; //forward declaration allows access to methods

class DeltaRobot {
	
public: //temporarily here so I can test functions out	
//	float /*e, f, re, rf*/;
    float effectorSideLength, baseSideLength, upperArmLength, lowerArmLength, effectorHeadDepth;
    
	// trigonometric constants
	float sqrt3, pi, sin120, cos120, tan60, sin30, tan30;
	
	int calcAngleYZ(float, float, float, float&); // helper function
    
    struct workingPoint{
        float x, y, z;
        void set(float _x, float _y, float _z){
            x = _x;
            y = _y;
            z = _z;
        }
    };
    
    SerialConnection serialConnection;
    
    vector<ofPoint>queuedWaypoints;
    void gotoNextWaypt();
    
    float distanceBetweenPoints(ofPoint a, ofPoint b);
    
    float unitSpeed; //units/second
	
//public:
    
    float baseSideMultiplier, upperArmMultiplier, lowerArmMultiplier; //these multipliers define the ratio of the proportions of the robot (to be worked out by genetic search function, and made static)
	void changeProportions(float baseSideMultiplier, float upperArmMultiplier, float lowerArmMultiplier);
    
	DeltaRobot(float effectorSideLength); //constructor
    
//    bool directControl;
    float effectorX, effectorY, effectorZ, theta0, theta1, theta2;
	
	int calcInverse(float, float, float, float&, float&, float&); // inverse kinematics
	int calcForward(float, float, float, float&, float&, float&); //forward kinematics
    
    int setCartesianPosition(float x, float y, float z, bool setSteppers); // inverse kinematics
    int setAngles(float theta0, float theta1, float theta2); //forward kinematics
	
	bool positionIsPossible(float, float, float);
	void rotateCoordAboutOrigin(float angle, float, float, float&, float&);
    
    void setCoordinatesToRobot();
    void releaseCoordinatesFromRobot();
    
    void drawRobot();
    void update();
    
//    vector<workingPoint>calculateWorkingPointCloud();
    
    bool workingPointsAreValid(vector<workingPoint> testPoints);
    
    vector<workingPoint>workingPointCloud;
    void calculateWorkingPointCloud();
    void drawWorkingPointCloud();
    
    vector<workingPoint>cartesianPointCloud;
    void calculateCartesianPointCloud();
    void drawCartesianPointCloud();
    
    vector<workingPoint>workingCubicSpaceLimits;
    void calculateWorkingCubicSpaceLimits();
    void drawWorkingCubicSpace();
    
    vector<workingPoint>workingPointsInCubicSpace;
    void calculateWorkingPointsInCubicSpace();
    void drawWorkingPointsInCubicSpace();
    
    float calculateCartesianPointCloudSize(float, float, float, float &elapsedTime); //this is the fitness function that the ga runs
    
    bool currentPathFileIsPossible(PathLoader::pathFile);
    void runPath(PathLoader::pathFile);
};



#endif 