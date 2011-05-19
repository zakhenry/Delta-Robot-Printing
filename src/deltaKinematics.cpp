/*
 *  deltaKinematics.cpp
 *  Delta Sim
 *
 *  Created by Zak Henry on 8/04/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "deltaKinematics.h"

DeltaKinematics::DeltaKinematics(float ieffectorSideLength){ //constructor
    
    baseSideMultiplier = 2;
    upperArmMultiplier = 1.5;
    lowerArmMultiplier = 0.9;
    
	effectorSideLength = ieffectorSideLength;
	baseSideLength = effectorSideLength*baseSideMultiplier;
	upperArmLength = effectorSideLength*upperArmMultiplier;
	lowerArmLength = effectorSideLength*lowerArmMultiplier;
    
    //some constants (helps keep calcs fast)
    
	sqrt3 = sqrt(3.0);
	pi = 3.141592653;    // PI
	sin120 = sqrt3/2.0;   
	cos120 = -0.5;        
	tan60 = sqrt3;
	sin30 = 0.5;
	tan30 = 1/sqrt3;
    
//    directControl = false; //start in cartesian control mode
    setAngles(0, 0, 0); //set arms to home position
    
    calculateWorkingPointCloud(); //calculate point cloud straight away
	
	cout << "Delta Kinematics calculator instantiated \n";
}

int DeltaKinematics::calcAngleYZ(float x0, float y0, float z0, float &theta) { //returns 0 if ok, -1 if not
	float y1 = -0.5 * 0.57735 * baseSideLength; // f/2 * tg 30
	y0 -= 0.5 * 0.57735 * effectorSideLength;    // shift center to edge
	// z = a + b*y
	float a = (x0*x0 + y0*y0 + z0*z0 +lowerArmLength*lowerArmLength - upperArmLength*upperArmLength - y1*y1)/(2*z0);
	float b = (y1-y0)/z0;
	// discriminant
	float d = -(a+b*y1)*(a+b*y1)+lowerArmLength*(b*b*lowerArmLength+lowerArmLength); 
	if (d < 0) return -1; // non-existing point
	float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
	float zj = a + b*yj;
	theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
	return 0;
}

int DeltaKinematics::calcInverse(float x0, float y0, float z0, float &theta0, float &theta1, float &theta2) {
	theta0 = theta1 = theta2 = 0;
	int status = calcAngleYZ(x0, y0, z0, theta0);
	if (status == 0){
		status = calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta1);  // rotate coords to +120 deg
	}
	if (status == 0){
		status = calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta2);  // rotate coords to -120 deg
	}
	return status;
}

int DeltaKinematics::calcForward(float theta0, float theta1, float theta2, float &x0, float &y0, float &z0) {
	float t = (baseSideLength-effectorSideLength)*tan30/2;
	float dtr = pi/(float)180.0;
	
	theta0 *= dtr;
	theta1 *= dtr;
	theta2 *= dtr;
	
	float y1 = -(t + lowerArmLength*cos(theta0));
	float z1 = -lowerArmLength*sin(theta0);
	
	float y2 = (t + lowerArmLength*cos(theta1))*sin30;
	float x2 = y2*tan60;
	float z2 = -lowerArmLength*sin(theta1);
	
	float y3 = (t + lowerArmLength*cos(theta2))*sin30;
	float x3 = -y3*tan60;
	float z3 = -lowerArmLength*sin(theta2);
	
	float dnm = (y2-y1)*x3-(y3-y1)*x2;
	
	float w1 = y1*y1 + z1*z1;
	float w2 = x2*x2 + y2*y2 + z2*z2;
	float w3 = x3*x3 + y3*y3 + z3*z3;
	
	// x = (a1*z + b1)/dnm
	float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
	
	// y = (a2*z + b2)/dnm;
	float a2 = -(z2-z1)*x3+(z3-z1)*x2;
	float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
	
	// a*z^2 + b*z + c = 0
	float a = a1*a1 + a2*a2 + dnm*dnm;
	float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - upperArmLength*upperArmLength);
	
	// discriminant
	float d = b*b - (float)4.0*a*c;
	if (d < 0) return -1; // non-existing point
	
	z0 = -(float)0.5*(b+sqrt(d))/a;
	x0 = (a1*z0 + b1)/dnm;
	y0 = (a2*z0 + b2)/dnm;
	return 0;
}

int DeltaKinematics::setCartesianPosition(float x, float y, float z){
    
    int result = calcInverse(x, y, z, theta0, theta1, theta2);
    
    if (result != 0){
        cout << "Position is not possible";
    }
    
    return result;
}

int DeltaKinematics::setAngles(float theta0, float theta1, float theta2){
    int result = calcForward(theta0, theta1, theta2, effectorX, effectorY, effectorZ);
    
    if (result != 0){
        cout << "Position is not possible";
    }
    
    return result;
}

bool DeltaKinematics::positionIsPossible(float x0, float y0, float z0, float d0, float d1, float d2){
	float a, b, c;
	a = d0;
	b = d1;
	c = d2;
	//cout << "a: " << a << " b: " << b << " c: " << c << "|| x0: " << x0 << " y0: " << y0 << " z0: " << z0 << "\n""\n";
	//cout << "calc inverse  = " << calcInverse(x0, y0, x0, a, b, c) <<"\n";
	if (calcInverse(x0, y0, z0, a, b, c)==0){
		return true;
	}else{
		return false;
	}
}

void DeltaKinematics::rotateCoordAboutOrigin(float angle, float xInit, float yInit, float &xEnd, float &yEnd){
	xEnd = xInit*cos(ofDegToRad(angle))+yInit*sin(ofDegToRad(angle));
	yEnd = -xInit*sin(ofDegToRad(angle))+yInit*cos(ofDegToRad(angle));
}

void DeltaKinematics::setCoordinatesToRobot(){
    glPushMatrix();
    
    
    glTranslatef(ofGetWidth()/2,ofGetHeight()/2-200,50); //moves robot to coordinates to centre (ish) of scene
}
void DeltaKinematics::releaseCoordinatesFromRobot(){
    glPopMatrix();
}

void DeltaKinematics::drawRobot(){
    
    ofSetColor(255, 0, 0);
	glBegin(GL_LINES); //x
    glVertex3f(0, 0, 0);
    glVertex3f(500, 0, 0);
    glEnd();
    
    ofSetColor(0, 255, 0);
	glBegin(GL_LINES); //y
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 500);
    glEnd();
    
    ofSetColor(0, 0, 255);
	glBegin(GL_LINES); //z
    glVertex3f(0, 0, 0);
    glVertex3f(0, 500, 0);
    glEnd();
    
	ofSetColor(255, 0, 255);
    
    glPushMatrix();
    float baseDistanceFromAxis = tan(ofDegToRad(30))*baseSideLength/2;
    float effectorDistanceFromAxis = tan(ofDegToRad(30))*effectorSideLength/2;
    glBegin(GL_LINES); //theta0 upper arm
    glVertex3f(0, 0, -baseDistanceFromAxis);
    glVertex3f(0, -sin(ofDegToRad(theta0))*upperArmLength, (-cos(ofDegToRad(theta0))*upperArmLength)-baseDistanceFromAxis);
    glEnd();
    
    glBegin(GL_LINES); //theta0 forearm
    glVertex3f(0, -sin(ofDegToRad(theta0))*upperArmLength, (-cos(ofDegToRad(theta0))*upperArmLength)-baseDistanceFromAxis);
    glVertex3f(effectorX, effectorZ, effectorY-effectorDistanceFromAxis);
    glEnd();	
    glRotatef(-120, 0, 1, 0);
    
    glBegin(GL_LINES); //theta1 upper arm
    glVertex3f(0, 0, -tan(ofDegToRad(30))*baseSideLength/2);
    glVertex3f(0, -sin(ofDegToRad(theta0))*upperArmLength, (-cos(ofDegToRad(theta0))*upperArmLength)-baseDistanceFromAxis);
    glEnd();
    glBegin(GL_LINES); //theta1 forearm
    glVertex3f(0, -sin(ofDegToRad(theta0))*upperArmLength, (-cos(ofDegToRad(theta0))*upperArmLength)-baseDistanceFromAxis);
	float rotEffectX, rotEffectY;
    rotateCoordAboutOrigin(120, effectorX, effectorY, rotEffectX, rotEffectY);
    glVertex3f(rotEffectX, effectorZ, rotEffectY-effectorDistanceFromAxis); //ignore middle one, that is the height
    glEnd();
	
    glRotatef(-120, 0, 1, 0);
	
    glBegin(GL_LINES); //theta2 upper arm
    glVertex3f(0, 0, -tan(ofDegToRad(30))*baseSideLength/2);
    glVertex3f(0, -sin(ofDegToRad(theta1))*upperArmLength, (-cos(ofDegToRad(theta1))*upperArmLength)-baseDistanceFromAxis);
    glEnd();
    
    glBegin(GL_LINES); //theta2 forearm //there is something seriously wrong with the forearms - they change in length for some reason
    glVertex3f(0, -sin(ofDegToRad(theta1))*upperArmLength, (-cos(ofDegToRad(theta1))*upperArmLength)-baseDistanceFromAxis);
	rotateCoordAboutOrigin(-120, effectorX, effectorY, rotEffectX, rotEffectY);
    glVertex3f(rotEffectX, effectorZ, rotEffectY-effectorDistanceFromAxis); //ignore middle one, that is the height
    glEnd();
	
	glPopMatrix();
	
	
	glPushMatrix();
	glTranslatef(effectorX, effectorZ, effectorY); // y and z swapped
	
	ofSetColor(255, 0, 0);
	glBegin(GL_TRIANGLES); //effector triangle
	glVertex3f(-effectorSideLength/2, 0, -tan(ofDegToRad(30))*effectorSideLength/2);
	glVertex3f(effectorSideLength/2, 0, -tan(ofDegToRad(30))*effectorSideLength/2);
	glVertex3f(0, 0, (sin(ofDegToRad(60))*effectorSideLength)-(tan(ofDegToRad(30))*effectorSideLength/2));
	glEnd();
	glPopMatrix();
	
	ofSetColor(0, 255, 0);
    glBegin(GL_TRIANGLES); //base triangle (placed down here so tranparency works)
    glVertex3f(-baseSideLength/2, 0, -tan(ofDegToRad(30))*baseSideLength/2);
    glVertex3f(baseSideLength/2, 0, -tan(ofDegToRad(30))*baseSideLength/2);
    glVertex3f(0, 0, (sin(ofDegToRad(60))*baseSideLength)-(tan(ofDegToRad(30))*baseSideLength/2));
    glEnd();
}


/*Workspace calculations*/


//vector<DeltaKinematics::workingPoint> DeltaKinematics::calculateWorkingPointCloud(){
void DeltaKinematics::calculateWorkingPointCloud(){ //could be really nice if this was threaded separately to main program ?
    
    cout << "Calculating working space point cloud, please wait...\n";
    
    workingPointCloud.erase(workingPointCloud.begin(), workingPointCloud.begin()+workingPointCloud.size()); //wipe existing point cloud if present

    int degreesToCheck = 180;
    int topValue = degreesToCheck/2;
    int bottomValue = -topValue;
    int increment = 10; //this is key to how many points are generated
    
    for (int testTheta0=bottomValue; testTheta0<=topValue; testTheta0+=increment){ //increment by 1 degree at a time
        for (int testTheta1=bottomValue; testTheta1<=topValue; testTheta1+=increment){ //increment by 1 degree at a time
            for (int testTheta2=bottomValue; testTheta2<=topValue; testTheta2+=increment){ //increment by 1 degree at a time
                workingPoint newPoint;
                
                int result = calcForward(testTheta0, testTheta1, testTheta2, newPoint.x, newPoint.y, newPoint.z);
                
                if (result == 0){
                    workingPointCloud.push_back(newPoint);
                }
                
            }
        }
    }
    
    cout << "Finished point positioning, finding magnitudes \n";
    
    
    
    
    cout << "Finished point cloud calculation, "<<workingPointCloud.size()<<" points added\n";
}

void DeltaKinematics::drawWorkingPointCloud(){
    ofSetColor(0, 0, 255);
    glPointSize(1.5);
    glBegin(GL_POINTS);
    
    for (int i=0; i<workingPointCloud.size(); i++){
        glVertex3f(workingPointCloud[i].x, workingPointCloud[i].z, workingPointCloud[i].y);
//        cout << "Point drawn in position ("<<workingPointCloud[i].x<<","<<workingPointCloud[i].y<<","<<workingPointCloud[i].z<<")\n";
    }
    
    glEnd();
    
}

void DeltaKinematics::changeProportions(float ibaseSideMultiplier, float iupperArmMultiplier, float ilowerArmMultiplier){
    
    baseSideMultiplier = ibaseSideMultiplier;
    upperArmMultiplier = iupperArmMultiplier;
    lowerArmMultiplier = ilowerArmMultiplier;
    
    baseSideLength = effectorSideLength*baseSideMultiplier;
	upperArmLength = effectorSideLength*upperArmMultiplier;
	lowerArmLength = effectorSideLength*lowerArmMultiplier;
    
    calculateWorkingPointCloud(); //need to recalculate the point cloud because conditions have changed
    
    cout << "New proportions ("<<baseSideMultiplier<<","<<upperArmMultiplier<<","<<lowerArmMultiplier<<") \n";
}


int DeltaKinematics::calculatePointCloudSize(int inputX, int inputY){
    cout << "external fitness function ran\n";
    cout << "inputs multiplied: "<<inputX*inputY<<"\n";
}

