/*
 *  deltaKinematics.cpp
 *  Delta Sim
 *
 *  Created by Zak Henry on 8/04/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "deltaRobot.h"

DeltaRobot::DeltaRobot(float ieffectorSideLength){ //constructor
    
    //next bit is redundant now?
    
    baseSideMultiplier = 2; //2
    upperArmMultiplier = 1.5; //1.5
    lowerArmMultiplier = 1.5; //0.9
    
	effectorSideLength = ieffectorSideLength;
	baseSideLength = effectorSideLength*baseSideMultiplier;
	upperArmLength = effectorSideLength*upperArmMultiplier;
	lowerArmLength = effectorSideLength*lowerArmMultiplier;
   
     sqrt3 = sqrt(3.0);
     pi = PI;    // PI
     sin120 = sin(ofDegToRad(120));
     cos120 = cos(ofDegToRad(120));     
     tan60 = tan(ofDegToRad(60));
     sin30 = sin(ofDegToRad(30));
     tan30 = tan(ofDegToRad(30));
    
	cout << "Delta Robot class instantiated \n";
}

void DeltaRobot::update(){
}

int DeltaRobot::calcAngleYZ(float x0, float y0, float z0, float &theta) { //returns 0 if ok, -1 if not
	float y1 = -0.5 * tan30 * baseSideLength; // f/2 * tan 30
//    cout << y1 << "\n";
	y0 -= 0.5 * tan30 * effectorSideLength;    // shift center to edge
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

int DeltaRobot::calcInverse(float x0, float y0, float z0, float &theta0, float &theta1, float &theta2) {
    float t0, t1, t2;
	int status = calcAngleYZ(x0, y0, z0, t0);
	if (status == 0){
		status = calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, t1);  // rotate coords to +120 deg
	}
	if (status == 0){
		status = calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, t2);  // rotate coords to -120 deg
	}
    
    if ((t0>-90)&&(t0<90)&&(t1>-90)&&(t1<90)&&(t2>-90)&&(t2<90)){
        theta0 = t0;
        theta1 = t1;
        theta2 = t2;
    }else{
        status = -1;
//        cout <<"position impossible (theta0 tried to be "<<t0<<", theta1 tried to be "<<t1<<", theta2 tried to be "<<t2<<")\n";
    }
    
	return status;
}

int DeltaRobot::calcForward(float theta0, float theta1, float theta2, float &x0, float &y0, float &z0) {
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
    if (z0>0){
        return -1;
    }
    
	x0 = (a1*z0 + b1)/dnm;
	y0 = (a2*z0 + b2)/dnm;
	return 0;
}

int DeltaRobot::setCartesianPosition(float x, float y, float z){
    
    int result = calcInverse(x, y, z, theta0, theta1, theta2);
    
    if (result != 0){
//        cout << "Position is not possible\n";
    }else{
        effectorX = x;
        effectorY = y;
        effectorZ = z;
    }
    
    return result;
}

int DeltaRobot::setAngles(float theta0, float theta1, float theta2){
    int result = calcForward(theta0, theta1, theta2, effectorX, effectorY, effectorZ);
    
    if (result != 0){
        cout << "Position is not possible\n";
    }
    
    return result;
}

bool DeltaRobot::positionIsPossible(float x0, float y0, float z0){
	float a, b, c;
	//cout << "a: " << a << " b: " << b << " c: " << c << "|| x0: " << x0 << " y0: " << y0 << " z0: " << z0 << "\n""\n";
	//cout << "calc inverse  = " << calcInverse(x0, y0, x0, a, b, c) <<"\n";
	if (calcInverse(x0, y0, z0, a, b, c)==0){
        if ((a>-90)&&(a<90)&&(b>-90)&&(b<90)&&(c>-90)&&(c<90)){
            return true;
        }
	}
    return false;
}

void DeltaRobot::rotateCoordAboutOrigin(float angle, float xInit, float yInit, float &xEnd, float &yEnd){
	xEnd = xInit*cos(ofDegToRad(angle))+yInit*sin(ofDegToRad(angle));
	yEnd = -xInit*sin(ofDegToRad(angle))+yInit*cos(ofDegToRad(angle));
}

void DeltaRobot::setCoordinatesToRobot(){
    glPushMatrix();
    
    
    glTranslatef(ofGetWidth()/2,ofGetHeight()/2-200,50); //moves robot to coordinates to centre (ish) of scene
}
void DeltaRobot::releaseCoordinatesFromRobot(){
    glPopMatrix();
}

void DeltaRobot::drawRobot(){
    
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
    
        glBegin(GL_LINES); //theta0 marker
        glVertex3f(0, 0, -baseDistanceFromAxis);
        glVertex3f(0, 30, -baseDistanceFromAxis);
        glEnd();
    
        glBegin(GL_LINES); //theta0 upper arm
        glVertex3f(0, 0, -baseDistanceFromAxis);
        glVertex3f(0, -sin(ofDegToRad(theta0))*upperArmLength, -(cos(ofDegToRad(theta0))*upperArmLength)-baseDistanceFromAxis);
        glEnd();
        
        ofSetColor(0, 255, 255);
        glBegin(GL_LINES); //theta0 forearm
        glVertex3f(0, -sin(ofDegToRad(theta0))*upperArmLength, -(cos(ofDegToRad(theta0))*upperArmLength)-baseDistanceFromAxis);
        glVertex3f(effectorX, effectorZ, effectorY-effectorDistanceFromAxis);
        glEnd();
    
        glRotatef(120, 0, 1, 0);
        
        ofSetColor(255, 0, 255);
        glBegin(GL_LINES); //theta1 upper arm
        glVertex3f(0, 0, -tan(ofDegToRad(30))*baseSideLength/2);
        glVertex3f(0, -sin(ofDegToRad(theta1))*upperArmLength, -(cos(ofDegToRad(theta1))*upperArmLength)-baseDistanceFromAxis); //im suspicious of this line
        glEnd();
        
        ofSetColor(0, 255, 255);
        glBegin(GL_LINES); //theta1 forearm
        glVertex3f(0, -sin(ofDegToRad(theta1))*upperArmLength, -(cos(ofDegToRad(theta1))*upperArmLength)-baseDistanceFromAxis);
        float rotEffectX, rotEffectY;
        rotateCoordAboutOrigin(-120, effectorX, effectorY, rotEffectX, rotEffectY);
        glVertex3f(rotEffectX, effectorZ, rotEffectY-effectorDistanceFromAxis); //ignore middle one, that is the height
        glEnd();
        
        glRotatef(-240, 0, 1, 0);
    
        ofSetColor(255, 0, 255);
        glBegin(GL_LINES); //theta2 upper arm
        glVertex3f(0, 0, -tan(ofDegToRad(30))*baseSideLength/2);
        glVertex3f(0, -sin(ofDegToRad(theta2))*upperArmLength, -(cos(ofDegToRad(theta2))*upperArmLength)-baseDistanceFromAxis);
        glEnd();
        
        ofSetColor(100, 100, 255);
        glBegin(GL_LINES); //theta2 forearm //there is something seriously wrong with the forearms - they change in length for some reason
        glVertex3f(0, -sin(ofDegToRad(theta2))*upperArmLength, -(cos(ofDegToRad(theta2))*upperArmLength)-baseDistanceFromAxis);
        rotateCoordAboutOrigin(-240, effectorX, effectorY, rotEffectX, rotEffectY);
        glVertex3f(rotEffectX, effectorZ, rotEffectY-effectorDistanceFromAxis); //ignore middle one, that is the height
        glEnd();
        
        glPopMatrix();
        
        
        glPushMatrix();
        glTranslatef(effectorX, effectorZ, effectorY); // y and z swapped
        
    //    cout<< "z:"<<effectorZ<<"\n";
        
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

void DeltaRobot::calculateWorkingPointCloud(){ //could be really nice if this was threaded separately to main program ?
    
//    cout << "Calculating working space point cloud, please wait...\n";
    
    workingPointCloud.clear(); //wipe existing point cloud if present

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
    
    
//    cout << "Finished working point cloud calculation, "<<workingPointCloud.size()<<" points added\n";
}

void DeltaRobot::drawWorkingPointCloud(){
    ofSetColor(0, 0, 255);
    glPointSize(1.5);
    glBegin(GL_POINTS);
    
    for (int i=0; i<workingPointCloud.size(); i++){
        glVertex3f(workingPointCloud[i].x, workingPointCloud[i].z, workingPointCloud[i].y);
//        cout << "Point drawn in position ("<<workingPointCloud[i].x<<","<<workingPointCloud[i].y<<","<<workingPointCloud[i].z<<")\n";
    }
    
    glEnd();
    
}

void DeltaRobot::changeProportions(float ibaseSideMultiplier, float iupperArmMultiplier, float ilowerArmMultiplier){
    
    baseSideMultiplier = ibaseSideMultiplier;
    upperArmMultiplier = iupperArmMultiplier;
    lowerArmMultiplier = ilowerArmMultiplier;
    
    baseSideLength = effectorSideLength*baseSideMultiplier;
	upperArmLength = effectorSideLength*upperArmMultiplier;
	lowerArmLength = effectorSideLength*lowerArmMultiplier;
    
    
    setCartesianPosition(effectorX, effectorY, effectorZ);
    calculateCartesianPointCloud();
    calculateWorkingPointCloud();
    
//    cout << "New proportions ("<<baseSideMultiplier<<","<<upperArmMultiplier<<","<<lowerArmMultiplier<<") \n\n";
}

void DeltaRobot::calculateCartesianPointCloud(){ 
    
    cartesianPointCloud.clear();
    
    int minX, minY, minZ, maxX, maxY, maxZ;
    
    int increment = 20;
    
    maxZ = 0; //no point can be above the base
//    minZ = -(upperArmLength+lowerArmLength);
    minZ = -500;
    
//    maxX = upperArmLength+lowerArmLength; //it is in reality slightly less
    maxX = 500; //it is in reality slightly less
    maxY = maxX;
    
    minY = minX = -(maxX);
    
//    cout << "minX: "<<minX<<", minY: "<<minY<<", minZ: "<<minZ<<", maxX: "<<maxX<<", maxY: "<<maxY<<", maxZ: "<<maxZ<<"\n";
    
    for (int xInc = minX; xInc<maxX; xInc+=increment) {
        for (int yInc = minY; yInc<maxY; yInc+=increment) {
            for (int zInc = minZ; zInc<maxZ; zInc+=increment) {
                
                if (positionIsPossible(xInc, yInc, zInc)){
                    workingPoint newPoint;
                    newPoint.x = xInc+ofRandom(-5, 5);
                    newPoint.y = yInc+ofRandom(-5, 5);
                    newPoint.z = zInc+ofRandom(-5, 5);
                    cartesianPointCloud.push_back(newPoint);
                }
                
                
            }
        }
    }
    
//    cout << "Finished cartesian point cloud calculation, "<<cartesianPointCloud.size()<<" points added\n";
    
}

float DeltaRobot::calculateCartesianPointCloudSize(float baseSideMultiplier, float upperArmMultiplier, float lowerArmMultiplier, float&elapsedTime){ //main fitness function for GA
    
    clock_t tStart = clock();
    
    
    
//    changeProportions(baseSideMultiplier, upperArmMultiplier, lowerArmMultiplier);
    
//    calculateCartesianPointCloud();
//    float fitness = -1;

    /*if (workingPointCloud.size()>0&&cartesianPointCloud.size()>0){
        fitness = (float)cartesianPointCloud.size()/((float)workingPointCloud.size());
        cout << "workingPointCloud.size(): "<<(float)workingPointCloud.size()<<" cartesianPointCloud.size(): "<<(float)cartesianPointCloud.size()<<" calculated fitness to be: "<<fitness<<" \n";
    }*/
        
    
    
//    float fitness = 200-50*powf(((upperArmMultiplier*2)-powf((baseSideMultiplier*2), 2)), 2)+powf((1-(lowerArmMultiplier*2)), 2)+5*powf(upperArmMultiplier*5, 3); //Rosenbrock's banana function modified
    
//    float fitness = 200.0-(powf(powf(baseSideMultiplier, 2)+upperArmMultiplier-11, 2)+powf((baseSideMultiplier+powf(upperArmMultiplier, 2)-7), 2)); //Himmelblau's function modified to give maximums at 200
    
    float fitness = 200.0-(powf(powf((baseSideMultiplier*10)-5, 2)+(upperArmMultiplier*10)-5-11, 2)+powf(((baseSideMultiplier*10)-5+powf((upperArmMultiplier*10)-5, 2)-7), 2))-10*(powf((lowerArmMultiplier*10)-5, 2)); //Himmelblau's modified function to give a 4d graph. 4 maximums give 200

//    float fitness = 200-80*sin(baseSideMultiplier*80)-40*sin(upperArmMultiplier*50)-80*sin(lowerArmMultiplier*10);
    
    elapsedTime = (double)(clock() - tStart)/CLOCKS_PER_SEC;
    
    return fitness;
}

void DeltaRobot::drawCartesianPointCloud(){
    ofSetColor(0, 255, 0);
    glPointSize(1.5);
    glBegin(GL_POINTS);

    for (int i=0; i<cartesianPointCloud.size(); i++){
        glVertex3f(cartesianPointCloud[i].x, cartesianPointCloud[i].z, cartesianPointCloud[i].y);
        //        cout << "Point drawn in position ("<<cartesianPointCloud[i].x<<","<<cartesianPointCloud[i].y<<","<<cartesianPointCloud[i].z<<")\n";
    }
    
    glEnd();
    
}


