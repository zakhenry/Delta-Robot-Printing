#include "testApp.h"

float effectorSideLength =  115.0; //every dimension is based off this master length

bool showWorkPointCloud = false;

DeltaKinematics deltaRobot(effectorSideLength);

GeneticAlgorithm ga;

ofSerial serial;

bool drawRobot = true;

//--------------------------------------------------------------
void testApp::setup(){	
	ofBackground(0,0,0);
		
//	ofSetVerticalSync(true);

    //some model / light stuff
    glEnable (GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);
    
    serial.enumerateDevices();
    
    if(!serial.setup(/*"tty.usbserial-A9007Mbm"*/5, 9600)){
        
        printf("Serial setup failed!");
        
    }

}

//--------------------------------------------------------------
void testApp::update(){
    
//    cout << "Serial output: "<<serial.readByte()<< "\n";

}

//--------------------------------------------------------------
void testApp::draw(){
    
	glTranslatef(ofGetWidth()/2,ofGetHeight()/2,0);
	//tumble according to mouse
	glRotatef(-mouseY,1,0,0);
	glRotatef(-mouseX,0,1,0);
	glTranslatef(-ofGetWidth()/2,0,0);
	
    /*
    ofSetColor(255, 0, 0);
	glBegin(GL_LINE);
        glVertex3f(-100, 0, 0);
        glVertex3f(100, 0, 0);
    glEnd();
    */
    
    ofSetColor(100, 100, 100);
    ofDrawBitmapString("fps: "+ofToString(ofGetFrameRate(), 2), 200, -200);
    
/*    
	 //fake back wall
    ofSetColor(50, 50, 50);
    glBegin(GL_QUADS);
        glVertex3f(0.0, -ofGetHeight()/2, -600);
        glVertex3f(ofGetWidth(), -ofGetHeight()/2, -600);
        glVertex3f(ofGetWidth(), ofGetHeight()/2, -600);
        glVertex3f(0, ofGetHeight()/2, -600);
    glEnd();

    //fake wall
    ofSetColor(20, 20, 20);
    glBegin(GL_QUADS);
        glVertex3f(0.0, -ofGetHeight()/2, 0);
        glVertex3f(ofGetWidth(), -ofGetHeight()/2, 0);
        glVertex3f(ofGetWidth(), -ofGetHeight()/2, -600);
        glVertex3f(0, -ofGetHeight()/2, -600);
    glEnd();
*/    
    if (drawRobot){
        deltaRobot.setCoordinatesToRobot();
        deltaRobot.drawRobot();
        
        if (deltaRobot.workingPointCloud.size()>0&&showWorkPointCloud){
            deltaRobot.drawWorkingPointCloud();
        }
        
        deltaRobot.releaseCoordinatesFromRobot();
            
    }else{
        ga.drawSearchSpace();
        ga.drawCurrentPopulation();
    }
    
    
    
    

}

//--------------------------------------------------------------
void testApp::keyPressed  (int key){ 
	
	switch (key) {
		case 'd':
//			xMove +=5;
            deltaRobot.setCartesianPosition(deltaRobot.effectorX+=5, deltaRobot.effectorY, deltaRobot.effectorZ);
			break;
		case 'a':
            deltaRobot.setCartesianPosition(deltaRobot.effectorX-=5, deltaRobot.effectorY, deltaRobot.effectorZ);
			break;
		case 'e':
            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY+=5, deltaRobot.effectorZ);
			break;
		case 'q':
            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY-=5, deltaRobot.effectorZ);
			break;
		case 's':
            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY, deltaRobot.effectorZ+=5);
			break;
		case 'w':
            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY, deltaRobot.effectorZ-=5);
			break;
            
		case 32:
//            deltaRobot.directControl = !deltaRobot.directControl;
			break;
		case 't':
            deltaRobot.setAngles(deltaRobot.theta0-=5, deltaRobot.theta1, deltaRobot.theta2);
			break;
		case 'g':
            deltaRobot.setAngles(deltaRobot.theta0+=5, deltaRobot.theta1, deltaRobot.theta2);
			break;
		case 'y':
            deltaRobot.setAngles(deltaRobot.theta0, deltaRobot.theta1-=5, deltaRobot.theta2);
			break;
		case 'h':
            deltaRobot.setAngles(deltaRobot.theta0, deltaRobot.theta1+=5, deltaRobot.theta2);
			break;
		case 'u':
            deltaRobot.setAngles(deltaRobot.theta0, deltaRobot.theta1, deltaRobot.theta2-=5);
			break;
		case 'j':
            deltaRobot.setAngles(deltaRobot.theta0, deltaRobot.theta1, deltaRobot.theta2+=5);
			break;
        case 'p':
            if (!(deltaRobot.workingPointCloud.size()>0)){
                deltaRobot.calculateWorkingPointCloud();
            }
            showWorkPointCloud = !showWorkPointCloud;
			break;
        case 'z':
            deltaRobot.changeProportions(deltaRobot.baseSideMultiplier+=0.1, deltaRobot.upperArmMultiplier, deltaRobot.lowerArmMultiplier);
			break;
        case 'x':
            deltaRobot.changeProportions(deltaRobot.baseSideMultiplier-=0.1, deltaRobot.upperArmMultiplier, deltaRobot.lowerArmMultiplier);
			break;
        case 'c':
            deltaRobot.changeProportions(deltaRobot.baseSideMultiplier, deltaRobot.upperArmMultiplier+=0.1, deltaRobot.lowerArmMultiplier);
			break;
        case 'v':
            deltaRobot.changeProportions(deltaRobot.baseSideMultiplier, deltaRobot.upperArmMultiplier-=0.1, deltaRobot.lowerArmMultiplier);
			break;
        case 'b':
            deltaRobot.changeProportions(deltaRobot.baseSideMultiplier, deltaRobot.upperArmMultiplier, deltaRobot.lowerArmMultiplier+=0.1);
			break;
        case 'n':
            deltaRobot.changeProportions(deltaRobot.baseSideMultiplier, deltaRobot.upperArmMultiplier, deltaRobot.lowerArmMultiplier-=0.1);
			break;
            
        case 'o':
            ga.run();
            break;
            
        case 'l':
            ga.calculateSearchSpace();
            break;
            
        case 'k':
            drawRobot = !drawRobot;
            break;
            
        case 'm':
            break;
            
			
		default:
			break;
	}
	
//	cout << "Theta 0: " << deltaRobot.theta0 << " Theta 1: " << deltaRobot.theta1 <<" Theta 2: " << deltaRobot.theta2 << "\n";
//	cout << "EffectorX: " << deltaRobot.effectorX << " EffectorY: " << deltaRobot.effectorY << " EffectorZ: " << deltaRobot.effectorZ << "\n\n\n";
    
}

//--------------------------------------------------------------
void testApp::keyReleased  (int key){ 
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}
