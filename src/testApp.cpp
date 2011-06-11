#include "testApp.h"

float effectorSideLength =  115.0; //every dimension is based off this master length

bool showWorkPointCloud = false;
bool showCartesianPointCloud = false;

DeltaRobot deltaRobot(effectorSideLength);

GeneticAlgorithm ga;

bool drawRobot = true;
float currentTouchScale = 0;
float current3TouchHeight, current2TouchHeight = 0;

bool tumble = false;

float fitnessThreshold = 100; //mod himmelblau
float fitnessColorScale = 400;
//float fitnessThreshold = 5000; //delta

//--------------------------------------------------------------
void testApp::setup(){	
    
    // --- add the listeners
    ofAddListener(pad.update, this, &testApp::padUpdates);
    ofAddListener(pad.touchAdded, this, &testApp::newTouch);
    ofAddListener(pad.touchRemoved, this, &testApp::removedTouch);
    
	ofBackground(0,0,0);
		
//	ofSetVerticalSync(true);

    //some model / light stuff
    glEnable (GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);
    
    
    
    float x0 = 0;
    float y0 = 0;
    float z0 = -200;
    float theta;
    deltaRobot.calcAngleYZ(x0, y0, z0, theta);
    
    cout << "Deltarobot test function calcAngleYZ(float "<<x0<<", float "<<y0<<", float "<<z0<<", float &"<<theta<<")\n";
    
    
    
    

}

//--------------------------------------------------------------
void testApp::update(){
    
//    cout << "Serial output: "<<serial.readByte()<< "\n";
    
    deltaRobot.update();
    
//    cout <<"current2TouchHeight: "<<current2TouchHeight<<"\n";

}

//--------------------------------------------------------------
void testApp::draw(){
    
	glTranslatef(ofGetWidth()/2,ofGetHeight()/2,0);
	//tumble according to mouse
    if (!tumble){
        glRotatef(-mouseY,1,0,0);
        glRotatef(-mouseX,0,1,0);
    }else{
        glRotatef(ofGetElapsedTimef()*13,1,0,0);
        glRotatef(ofGetElapsedTimef()*11,0,1,0);
        glRotatef(ofGetElapsedTimef()*7,0,0,1);
//        glRotatef(sin(ofGetElapsedTimef()/3)*100,1,0,0);
//        glRotatef(sin(ofGetElapsedTimef()/3-(PI/3))*100,0,1,0);
//        glRotatef(sin(ofGetElapsedTimef()/3+(PI/3))*100,0,0,1);
    }
	
    
    
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
        
        if (deltaRobot.cartesianPointCloud.size()>0&&showCartesianPointCloud){
            deltaRobot.drawCartesianPointCloud();
        }
        
        if (pathLoader.currentPathFile.points.size()>0){
            pathLoader.drawCurrentPath(true);
        }
        
        deltaRobot.releaseCoordinatesFromRobot();
            
    }else{
        ga.drawSearchSpace(fitnessThreshold, fitnessColorScale);
        ga.drawCurrentPopulation(fitnessThreshold, fitnessColorScale);
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
            
        case 'p':
            if (!(deltaRobot.workingPointCloud.size()>0)){
                deltaRobot.calculateWorkingPointCloud();
            }
            showWorkPointCloud = !showWorkPointCloud;
            showCartesianPointCloud = false;
			break;
            
        case '[':
            if (!(deltaRobot.cartesianPointCloud.size()>0)){
                deltaRobot.calculateCartesianPointCloud();
            }
            showCartesianPointCloud = !showCartesianPointCloud;
            showWorkPointCloud = false;
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
            
        case ' ':
            tumble = !tumble;
            break;
            
        case 'r':
            deltaRobot.runPath(pathLoader.currentPathFile);
            break;
            
			
		default:
            cout <<"Key ("<<key<<") pressed\n";
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

void testApp::padUpdates(int & touchCount) {
    //    cout << "pad updates & has "<<t<<" touches\n";
    
        
        if (touchCount==2){ //scaling
            MTouch t1,t2;
            if (pad.getTouchAt(0,&t1) && pad.getTouchAt(1,&t2) ){
                float averageTouchHeight = ((t1.y)+(t2.y))/2;
                
                //            cout << "The averageTouchHeight is: " << averageTouchHeight << "\n";
                
                if (abs(averageTouchHeight-current2TouchHeight)>0.05){
                    float alteration;
                    
                    if (averageTouchHeight>current2TouchHeight){
                        alteration++;
                    }else{
                        alteration--;
                    }
                    
                    fitnessColorScale += alteration*3;
                    
//                    cout << "fitness color scale:"<<fitnessColorScale<<"\n";
                    
                    current2TouchHeight = averageTouchHeight;
                }

            }
        }else if (touchCount==3){
            
            MTouch t1, t2, t3;
            
            if (pad.getTouchAt(0,&t1) && pad.getTouchAt(1,&t2) && pad.getTouchAt(2,&t3)){
                
                float averageTouchHeight = ((t1.y)+(t2.y)+(t3.y))/3;
                
                //            cout << "The averageTouchHeight is: " << averageTouchHeight << "\n";
                
                if (abs(averageTouchHeight-current3TouchHeight)>0.05){
                    float alteration;
                    
                    if (averageTouchHeight>current3TouchHeight){
                        alteration++;
                    }else{
                        alteration--;
                    }
                    
                    fitnessThreshold += alteration*10; //mod himmelblau
//                    fitnessThreshold += alteration*50000; //delta
                    
                    current3TouchHeight = averageTouchHeight;
                }
            }
        }
    
}

void testApp::newTouch(int & n) {
    //    cout << "++++++ a new touch"<<n<<"\n";
}

void testApp::removedTouch(int & r) {
    //    cout << "------ a removed touch"<<r<<"\n";
    currentTouchScale = 0;
    
}


float testApp::distanceBetweenTouches(MTouch t1, MTouch t2){
    return powf((pow(t1.x-t2.x, 2)+pow(t1.y-t2.y, 2)), 0.5);
}

