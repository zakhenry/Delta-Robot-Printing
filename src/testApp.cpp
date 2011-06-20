#include "testApp.h"

float effectorSideLength =  115.0; //every dimension is based off this master length

bool showWorkPointCloud = false;
bool showCartesianPointCloud = false;

DeltaRobot deltaRobot(effectorSideLength);

float currentTouchScale = 0;
float current3TouchHeight, current2TouchHeight = 0;
float zoom = 1;

bool tumble = false;

float effectorCursorX, effectorCursorY, effectorCursorZ = 0;
bool cursorPositionPossible = false;
bool runSteppersWithCursor = false;



//--------------------------------------------------------------
void testApp::setup(){	
    
    // --- add the listeners
    ofAddListener(pad.update, this, &testApp::padUpdates);
    ofAddListener(pad.touchAdded, this, &testApp::newTouch);
    ofAddListener(pad.touchRemoved, this, &testApp::removedTouch);
    
//	ofBackground(250, 250, 250);
	ofBackground(255, 255, 255);
    ofSetCircleResolution(50);

//	ofSetVerticalSync(true);

    //some model / light stuffx

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glClear (GL_COLOR_BUFFER_BIT);
    glEnable (GL_BLEND);
    glEnable (GL_POLYGON_SMOOTH);
//    glDisable (GL_DEPTH_TEST);
    glLineWidth(4.0);
    
    
    //fog
    GLfloat fogColor[4] = {1,1,1, 1.0};
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogf(GL_FOG_DENSITY, 0.2);
    glHint(GL_FOG_HINT, GL_DONT_CARE);
    glFogf(GL_FOG_START, 500);
    glFogf(GL_FOG_END, 4000);
    glEnable(GL_FOG);
    
    franklinBook.loadFont("frabk.ttf", 32);
    
    mouseY = 220; //initial setup so the world will be on an orientation that makes sense
    
    cout <<"testApp setup complete\n";

}

//--------------------------------------------------------------
void testApp::update(){
    
//    cout << "Serial output: "<<serial.readByte()<< "\n";
    
    deltaRobot.update();
    
//    cout <<"current2TouchHeight: "<<current2TouchHeight<<"\n";
//cout <<"zoom"<<zoom<<"\n";
}



void drawGrid(int spacing, int lines, int zDepth){
    for (int i=-(lines/2); i<(lines/2); i++){
        glBegin(GL_LINES);
        glVertex3f(i*spacing, zDepth, -(spacing*lines)/2);
        glVertex3f(i*spacing, zDepth, spacing*lines/2);
        glEnd();
    }
    
    for (int i=-(lines/2); i<(lines/2); i++){
        glBegin(GL_LINES);
        glVertex3f(-(spacing*lines)/2, zDepth, i*spacing);
        glVertex3f(spacing*lines/2, zDepth, i*spacing);
        glEnd();
    }
}

//--------------------------------------------------------------
void testApp::draw(){
    
    glPushMatrix();
    
    
	glTranslatef(ofGetWidth()/2,ofGetHeight()/2,0);
    
	//tumble according to mouse
    if (!tumble){
        glRotatef(-mouseY,1,0,0);
        glRotatef(-mouseX,0,1,0);
        
//        cout << "mousex: "<<mouseX<<" mouseY: "<<mouseY<<"\n";
    }else{
        glRotatef(-220+sin(ofGetElapsedTimef()/3)*20,1,0,0);
        glRotatef(ofGetElapsedTimef()*15,0,1,0);
    }
	glScalef(zoom, zoom, zoom);
    
    
	glTranslatef(-ofGetWidth()/2,0,0);
	
  
    
    deltaRobot.setCoordinatesToRobot();
        
    /*ground plane*/
    
    ofSetColor(0xdddddd);
    glLineWidth(1);
    drawGrid(50, 150, -500);
    glLineWidth(4);
    
        
        if (pathLoader.currentPathFile.points.size()>0){
            pathLoader.drawCurrentPath(true);
        }
    
        deltaRobot.drawRobot();
        
        if (deltaRobot.workingPointCloud.size()>0&&showWorkPointCloud){
            deltaRobot.drawWorkingPointCloud();
        }
        
        if (deltaRobot.cartesianPointCloud.size()>0&&showCartesianPointCloud){
            deltaRobot.drawCartesianPointCloud();
        }
        
    /*Effector cursor*/
    glPointSize(5.0);
    glBegin(GL_POINTS);
    if (cursorPositionPossible){
        ofSetColor(0x000000);  
    }else{
        ofSetColor(0xff0000);
    }
    
    glVertex3f(effectorCursorX, effectorCursorZ, effectorCursorY);
    glEnd();
        
        deltaRobot.releaseCoordinatesFromRobot();
    
    glPopMatrix();
            
    addDial(100, ofGetHeight()-100, deltaRobot.theta0, "1");
    addDial(250, ofGetHeight()-100, deltaRobot.theta1, "2");
    addDial(400, ofGetHeight()-100, deltaRobot.theta2, "3");
    
    glPushMatrix();
    
    glTranslatef(100,100,0);
    
    ofSetColor(0xcccccc);
    
    ofCircle(0, 0, 50);
    
	//tumble according to mouse
    if (!tumble){
        glRotatef(-mouseY,1,0,0);
        glRotatef(-mouseX,0,1,0);
    }else{
        glRotatef(ofGetElapsedTimef()*13,1,0,0);
        glRotatef(ofGetElapsedTimef()*11,0,1,0);
        glRotatef(ofGetElapsedTimef()*7,0,0,1);
    }
    
    
    ofSetColor(0xff0000);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 20);
    glEnd();
    
    ofSetColor(0x0000ff);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(20, 0, 0);
    glEnd();
    
    ofSetColor(0x00ff00);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 20, 0);
    glEnd();
    
    
    
    glPopMatrix();
    
    
    ofSetColor(0xcccccc);
    ofPushMatrix();
    ofTranslate(ofGetWidth()-100, ofGetHeight()-100);
    ofCircle(0, 0, 60);
    ofSetColor(0xffffff);
    ofScale(1,1,1);
    ofTranslate(-35, 10);
    franklinBook.drawString(ofToString((int)ofGetFrameRate()), 0,0);
    ofPopMatrix();


}

//--------------------------------------------------------------
void testApp::keyPressed  (int key){ 
	
	switch (key) {
            
            if (deltaRobot.stepperControl.robotReadyForData()){
                
            
		case 'd':
            cursorPositionPossible = (deltaRobot.setCartesianPosition(effectorCursorX+=5, effectorCursorY, effectorCursorZ, runSteppersWithCursor)==0);
			break;
		case 'a':
//            deltaRobot.setCartesianPosition(deltaRobot.effectorX-5, deltaRobot.effectorY, deltaRobot.effectorZ, true);
            cursorPositionPossible = (deltaRobot.setCartesianPosition(effectorCursorX-=5, effectorCursorY, effectorCursorZ, runSteppersWithCursor)==0);
			break;
		case 'e':
//            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY+5, deltaRobot.effectorZ, true);
            cursorPositionPossible = (deltaRobot.setCartesianPosition(effectorCursorX, effectorCursorY+=5, effectorCursorZ, runSteppersWithCursor)==0);
			break;
		case 'q':
//            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY-5, deltaRobot.effectorZ, true);
            cursorPositionPossible = (deltaRobot.setCartesianPosition(effectorCursorX, effectorCursorY-=5, effectorCursorZ, runSteppersWithCursor)==0);
			break;
		case 's':
//            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY, deltaRobot.effectorZ+5, true);
            cursorPositionPossible = (deltaRobot.setCartesianPosition(effectorCursorX, effectorCursorY, effectorCursorZ+=5, runSteppersWithCursor)==0);
			break;
		case 'w':
//            deltaRobot.setCartesianPosition(deltaRobot.effectorX, deltaRobot.effectorY, deltaRobot.effectorZ-5, true);
            cursorPositionPossible = (deltaRobot.setCartesianPosition(effectorCursorX, effectorCursorY, effectorCursorZ-=5, runSteppersWithCursor)==0);
			break;
                
                }

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

            
        case ' ':
            tumble = !tumble;
            break;
            
        case 'r':
            deltaRobot.runPath(pathLoader.currentPathFile);
            break;
            
        case 13:
            runSteppersWithCursor = !runSteppersWithCursor;
            break;
            
			
		default:
            cout <<"Key ("<<key<<") pressed\n";
			break;
	}
	
	cout << "Theta 0: " << deltaRobot.theta0 << " Theta 1: " << deltaRobot.theta1 <<" Theta 2: " << deltaRobot.theta2 << "\n";
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
                    
                    current2TouchHeight = averageTouchHeight;
                    
                    zoom -= alteration/50;
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

void testApp::addDial(int x, int y, float rotation, string label){
    
    ofSetColor(0xcccccc);
    ofPushMatrix();
    ofTranslate(x, y);
    ofCircle(0, 0, 60);
    ofSetColor(0xffffff);
    ofCircle(0, 0, 10);
    ofSetRectMode(OF_RECTMODE_CENTER);
    
    ofPushMatrix();
    ofTranslate(-50, 13);
    franklinBook.drawString(label, 0,0);
    ofPopMatrix();
    
    ofRotateZ(rotation);
    ofPushMatrix();
    ofTranslate(25, 0);
    ofRect(0, 0, 50, 5);
    ofPopMatrix();
    
    ofSetColor(0xffffff);
    ofScale(0.6,0.6,1);
    ofTranslate(20, -10);
    franklinBook.drawString(ofToString((int)rotation), 0,0);
    
    ofPopMatrix();
    
}

