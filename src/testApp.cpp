#include "testApp.h"

float effectorSideLength =  320.0; //every dimension is based off this master length

bool showWorkPointCloud = false;
bool showCartesianPointCloud = false;
bool showWorkingPointsInCubicSpace = false;
bool showWorkingCube = false;

DeltaRobot deltaRobot(effectorSideLength);

float currentTouchScale = 0;
float current3TouchHeight, current2TouchHeight = 0;
float zoom = .5;

bool tumble = false;

bool kinectCursor = false;
int kinectScale = 500;


bool cursorPositionPossible = false;
bool runSteppersWithCursor = false;

int activeConsole = 0;




//--------------------------------------------------------------
void testApp::setup(){
    
//    ofSetLogLevel(OF_LOG_VERBOSE);
    
//    ofSetDataPathRoot("./");
    
    textEntryMode = false;
    serialHUDvisible = false;
    
    ofHideCursor();
    // --- add the listeners
    ofAddListener(pad.update, this, &testApp::padUpdates);
    ofAddListener(pad.touchAdded, this, &testApp::newTouch);
    ofAddListener(pad.touchRemoved, this, &testApp::removedTouch);
    
//	ofBackground(250, 250, 250);
	ofBackground(255, 255, 255);
    ofSetCircleResolution(50);

//	ofSetVerticalSync(true);

    
    //some model / light stuff
    glEnable (GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);
    
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
    
    effectorCursorZ = -1000;

}

//--------------------------------------------------------------
void testApp::update(){
    
//    cout << "Serial output: "<<serial.readByte()<< "\n";
    
    deltaRobot.update();

    if (kinectCursor){
        oscListen.update();
            ofPoint newPosition = oscListen.getHighpoint();
            effectorCursorX = (newPosition.x-0.5)*kinectScale;
            effectorCursorY = (newPosition.y-0.5)*kinectScale;
            effectorCursorZ = -(300-newPosition.z*kinectScale);
            
            
            cursorPositionPossible = (deltaRobot.setCartesianPosition(effectorCursorX, effectorCursorY, effectorCursorZ, runSteppersWithCursor)==0);
            cout << "x:"<<effectorCursorX<<", y:"<<effectorCursorX<<", z: "<<effectorCursorZ<<"\n";
    }
    
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
        drawGrid(100, 150, -1000);
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
            
            if (deltaRobot.workingPointsInCubicSpace.size()>0&&showWorkingPointsInCubicSpace){
                deltaRobot.drawWorkingPointsInCubicSpace();
            }
        
            if(showWorkingCube){
                deltaRobot.drawWorkingCubicSpace();
            }
        
                
            
        /*Effector cursor*/
        glPointSize(5.0);
        glBegin(GL_POINTS);
//        if (cursorPositionPossible){
//            ofSetColor(0x000000);  
//        }else{
            ofSetColor(0xff0000);
//        }
        
        glVertex3f(effectorCursorX, effectorCursorZ, effectorCursorY);
        glEnd();
            
            deltaRobot.releaseCoordinatesFromRobot();
        
    glPopMatrix();
    
    glPushMatrix();
    
    glTranslatef(100,100,0);
    
    ofSetColor(0xcccccc);
    
    ofCircle(0, 0, 50);
    
    glTranslatef(80, 50, 100);
    
    //tumble according to mouse
    if (!tumble){
        glRotatef(-mouseY,1,0,0);
        glRotatef(-mouseX,0,1,0);
    }else{
        glRotatef(-220+sin(ofGetElapsedTimef()/3)*20,1,0,0);
        glRotatef(ofGetElapsedTimef()*15,0,1,0);
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

    
    if (serialHUDvisible){
    
        ofPushMatrix();
        
//        ofScale(.3, .3);
        glTranslatef(ofGetWidth()/2, ofGetHeight()/2, 800);
        ofScale(.23, .23);
        
            ofSetColor(0, 220);
            ofRect(0, 0, ofGetWidth(), ofGetHeight());

            glTranslatef(-ofGetWidth()/2, -ofGetHeight()/2, 10);
                    
            addDial(150, ofGetHeight()-120, deltaRobot.theta0, "1");
            addDial(300, ofGetHeight()-120, deltaRobot.theta1, "2");
            addDial(450, ofGetHeight()-120, deltaRobot.theta2, "3");

        
            for (int i=0; i<4; i++){
                
                int upperWidth = 550;
                int upperHeight = 500;
                int spacing = 30;
                
                if (i==0){
                    drawConsole(i, activeConsole==i, 100, 100+upperHeight+spacing, upperWidth*3+spacing*2, 50);
                }else{
                    drawConsole(i, activeConsole==i, 100+(i-1)*(upperWidth+spacing), 100, upperWidth, upperHeight);
                }
                
                
                
            }
            
            ofSetColor(0x353535, 100);
            ofPushMatrix();
                ofTranslate(ofGetWidth()-150, ofGetHeight()-120);
                ofCircle(0, 0, 60);
                ofSetColor(0x888888);
                ofScale(1,1,1);
                glTranslatef(-35, 10, 0.1);
                franklinBook.drawString(ofToString((int)ofGetFrameRate()), 0,0);
            ofPopMatrix();
        
//            glTranslatef(0, 0, 2);
//            ofSetColor(0xffffff);
//            ofRect(mouseX, mouseY, 2, 20); //cursor
        
        ofPopMatrix();
        
    }//if hud visible

}

//--------------------------------------------------------------
void testApp::keyPressed  (int key){ 
    
    if (!textEntryMode){ //if not in text entry mode 
    
    cout << "about to switch "<<key<<endl;
	
	switch (key) {
            
   
            if (deltaRobot.serialConnection.robotReadyForData()){
                
            
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
            showWorkingPointsInCubicSpace = false;
			break;
            
        case '[':
            if (!(deltaRobot.cartesianPointCloud.size()>0)){
                deltaRobot.calculateCartesianPointCloud();
            }
            showCartesianPointCloud = !showCartesianPointCloud;
            showWorkPointCloud = false;
            showWorkingPointsInCubicSpace = false;
			break;
            
        case ']':
        {
            if (!(deltaRobot.workingPointsInCubicSpace.size()>0)){
                deltaRobot.calculateWorkingPointsInCubicSpace();
            }
            showWorkingPointsInCubicSpace = !showWorkingPointsInCubicSpace;
            showWorkPointCloud = false;
            showCartesianPointCloud = false;
        }
			break;
            
        case 'o':
            if (!(deltaRobot.workingCubicSpaceLimits.size()>0)){
                deltaRobot.calculateWorkingCubicSpaceLimits();
            }
            showWorkingCube = !showWorkingCube;
			break;

            
        case ' ':
            tumble = !tumble;
            break;
            
        case 'k':
            kinectCursor = !kinectCursor;
            break;
            
        case 'r':
            deltaRobot.runPath(pathLoader.currentPathFile);
            break;
            
        case 13:
            runSteppersWithCursor = !runSteppersWithCursor;
            break;
            
        case 96:
            serialHUDvisible = true;
            textEntryMode = true;
            break;
            
			
		default:
            cout <<"Key ("<<key<<") pressed\n";
			break;
	}
	
	cout << "Theta 0: " << deltaRobot.theta0 << " Theta 1: " << deltaRobot.theta1 <<" Theta 2: " << deltaRobot.theta2 << "\n";
//	cout << "EffectorX: " << deltaRobot.effectorX << " EffectorY: " << deltaRobot.effectorY << " EffectorZ: " << deltaRobot.effectorZ << "\n\n\n";
            
    }else{ //text entry mode
        
        switch (key) {
            
                
            case 96: //tilde
                serialHUDvisible = false;
                textEntryMode = false;
            break;
            
            case 9: //tab
                activeConsole = (activeConsole>2)?0:activeConsole+1; //cycle 4 numbers
                cout << "activeConsole is now "<<activeConsole<<endl;
            break;
                
            case 13: //enter
                //trigger new text entry event
                if (activeConsole==0){
                    deltaRobot.serialConnection.println(0, textEntryBuffer);
                    deltaRobot.serialConnection.println(1, textEntryBuffer);
                    deltaRobot.serialConnection.println(2, textEntryBuffer);
                }else{
                    deltaRobot.serialConnection.println(activeConsole-1, textEntryBuffer);
                }
                
                
                textEntryBuffer.clear();
            break;
                
            case 127: //backspace
            {
                if (textEntryBuffer.size()>0){
                    textEntryBuffer.erase(textEntryBuffer.end()-1);
                }
                
            }
            break;
                
            
                
            default:
                textEntryBuffer.append(1, key);
                break;
        }
        
        
        
    }
    
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
    
    ofSetColor(0x353535, 100);
    ofPushMatrix();
    glTranslatef(x, y, .1);
    ofCircle(0, 0, 60);
    glTranslatef(0, 0, .1);
    ofSetColor(0x888888);
    ofCircle(0, 0, 10);
    ofSetRectMode(OF_RECTMODE_CENTER);
    
    ofPushMatrix();
    ofTranslate(-50, 13);
    franklinBook.drawString(label, 0,0);
    ofPopMatrix();
    
    ofRotateZ(rotation);
    ofPushMatrix();
    glTranslatef(25, 0, .1);
    ofRect(0, 0, 50, 5);
    ofPopMatrix();
    
    ofSetColor(0x888888);
    ofScale(0.6,0.6,1);
    glTranslatef(20, -10, .1);
    franklinBook.drawString(ofToString(rotation), 0,0);
    
    ofPopMatrix();
    
}

void testApp::drawConsole(int idNum, bool active, int x, int y, int width, int height){
    
    ofSetRectMode(OF_RECTMODE_CORNER);
    
    ofPushMatrix();
    
        ofSetColor(0x353535, 100);
        ofRect(x, y, width, height);
        glTranslatef(0, 0, .1);
        ofSetColor(0x333333, 200);
        ofRect(x, y+height-50, width, 50);
        if (active){
            glTranslatef(0, 0, .1);
            ofSetColor(0x222222);
            ofRect(x+5, y+height-45, width-10, 40);
            ofSetColor(0x888888);
            glTranslatef(0, 0, .1);
            franklinBook.drawString(textEntryBuffer, x+15,y+height-10);
        }
    
    
    
    vector<serialMessage> messageList;
    
    switch (idNum) {
        case 1:
            messageList = deltaRobot.serialConnection.stepper0io;
        break;
            
        case 2:
            messageList = deltaRobot.serialConnection.stepper1io;
        break;
            
        case 3:
            messageList = deltaRobot.serialConnection.stepper2io;
        break;
            
        default:
            break;
    }
    
    int messageListSize = messageList.size();
    
    int lines = (messageListSize>22)? 22 : messageListSize;
    
    if (messageListSize>0){
        
        ofSetColor(0x888888);
        
//        cout << "there is "<<messageListSize<<" messages"<<endl;
        
        glTranslatef(0, 0, .1);
        if (idNum!=0){
            for (int i=0; i<lines; i++){
                
                int elementId = 0;
                
                if (messageListSize>lines){
                    elementId = messageListSize-(lines-i);
                }else{
                    elementId = i;
                }
                
//                cout << "attempting to access id #"<<elementId<<endl;
                
                serialMessage currentMessage = messageList[elementId];
                
                ofPushMatrix();
                ofTranslate(x+20, y+20+i*20);
                ofScale(.6, .6);
                franklinBook.drawString(currentMessage.message, 0, 0);
                ofPopMatrix();
                
            }
        }
    }
    

    
    
    ofPopMatrix();
    
    ofSetRectMode(OF_RECTMODE_CENTER);
    
}

