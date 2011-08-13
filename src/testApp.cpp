#include "testApp.h"

GeneticAlgorithm ga; //comment out if using serial devices

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

    //some model / light stuff
    glEnable (GL_DEPTH_TEST);
    glShadeModel (GL_SMOOTH);

}

//--------------------------------------------------------------
void testApp::update(){

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
    }
	
    
    
	glTranslatef(-ofGetWidth()/2,0,0);

    ofSetColor(100, 100, 100);
    ofDrawBitmapString("fps: "+ofToString(ofGetFrameRate(), 2), 200, -200);

    ga.drawSearchSpace(fitnessThreshold, fitnessColorScale);
    ga.drawCurrentPopulation(fitnessThreshold, fitnessColorScale);
 
    
    

}

//--------------------------------------------------------------
void testApp::keyPressed  (int key){ 
	
	switch (key) {
		            
        case 'o':
            ga.run();
            break;
            
        case 13:
            ga.calculateSearchSpace();
            break;
            
        case ' ':
            tumble = !tumble;
            break;
            
        case 'w':
            ga.showWalls = !ga.showWalls;
            break;
            
			
		default:
            cout <<"Key ("<<key<<") pressed\n";
			break;
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

