//
//  stepperControl.cpp
//  Delta Sim
//
//  Created by Zak Henry on 9/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "stepperControl.h"


StepperControl::StepperControl(){ //constructor
    
	cout << "Stepper control class instantiated \n";
    
    stepper0Ready = stepper1Ready = stepper2Ready = true;
    
    serial0.enumerateDevices();
    
    if(!serial0.setup(/*"tty.usbserial-A9007Mbm"*/5, 9600)){
        printf("Serial setup failed!\n");
        
    }else{
        setStepper(0, 1800, 100);
    }
    
    if(!serial1.setup(/*"tty.usbserial-A9007Mbm"*/6, 9600)){
        printf("Serial setup failed!\n");
    }
    
    if(!serial2.setup(/*"tty.usbserial-A9007Mbm"*/7, 9600)){
        printf("Serial setup failed!\n");
    }

}

bool StepperControl::println(int stepper, string line){
    
    unsigned char * charLine = new unsigned char[line.size()+1];
    charLine[line.size()]=0;
    memcpy(charLine,line.c_str(),line.size());
    
    switch (stepper) {
        case 0:
            stepper0Ready = false;
            serial0.writeBytes(charLine,line.size());
            cout <<"output to serial0: "<<charLine<<"\n";
        break;
            
        case 1:
            stepper1Ready = false;
            serial1.writeBytes(charLine,line.size());
        break;
            
        case 2:
            stepper1Ready = false;
            serial2.writeBytes(charLine,line.size());
        break;
            
        default:
        break;
    }
}

void StepperControl::setStepper(int stepper, float angle, float speed){
    
    char buffer [50];
    sprintf (buffer, "d%f-s%de", angle, (int)speed); //note speed is cast to int for now, will be float eventually
    println (stepper, buffer);
    
}

void StepperControl::update(){
//    cout << "stepper control updated\n";
    if (!stepper0Ready){
        string message;
        readUntil(0, message, '\n');
        cout <<"message recieved was\n";ç
        if (message == "complete"){
            stepper0Ready = true;
        }
    }

}

bool StepperControl::readUntil(int stepper, string& rResult, char cUntil) {
    cout <<"waiting for message\n";
	char b[1];
	char buf[1];
	int  i = 0;
    
	do {
        int n = -1;
		
        switch (stepper) {
            case 0:
                n = serial0.readByte();  // read a char at a time
                break;
                
            case 1:
                n = serial1.readByte();  // read a char at a time
                break;
                
            case 2:
                n = serial2.readByte();  // read a char at a time
                break;
                
            default:
                break;
        }
		if( n == -1) return false;    // couldn't read
		if(n == 0) {
			return false;
		}
		buffer.push_back(b[0]);
		i++;
	} while( b[0] != cUntil );
	
	std::vector<char>::iterator it = buffer.begin();
	while(it != buffer.end()) {
		rResult.push_back((*it));
		++it;
	}
	buffer.clear();
	return true;
    
}
