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
    stepper0Connected = stepper1Connected = stepper2Connected = false;
    
    serial0.enumerateDevices();
    
    if(!serial0.setup(/*"tty.usbserial-A9007Mbm"*/5, 9600)){
        printf("Serial setup failed!\n");
    }else{
        stepper0Connected = true;
    }
    
    if(!serial1.setup(/*"tty.usbserial-A9007Mbm"*/7, 9600)){
        printf("Serial setup failed!\n");
    }else{
        stepper1Connected = true;
    }
    
    if(!serial2.setup(/*"tty.usbserial-A9007Mbm"*/9, 9600)){
        printf("Serial setup failed!\n");
    }else{
        stepper2Connected = true;
    }

}

bool StepperControl::println(int stepper, string line){
    
    unsigned char * charLine = new unsigned char[line.size()+1];
    charLine[line.size()]=0;
    memcpy(charLine,line.c_str(),line.size());
    
    switch (stepper) {
        case 0:
        {
            if (stepper0Connected){
                stepper0Ready = false;
                serial0.writeBytes(charLine,line.size());
                cout <<"output to serial0: "<<charLine<<"\n";
            }
            
        }
        break;
            
        case 1:
        {
            if (stepper1Connected){
                stepper1Ready = false;
                serial1.writeBytes(charLine,line.size());
            }
        }
        break;
            
        case 2:
        {
            if (stepper2Connected){
                stepper2Ready = false;
                serial2.writeBytes(charLine,line.size());
            }
        }
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
    if (!stepper0Ready&&stepper0Connected){
        string message;
        readUntil(0, message, '\n');
        cout <<"message recieved was\n";
        if (message == "complete"){
            stepper0Ready = true;
        }
    }
    
    if (!stepper1Ready&&stepper1Connected){
        string message;
        readUntil(1, message, '\n');
        cout <<"message recieved was\n";
        if (message == "complete"){
            stepper1Ready = true;
        }
    }
    
    if (!stepper2Ready&&stepper2Connected){
        string message;
        readUntil(2, message, '\n');
        cout <<"message recieved was\n";
        if (message == "complete"){
            stepper2Ready = true;
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

bool StepperControl::robotReadyForData(){
    
    
    if (stepper0Connected && stepper1Connected && stepper2Connected && stepper0Ready && stepper1Ready && stepper2Ready){
        return true;
    }
    
    return false;
    
}
