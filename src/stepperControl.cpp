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
    
    serial0.enumerateDevices();
    
    if(!serial0.setup(/*"tty.usbserial-A9007Mbm"*/5, 9600)){
        printf("Serial setup failed!\n");
        
    }else{
        println("d180-s100e", 0);
    }
    
    if(!serial1.setup(/*"tty.usbserial-A9007Mbm"*/6, 9600)){
        printf("Serial setup failed!\n");
    }
    
    if(!serial2.setup(/*"tty.usbserial-A9007Mbm"*/7, 9600)){
        printf("Serial setup failed!\n");
    }
}

bool StepperControl::println(string line, int stepper){
    
    unsigned char * a = new unsigned char[line.size()+1];
    a[line.size()]=0;
    memcpy(a,line.c_str(),line.size());
    
    switch (stepper) {
        case 0:
            serial0.writeBytes(a,line.size());
            cout <<"output to serial0: "<<a<<"\n";
        break;
            
        case 1:
            serial1.writeBytes(a,1);
        break;
            
        case 2:
            serial2.writeBytes(a,1);
        break;
            
        default:
        break;
    }
}

//void StepperControl::message
