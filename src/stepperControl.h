//
//  stepperControl.h
//  Delta Sim
//
//  Created by Zak Henry on 9/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//
#ifndef _STEPPER_CONTROL // if this class hasn't been defined, the program can define it
#define _STEPPER_CONTROL // by using this if statement you prevent the class to be called more 
// than once which would confuse the compiler

#include "ofMain.h"


class StepperControl {
    
public:
    
    StepperControl(); //constructor
    
    ofSerial serial0, serial1, serial2;
    
    bool println(string line, int stepper);
    
};





#endif 