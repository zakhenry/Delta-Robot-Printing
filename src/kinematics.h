//
//  kinematics.h
//  Delta Control
//
//  Created by Zak Henry on 10/12/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _KINEMATICS // if this class hasn't been defined, the program can define it
#define _KINEMATICS // by using this if statement you prevent the class to be called more 


class Kinematics {
    
    public:
    
    Kinematics(float e, float f, float re, float rf);
    
    int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
    int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);

};

#endif 