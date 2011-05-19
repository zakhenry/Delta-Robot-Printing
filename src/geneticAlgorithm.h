//
//  geneticAlgorithm.h
//  Delta Sim
//
//  Created by Zak Henry on 19/05/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _GENETIC_ALGORITHM // if this class hasn't been defined, the program can define it
#define _GENETIC_ALGORITHM // by using this if statement you prevent the class to be called more 
// than once which would confuse the compiler

#include "ofMain.h"
#include "deltaKinematics.h"

class DeltaKinematics;

class GeneticAlgorithm {
    
    
    DeltaKinematics deltaRobot;
    
    
public:
    
    struct parameters {
        float minX, maxX, minY, maxY, increment;
    } parms;
    
    int (*fitnessFunction)(int, int); //pointer to function
    
    GeneticAlgorithm(); //constructor
    
    void run();
    
    
};





#endif 