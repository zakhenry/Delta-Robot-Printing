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
    
    struct parameters {
        float minX, maxX, minY, maxY, increment;
    } parms;
    
    struct specimen {
        float x, y, fitness;
        int age;
    };
    
    
    struct compareFitness{
        bool operator()(const specimen& lhs , const specimen& rhs) const{
            return lhs.fitness < rhs.fitness; //bool should be equivalent to the statement "lhs is fitter than rhs"
        }
    };
    
    specimen generateRandomSpecimen(parameters parms);
    
    bool compareSpecimenFitness(specimen a, specimen b);
    void sortPopulationByFitness();
    void cullPopulation();
    void breedPopulation();
    
    vector<specimen> population;
    
public:
    
    int populationSize;
    float breedingPopulation; //percentage of population size (must be less than 66.66). This size also defines the randoms added each cull
    
    GeneticAlgorithm(); //constructor
    
    void run();
    
    
    
    
};


#endif 