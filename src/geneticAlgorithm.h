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
        float minX, maxX, minY, maxY, mutationFactor;
    } parms;
    
    struct specimen {
        float x, y, fitness, fitnessTimeCalc;
        unsigned int age, generation, children, idNum;
    };
    
    int currentIdNumber;
    int nextIdNumber();
    
    
    struct compareFitness{
        bool operator()(const specimen& lhs , const specimen& rhs) const{
            return lhs.fitness > rhs.fitness; //bool should be equivalent to the statement "lhs is fitter than rhs"
        }
    };
    
    struct compareSpecimenEquality{
        bool operator()(const specimen& a , const specimen& b) const{
            if (a.fitness==b.fitness){
                if (a.x == b.x && a.y == b.y){
                    return true;
                }
            }
            return false;
        }
    };
    
    
    
    //ga fundamentals
    void initiatePopulation();
    void removeDuplicatesFromPopulation();
    
    specimen createChild(specimen a, specimen b);
    specimen generateRandomSpecimen(parameters parms);
    
    vector<specimen> population;
    
    //old ga method:
    void sortPopulationByFitness();
    void cullPopulation();
    void breedPopulation();
    void padPopulationWithRandomSpecimens();
    
    //new ga method:
    void rutAndBreedIndividuals(); //three individuals are chosen from population (at random for now) and weakest is killed then fittest two breed to produce one child to keep the population stable
    
    
    
    
    /*Drawing functions*/
    
    vector<specimen>bruteForceSearchSpace(parameters parms);
    float searchIncrement;
    vector<specimen>allSpecimens;
    
    
    
    ofColor HSVToRGB(float h, float s, float v, ofColor &in);
    
public:
    
    int generations;
    int populationSize;
    float breedingPopulationSize; //percentage of population size (must be less than 66.66). This size also defines the randoms added each cull
    
    GeneticAlgorithm(); //constructor
    
    void run();
    void reset();

    void calculateSearchSpace();
    void drawSearchSpace();
    
    void drawCurrentPopulation();
    
    
};


#endif 