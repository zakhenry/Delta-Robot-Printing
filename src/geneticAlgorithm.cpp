//
//  geneticAlgorithm.cpp
//  Delta Sim
//
//  Created by Zak Henry on 19/05/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "geneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm() : deltaRobot(1){ //constructor ( after the : it constructs the deltaRobot)
    
    parms.maxX = 2;
    parms.minX = -2;
    parms.maxY = 3;
    parms.minY = -3;
//    parms.increment = 0.1; //don't think this is needed for a ga (limits accuracy)
    
	cout << "Genetic Algorithmic search class initiated \n";
}

void GeneticAlgorithm::run(){
    
    cout << "Genetic algorithm running...\n";
    
    for (int i=0; i<5; i++){
        population.push_back(generateRandomSpecimen(parms));
    }
    
    sortPopulationByFitness();
    
    cout << "Finished\n";
}

GeneticAlgorithm::specimen GeneticAlgorithm::generateRandomSpecimen(parameters parms){
    specimen newSpecimen;
    newSpecimen.x = ofRandom(parms.minX, parms.maxX);
    newSpecimen.y = ofRandom(parms.minY, parms.maxY);
    
    newSpecimen.fitness = deltaRobot.calculatePointCloudSize(newSpecimen.x, newSpecimen.y); //this is likely going to be the choke point
    
    cout << "new specimen fitness is: " <<newSpecimen.fitness<<"\n";
    
    return newSpecimen;
}

/*bool GeneticAlgorithm::compareSpecimenFitness(specimen a, specimen b){
     cout << "a fitness is "<<a.fitness<<", b fitness is "<<b.fitness<<"\n";
    return (a.fitness < b.fitness);
}*/


void GeneticAlgorithm::sortPopulationByFitness(){
    
    struct sort_by_one
    {
        bool operator () (const specimen& lhs , const specimen& rhs) // replace YourStruct
        {
            return lhs.fitness < rhs.fitness;
        }
    };
    
//    sort(population.begin(), population.end());
//    sort(population.begin(), population.end(), sort_by_one()); //I AM WORKING HERE
    
//    cout << "Specimen comparison is 0 is smaller than 1: "<<compareSpecimenFitness(population[0], population[1])<<"\n";
    
//    vector<specimen> fittestSpecimens;
    
    for (int i=0; i<population.size(); i++){
        cout << population[i].fitness << "\n";
    }

}


