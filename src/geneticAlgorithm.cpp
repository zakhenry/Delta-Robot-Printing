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
    
    populationSize = 100;
    breedingPopulation = 0.6;
//    parms.increment = 0.1; //don't think this is needed for a ga (limits accuracy)
    
	cout << "Genetic Algorithmic search class initiated \n";
}

void GeneticAlgorithm::run(){
    
    cout << "Genetic algorithm running...\n";
    
    int randomSpecimensToAdd = populationSize-population.size();
    
    for (int i=0; i<randomSpecimensToAdd; i++){
        population.push_back(generateRandomSpecimen(parms));
    }
    
    sortPopulationByFitness();
    cullPopulation();
    
    cout << "Finished\n";
}

GeneticAlgorithm::specimen GeneticAlgorithm::generateRandomSpecimen(parameters parms){
    specimen newSpecimen;
    newSpecimen.x = ofRandom(parms.minX, parms.maxX);
    newSpecimen.y = ofRandom(parms.minY, parms.maxY);
    
    newSpecimen.fitness = deltaRobot.calculatePointCloudSize(newSpecimen.x, newSpecimen.y); //this is likely going to be the choke point (not actually running point cloud algo yet)
    
//    cout << "new specimen fitness is: ()" <<newSpecimen.fitness<<"\n";
    
    return newSpecimen;
}

void GeneticAlgorithm::sortPopulationByFitness(){
    
    for (int i=0; i<population.size(); i++){
        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<")has fitness "<<population[i].fitness << "\n";
    }
    
    sort(population.begin(), population.end(), compareFitness()); //I AM WORKING HERE

}

void GeneticAlgorithm::cullPopulation(){
    
    for (int i=0; i<population.size(); i++){
        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<")has fitness "<<population[i].fitness << "\n";
    }
    
    float lastSpecimenToSurvive = 0.6*population.size(); //percent
    
    cout << "last specimen to survive: "<<lastSpecimenToSurvive<<"\n";
    
    int populationToSurvive = breedingPopulation*populationSize;
    
    population.erase(population.begin()+populationToSurvive, population.end());
    
    for (int i=0; i<population.size(); i++){
        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<")has fitness "<<population[i].fitness << "\n";
    }
    
}

//"popularity function" possibly could have a period of 'socializing' for the population where each specimen is allowed to 'walk' toward the nearest fittest (could be more than one to avoid local optimum finding) individual 

void GeneticAlgorithm::breedPopulation(){
    
}


