//
//  geneticAlgorithm.cpp
//  Delta Sim
//
//  Created by Zak Henry on 19/05/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "geneticAlgorithm.h"


GeneticAlgorithm::GeneticAlgorithm() : deltaRobot(1){ //constructor ( after the : it constructs the deltaRobot)
    
    parms.maxX = 4;
    parms.minX = -4;
    parms.maxY = 4;
    parms.minY = -4;
    
    
    populationSize = 10000;
    breedingPopulationSize = 0.1;
    generations = 0;
    currentIdNumber = 0;
//    parms.increment = 0.1; //don't think this is needed for a ga (limits accuracy)
    
	cout << "Genetic Algorithmic search class initiated \n";
}

void GeneticAlgorithm::run(){
    
    cout << "Genetic algorithm running...\n";
    
    int initialPopulation = populationSize;
    
    for (int i=0; i<initialPopulation; i++){
        population.push_back(generateRandomSpecimen(parms));
    }
    
    for (int i=0; i<population.size(); i++){
        cout << "First Generation Specimen ["<<population[i].idNum<<"]("<<population[i].x<<", "<<population[i].y<<" aged "<<population[i].age<<" with "<<population[i].children<<" children) has fitness "<<population[i].fitness << "\n";
    }
    
    sortPopulationByFitness();
    cullPopulation();
    
    while (generations<100) {
        
        //"popularity function" possibly could have a period of 'socializing' for the population where each specimen is allowed to 'walk' toward the nearest fittest (could be more than one to avoid local optimum finding) individual 
        breedPopulation();
        padPopulationWithRandomSpecimens();
        sortPopulationByFitness();
        cullPopulation();
        removeDuplicatesFromPopulation();
        
        
        cout << "Generation:"<<generations<<" Best Specimen ["<<population[0].idNum<<"]("<<population[0].x<<", "<<population[0].y<<" aged "<<population[0].age<<" with "<<population[0].children<<" children) has fitness "<<population[0].fitness << "\n";
    }
    
    cout << "Best specimen found in generation "<<population[0].generation<<"\n";
    
    
    for (int i=0; i<population.size(); i++){
        cout << "Last Generation Specimen ["<<population[i].idNum<<"]("<<population[i].x<<", "<<population[i].y<<" aged "<<population[i].age<<" with "<<population[i].children<<" children) has fitness "<<population[i].fitness << "\n";
    }
    
    cout << "GA Finished\n";
}

void GeneticAlgorithm::reset(){
    generations = 0;
    population.clear(); //removes all specimens
    currentIdNumber = 0;
    cout << "GA Reset\n";
}

GeneticAlgorithm::specimen GeneticAlgorithm::generateRandomSpecimen(parameters parms){
    specimen newSpecimen;
    newSpecimen.x = ofRandom(parms.minX, parms.maxX);
    newSpecimen.y = ofRandom(parms.minY, parms.maxY);
    
    newSpecimen.fitness = deltaRobot.calculatePointCloudSize(newSpecimen.x, newSpecimen.y); //this is likely going to be the choke point (not actually running point cloud algo yet)
    newSpecimen.age = 0; //any member of population created randomly will have an age of 0
    newSpecimen.generation = generations;
    newSpecimen.children = 0;
    newSpecimen.idNum = nextIdNumber();
//    cout << "new specimen fitness is: ()" <<newSpecimen.fitness<<"\n";
    
    return newSpecimen;
}

void GeneticAlgorithm::sortPopulationByFitness(){
    
    for (int i=0; i<population.size(); i++){
//        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<")has fitness "<<population[i].fitness << "\n";
    }
    
    sort(population.begin(), population.end(), compareFitness()); 

}

void GeneticAlgorithm::cullPopulation(){
    
    int populationToSurvive = breedingPopulationSize*population.size();
    
    population.erase(population.begin()+populationToSurvive, population.end()); //remove unfit specimens
    

    for (int i=0; i<population.size(); i++){
        population[i].age++; //increment age
//        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<") has fitness "<<population[i].fitness << "\n";
    }
    
}

void GeneticAlgorithm::removeDuplicatesFromPopulation(){
    
    vector<specimen>::iterator it = unique(population.begin(), population.end(), compareSpecimenEquality());
    population.resize(it - population.begin());
    
//    cout << "pop size after all culling: "<<population.size()<<"\n";
    
}

//"popularity function" possibly could have a period of 'socializing' for the population where each specimen is allowed to 'walk' toward the nearest fittest (could be more than one to avoid local optimum finding) individual 

void GeneticAlgorithm::breedPopulation(){
    
    vector<specimen> breedingPopulation;
    breedingPopulation = population;
    
    population.clear(); //this is only so the parents can have children counters incremented. delete if too costly
    
    while (breedingPopulation.size()>=2) { //while children can still be made
        
        //im sure the following could be handled much better? surely??
        
        int firstParentId = ofRandom(0, breedingPopulation.size()-1);
        specimen firstParent = breedingPopulation[firstParentId];
        breedingPopulation.erase(breedingPopulation.begin()+firstParentId);
        firstParent.children ++;
        
        int secondParentId = ofRandom(0, breedingPopulation.size()-1);
        specimen secondParent = breedingPopulation[secondParentId];
        breedingPopulation.erase(breedingPopulation.begin()+secondParentId);
        secondParent.children ++;
        
        
        specimen child = createChild(firstParent, secondParent);
        
        population.push_back(firstParent);
        population.push_back(secondParent);
        population.push_back(child);
        
    }
    
    for (int i=0; i<population.size(); i++){
//        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<") has fitness "<<population[i].fitness << "\n";
    }
    
    generations ++; //a new generation has been added
    
    
}

GeneticAlgorithm::specimen GeneticAlgorithm::createChild(specimen parentA, specimen parentB){ //currently using two parents though three is apparently better in some cases if a little weird
    //defines crossover algorithm (currently using simple averaging of vars)
    
    specimen child;
    child.x = (parentA.x+parentB.x)*0.5; //multiplication is quicker
    child.y = (parentA.y+parentB.y)*0.5;
    
    child.fitness = deltaRobot.calculatePointCloudSize(child.x, child.y);
    child.age = 0;
    child.generation = generations;
    child.children = 0;
    child.idNum = nextIdNumber();
    
    return child;
}

void GeneticAlgorithm::padPopulationWithRandomSpecimens(){
    
    //fills population's remaining space with complete random specimens to keep the current colony on its toes
    
    while (population.size()<populationSize) {
        population.push_back(generateRandomSpecimen(parms));
    }
    
    /*for (int i=0; i<population.size(); i++){
        cout << "Generation:"<<generations<<" Specimen ["<<population[i].idNum<<"]("<<population[i].x<<", "<<population[i].y<<" aged "<<population[i].age<<" with "<<population[i].children<<" children) has fitness "<<population[i].fitness << "\n";
    }*/
        
}

int GeneticAlgorithm::nextIdNumber(){
    currentIdNumber++;
    return currentIdNumber;
}

bool GeneticAlgorithm::specimensAreEqual(specimen a, specimen b){
    if (a.fitness==b.fitness){
        if (a.x == b.x && a.y == b.y){
            return true;
        }
    }
    return false;
}

