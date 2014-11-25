#ifndef AGENT_HPP_
#define AGENT_HPP_

#include <vector>
#include "data_structs.hpp"
#include "orchard.hpp"

class Agent
{
public:
    Agent(int i, Coordinate c);
    
    ~Agent();
    
    Coordinate getCurLoc() { return curLoc; }
    
    Coordinate getTargetLoc() { return targetLoc; }
    
    int getCurBinId() { return curBinId; }
    
    int getTargetBinId() { return targetBinId; }
    
    float getDistance(Coordinate loc1, Coordinate loc2);
    
    int getBinIndexById(std::vector<AppleBin> bins, int id);
    
    void takeAction(int *binCounter, std::vector<AppleBin> &bins, std::vector<AppleBin> &repo, 
        std::vector<Agent> &agents, Orchard env, std::vector<Coordinate> &newLocs);
    
    void move(AppleBin curBin);
    
private:
    int id;
    Coordinate curLoc;
    Coordinate targetLoc;
    int curBinId;
    int targetBinId;
    
    std::vector<int> getIdleBins(std::vector<AppleBin> bins, std::vector<Agent> agents);
    
    int getClosestFullBin(std::vector<int> indexes, std::vector<AppleBin> bins);
    
    int getFirstEstFullBin(std::vector<int> indexes, std::vector<AppleBin> bins);
    
    void filterRegisteredLocations(std::vector<Coordinate> &newLocs);
    
    int getBinIndexByLocation(std::vector<AppleBin> &bins, Coordinate loc);
    
    Coordinate selectNewLocation(std::vector<Agent> &agents, std::vector<AppleBin> &bins, 
        std::vector<Coordinate> &newLocs);
    
    Coordinate getRepoLocation() { return Coordinate(0, curLoc.y); /* Repo at column 0 at every row */ }
    
    bool isLocationValid(Coordinate loc);
    
    AppleBin copyBin(AppleBin ab);
};

#endif // AGENT_HPP_
