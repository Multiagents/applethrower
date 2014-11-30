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
    
    std::vector<int> getIdleBins(std::vector<AppleBin> bins, std::vector<Agent> agents);
    
    void takeAction(int *binCounter, std::vector<AppleBin> &bins, std::vector<AppleBin> &repo, 
        std::vector<Agent> &agents, Orchard env, std::vector<LocationRequest> &requests);
    
    void move(AppleBin curBin);
    
private:
    int id;
    Coordinate curLoc;
    Coordinate targetLoc;
    int curBinId;
    int targetBinId;
    
    int getClosestFullBin(std::vector<int> indexes, std::vector<AppleBin> bins);
    
    int getFirstEstFullBin(std::vector<int> indexes, std::vector<AppleBin> bins);
    
    void filterRegisteredLocations(std::vector<LocationRequest> &requests);
    
    int getBinIndexByLocation(std::vector<Agent> &agents, std::vector<AppleBin> &bins, Coordinate loc);

    int checkIfCarryingBin(std::vector<Agent> &agents, std::vector<AppleBin> &bins, Coordinate loc);
    
    Coordinate selectNewLocation(std::vector<Agent> &agents, std::vector<AppleBin> &bins, 
        std::vector<LocationRequest> &requests);
    
    Coordinate getRepoLocation() { return Coordinate(0, curLoc.y); /* Repo at column 0 at every row */ }
    
    bool isLocationValid(Coordinate loc);
    
    AppleBin copyBin(AppleBin ab);
};

#endif // AGENT_HPP_
