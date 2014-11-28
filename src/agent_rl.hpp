#ifndef AGENT_RL_HPP_
#define AGENT_RL_HPP_

#include <vector>
#include "params.hpp"
#include "data_structs.hpp"
#include "orchard.hpp"

struct Plan {
    int binId;
    float binValue;
    Plan(int b = -1, float v = 0) : binId(b), binValue(v) {}
};

class AgentRL
{
public:
    AgentRL(int i, Coordinate c, int n);
    
    ~AgentRL();
    
    Coordinate getCurLoc() { return curLoc; }
    
    Coordinate getTargetLoc() { return targetLoc; }
    
    int getCurBinId() { return curBinId; }
    
    int getTargetBinId() { return targetBinId; }
    
    int getBinIndexById(std::vector<AppleBin> bins, int id);
    
    std::vector<int> getIdleBins(std::vector<AgentRL> agents, std::vector<AppleBin> bins);
    
    int getStepCount(Coordinate src, Coordinate dst);
    
    float calcWaitTime(AppleBin ab, Orchard env, float reachTime);
    
    float calcPathValues(int binPath[], std::vector<AppleBin> bins, Orchard env);
    
    void makePlans(std::vector<AgentRL> agents, std::vector<AppleBin> bins, std::vector<Coordinate> newLocs, Orchard env);
    
private:
    int id;
    int numLayers;
    Coordinate curLoc;
    Coordinate targetLoc;
    int curBinId;
    int targetBinId;
    std::vector<Plan> plans;
    Plan activePlan;
    
    int getNextBinIndex(std::vector<int> idleBins, int binPath[], int layer);
};

#endif // AGENT_RL_HPP_
