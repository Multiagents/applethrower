#ifndef AUTO_AGENT_HPP_
#define AUTO_AGENT_HPP_

#include <vector>
#include "params.hpp"
#include "data_structs.hpp"
#include "orchard.hpp"

struct Plan {
    int binId;
    float value;
    Plan(int b = -1, float v = 0) : binId(b), value(v) {}
};

struct AutoState {
    int binStepCount;
    int locStepCount;
    int binToLocStepCount;
    int binEstFullTime; // estimated time until the bin is full
    float reward;
    AutoState(int b, int l, int d, int e, float r = 0) 
        : binStepCount(b), locStepCount(l), binToLocStepCount(d), binEstFullTime(e), reward(r) {}
};

class AutoAgent
{
public:
    AutoAgent(int i, Coordinate c, int n);
    
    ~AutoAgent();
    
    Coordinate getCurLoc() { return curLoc; }
    
    Coordinate getTargetLoc() { return targetLoc; }
    
    int getCurBinId() { return curBinId; }
    
    int getTargetBinId() { return targetBinId; }
    
    std::vector<Plan> getPlans() { return plans; }
    
    Plan getActivePlan() { return activePlan; }
    
    int getBinIndexById(std::vector<AppleBin> bins, int id);
    
    std::vector<int> getIdleBins(std::vector<AutoAgent> agents, std::vector<AppleBin> bins);
    
    int getStepCount(Coordinate src, Coordinate dst);
    
    float calcWaitTime(AppleBin ab, Orchard env, float reachTime);
    
    float calcPathValues(int binPath[], std::vector<AppleBin> bins, Orchard env);
    
    void makePlans(std::vector<AutoAgent> agents, std::vector<AppleBin> bins, Orchard env);
    
    void selectPlan(std::vector<AutoAgent> &agents, std::vector<AppleBin> bins);
    
    int getStateIndex(AutoState s);
    
    void removeLocationRequest(Coordinate loc, std::vector<Coordinate> &locRequests);
    
    Coordinate selectClosestLocationRequest(Coordinate loc, std::vector<Coordinate> locRequests, 
        std::vector<AutoAgent> agents);
    
    Coordinate selectLocationRequest(std::vector<Coordinate> locRequests, AppleBin ab, std::vector<AutoAgent> agents, 
        int *stateIndex);
    
    void move(Coordinate loc, std::vector<AppleBin> &bins, int index);
    
    void takeAction(int *binCounter, std::vector<AppleBin> &bins, std::vector<Coordinate> &locRequests, 
        std::vector<AutoAgent> &agents, std::vector<AppleBin> &repo, Orchard env);
    
private:
    int id;
    int numLayers;
    Coordinate curLoc;
    int curBinId;
    int targetBinId;
    Coordinate targetLoc;
    Plan activePlan;
    Coordinate activeLocation;
    int activeStateIndex;
    std::vector<Plan> plans;
    static std::vector<AutoState> states;
    
    void removePlan(int binId);
    
    bool areSameStates(AutoState s1, AutoState s2);
    
    bool isLocationServed(Coordinate loc, std::vector<AutoAgent> agents);
    
    bool isLocationValid(Coordinate l);
    
    AppleBin copyBin(AppleBin ab);
};

#endif // AUTO_AGENT_HPP_



