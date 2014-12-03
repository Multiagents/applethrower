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
    static const float C_H;
    static const float C_B;
    
    AutoAgent(int i, Coordinate c, int n, bool learn = false);
    
    ~AutoAgent();
    
    int getId() { return id; }
    
    void setCurLoc(Coordinate loc) { curLoc = loc; }
    
    Coordinate getCurLoc() { return curLoc; }
    
    Coordinate getTargetLoc() { return targetLoc; }
    
    int getCurBinId() { return curBinId; }
    
    int getTargetBinId() { return targetBinId; }
    
    std::vector<Plan> getPlans() { return plans; }
    
    Plan getActivePlan() { return activePlan; }
    
    int getBinIndexById(std::vector<AppleBin> bins, int id);
    
    int getBinIndexByLoc(std::vector<AppleBin> bins, Coordinate loc);
    
    std::vector<int> getIdleBins(std::vector<AutoAgent> agents, std::vector<AppleBin> bins);
    
    int getStepCount(Coordinate src, Coordinate dst);
    
    float calcWaitTime(AppleBin ab, Orchard env, float reachTime, std::vector<AppleBin> bins);
    
    float calcPathValues(int binPath[], std::vector<AppleBin> bins, std::vector<AutoAgent> agents, Orchard env, 
        std::vector<Worker> workers);
    
    void makePlans(std::vector<AutoAgent> agents, std::vector<AppleBin> bins, Orchard env, std::vector<Worker> workers);
    
    void selectPlan(std::vector<AutoAgent> &agents, std::vector<AppleBin> bins);
    
    int getStateIndex(AutoState s);
    
    void removeLocationRequest(Coordinate loc, std::vector<LocationRequest> &requests);
    
    Coordinate selectClosestLocationRequest(Coordinate loc, std::vector<LocationRequest> requests, 
        std::vector<AutoAgent> agents, std::vector<AppleBin> bins);
    
    Coordinate selectLocationRequest(std::vector<LocationRequest> requests, AppleBin ab, std::vector<AutoAgent> agents, 
        int *stateIndex, std::vector<AppleBin> bins);
    
    void move(Coordinate loc, std::vector<AppleBin> &bins, int index);
    
    void takeAction(int *binCounter, std::vector<AppleBin> &bins, std::vector<LocationRequest> &requests, 
        std::vector<AutoAgent> &agents, std::vector<AppleBin> &repo, Orchard env, std::vector<Worker> workers, 
        int curTime);
    
    int getNumOfStates() { return (int) states.size(); }
    
    std::vector<AutoState> getStates() { return states; }
    
private:
    int id;
    int numLayers;
    bool useLearning;
    Coordinate curLoc;
    int curBinId;
    int targetBinId;
    Coordinate targetLoc;
    Plan activePlan;
    Coordinate activeLocation;
    int activeStateIndex;
    static std::vector<AutoState> states;
    std::vector<AppleBin> binsHistory;
    std::vector<Plan> plans;
    int lastDecisionTime;
    Coordinate lastDecisionLoc;
    Coordinate lastActiveLoc;
    float binWaitTime;
    float humanWaitTime;
    
    Coordinate getCarrierDestination(AppleBin ab, std::vector<AutoAgent> agents);
    
    int countWorkersAt(Coordinate loc, std::vector<Worker> workers);
    
    void removePlan(int binId);
    
    bool areSameStates(AutoState s1, AutoState s2);
    
    bool isLocationServed(Coordinate loc, std::vector<AutoAgent> agents, std::vector<AppleBin> bins);
    
    bool hasBin(Coordinate loc, std::vector<AppleBin> bins);
    
    bool isLocationValid(Coordinate l);
    
    int getRequestTime(Coordinate loc, std::vector<LocationRequest> requests);
    
    float getCFRewardAvg(std::vector<LocationRequest> requests, AppleBin ab);
    
    float getCFRewardClosest(std::vector<LocationRequest> requests, AppleBin ab);
    
    AppleBin copyBin(AppleBin ab);
};

#endif // AUTO_AGENT_HPP_



