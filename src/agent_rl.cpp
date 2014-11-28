#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "agent_rl.hpp"

AgentRL::AgentRL(int i, Coordinate c, int n)
{
    id = i;
    curLoc = c;
    numLayers = n;
    
    curBinId = -1;
    targetBinId = -1;
    targetLoc = Coordinate(-1, -1);
}

AgentRL::~AgentRL()
{
    curLoc = Coordinate(-1, -1);
    curBinId = -1;
    targetBinId = -1;
    targetLoc = Coordinate(-1, -1);
    activePlan.binId = -1;
    activePlan.binValue = 0;
}

int AgentRL::getBinIndexById(std::vector<AppleBin> bins, int id)
{
    for (int b = 0; b < (int) bins.size(); ++b) {
        if (bins[b].id == id)
            return b;
    }
    return -1;
}

std::vector<int> AgentRL::getIdleBins(std::vector<AgentRL> agents, std::vector<AppleBin> bins)
{
    std::vector<int> idleBins;
    
    if (bins.size() == 0)
        return idleBins;
    
    for (int b = 0; b < (int) bins.size(); ++b) {
        if (bins[b].onGround)
            idleBins.push_back(b);
        else
            idleBins.push_back(-1);
    }
    
    for (int a = 0; a < (int) agents.size(); ++a) {
        if (agents[a].id == id)
            continue;
        int bIdx = getBinIndexById(bins, agents[a].getCurBinId());
        if (bIdx != -1)
            idleBins[bIdx] = -1;
        int tIdx = getBinIndexById(bins, agents[a].getTargetBinId());
        if (tIdx != -1)
            idleBins[tIdx] = -1;
    }
    
    for (int i = idleBins.size() - 1; i >= 0; --i) {
        if (idleBins[i] == -1)
            idleBins.erase(idleBins.begin() + i);
    }
    
    return idleBins;
}

int AgentRL::getNextBinIndex(std::vector<int> idleBins, int binPath[], int layer)
{
    for (int i = 0; i < (int) idleBins.size(); ++i) {
        bool used = false;
        for (int j = 0; j < layer && used == false; ++j) {
            if (idleBins[i] == binPath[j])
                used = true;
        }
        if (!used)
            return idleBins[i];
    }
    return -1;
}

int AgentRL::getStepCount(Coordinate src, Coordinate dst)
{
    int initStep = 0;
    if (src.y != dst.y) {
        int leftStep = src.x - 0;
        int rightStep = ORCH_COLS - 1 - src.x;
        initStep = (leftStep <= rightStep) ? leftStep : rightStep;
    }
    
    return initStep + abs(src.x - dst.x) + abs(src.y - dst.y);
}

float AgentRL::calcWaitTime(AppleBin ab, Orchard env, float reachTime)
{
    float harvestedApples = ab.fillRate * reachTime;
    if (env.getApplesAt(ab.loc) - harvestedApples <= 0)
        return 0.0f;
    
    float remCapacity = BIN_CAPACITY - round(ab.capacity + harvestedApples);
    return remCapacity / ab.fillRate;
}

float AgentRL::calcPathValues(int binPath[], std::vector<AppleBin> bins, Orchard env)
{
    float sum = 0;
    float times[numLayers];
    
    for (int j = 0; j < numLayers; ++j) {
        if (binPath[j] == -1)
            break;
        float prevTime = (j > 0) ? times[j - 1] : 0;
        float reachTime = ((float) getStepCount(curLoc, bins[binPath[j]].loc)) / AGENT_SPEED_H;
        float waitTime = calcWaitTime(bins[binPath[j]], env, reachTime);
        float returnTime = (bins[binPath[j]].loc.x - 0) / AGENT_SPEED_L; // bin.loc.x - 0 (repo at column 0)
        times[j] = prevTime + reachTime + waitTime + returnTime;
    }
    
    for (int j = 0; j < numLayers; ++j)
        sum += times[j];
    
    return sum;
}

bool planComparator(Plan p1, Plan p2)
{
    return p1.binValue < p2.binValue;
}

void AgentRL::makePlans(std::vector<AgentRL> agents, std::vector<AppleBin> bins, std::vector<Coordinate> newLocs, 
    Orchard env)
{
    plans.clear();
    if (curBinId != -1 || targetBinId != -1) // agent is not idle; don't make a new plan
        return;
    
    std::vector<int> idleBins = getIdleBins(agents, bins);
    if (idleBins.size() == 0)
        return;
    
    int numIdleBins = idleBins.size();
    int binPaths[numIdleBins][numLayers];
    
    for (int i = 0; i < numIdleBins; ++i)
        for (int j = 0; j < numLayers; ++j)
            binPaths[i][j] = -1; // Initialization
    
    for (int i = 0; i < numIdleBins; ++i)
        for (int j = 0; j < numLayers && j < numIdleBins; ++j)
            binPaths[i][j] = getNextBinIndex(idleBins, binPaths[i], j);
    
    for (int i = 0; i < numIdleBins; ++i)
        plans.push_back(Plan(binPaths[i][0], calcPathValues(binPaths[i], bins, env)));
    std::sort(plans.begin(), plans.end(), planComparator);
}



