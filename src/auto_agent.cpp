#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <climits>
#include "auto_agent.hpp"

const float AutoAgent::C_H = 0.8;
const float AutoAgent::C_B = 0.3;
std::vector<AutoState> AutoAgent::states;

AutoAgent::AutoAgent(int i, Coordinate c, int n, bool learn)
{
    id = i;
    curLoc = c;
    numLayers = n;
    useLearning = learn;
    
    curBinId = -1;
    targetBinId = -1;
    targetLoc = Coordinate(-1, -1);
    activePlan = Plan(-1, 0);
    activeLocation = Coordinate(-1, -1);
    activeStateIndex = -1;
    plans.clear();
}

AutoAgent::~AutoAgent()
{
    curLoc = Coordinate(-1, -1);
    curBinId = -1;
    targetBinId = -1;
    targetLoc = Coordinate(-1, -1);
    activePlan = Plan(-1, 0);
    activeLocation = Coordinate(-1, -1);
    plans.clear();
}

int AutoAgent::getBinIndexById(std::vector<AppleBin> bins, int id)
{
    for (int b = 0; b < (int) bins.size(); ++b) {
        if (bins[b].id == id)
            return b;
    }
    return -1;
}

std::vector<int> AutoAgent::getIdleBins(std::vector<AutoAgent> agents, std::vector<AppleBin> bins)
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
        int stepCount = getStepCount(agents[a].curLoc, agents[a].activeLocation);
        if (bIdx != -1 && stepCount > AGENT_SPEED_H)
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

int AutoAgent::getStepCount(Coordinate src, Coordinate dst)
{
    int initStep = 0;
    if (!isLocationValid(src) || !isLocationValid(dst))
        return 0;
    
    if (src.y != dst.y) {
        int leftStep = src.x - 0;
        int rightStep = ORCH_COLS - 1 - src.x;
        initStep = (leftStep <= rightStep) ? leftStep : rightStep;
    }
    
    return initStep + abs(src.x - dst.x) + abs(src.y - dst.y);
}

float AutoAgent::calcWaitTime(AppleBin ab, Orchard env, float reachTime)
{
    float harvestedApples = ab.fillRate * reachTime;
    if (env.getApplesAt(ab.loc) - harvestedApples <= 0)
        return 0.0f;
    
    float remCapacity = BIN_CAPACITY - round(ab.capacity + harvestedApples);
    return remCapacity / ab.fillRate;
}

float AutoAgent::calcPathValues(int binPath[], std::vector<AppleBin> bins, Orchard env)
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
    return p1.value < p2.value;
}

void AutoAgent::makePlans(std::vector<AutoAgent> agents, std::vector<AppleBin> bins, Orchard env)
{
    plans.clear();
    if (curBinId != -1 || targetBinId != -1) // agent is not idle; don't make a new plan
        return;
    
    std::vector<int> idleBins = getIdleBins(agents, bins);
    printf("A%d sees %d idle bins.\n", id, (int) idleBins.size());
    if (idleBins.size() == 0)
        return;
    
    int numIdleBins = idleBins.size();
    int binSeqs[numIdleBins][numLayers];
    
    for (int i = 0; i < numIdleBins; ++i)
        for (int j = 0; j < numLayers; ++j)
            binSeqs[i][j] = -1; // Initialization
    
    // Create possible bin sequences
    for (int i = 0; i < numIdleBins; ++i) {
        for (int j = 0; j < numLayers && j < numIdleBins; ++j)
            binSeqs[i][j] = idleBins[j];
        std::next_permutation(idleBins.begin(), idleBins.end());
    }
    
    for (int i = 0; i < numIdleBins; ++i)
        plans.push_back(Plan(bins[binSeqs[i][0]].id, calcPathValues(binSeqs[i], bins, env)));
    
    std::sort(plans.begin(), plans.end(), planComparator); // sort by plan value, ascending
    
    printf("A%d plans:\n", id);
    for (int i = 0; i < (int) plans.size(); ++i)
        printf("P%d -> B%d, score: %f\n", i, plans[i].binId, plans[i].value);
}

void AutoAgent::removePlan(int binId)
{
    for (int p = 0; p < (int) plans.size(); ++p) {
        if (plans[p].binId == binId) {
            plans.erase(plans.begin() + p);
            --p;
        }
    }
}

void AutoAgent::selectPlan(std::vector<AutoAgent> &agents, std::vector<AppleBin> bins)
{
    printf("A%d has %d plans.\n", id, (int) plans.size());
    
    if (plans.size() == 0)
        return;
    
    for (int p = 0; p < (int) plans.size(); ++p) {
        int minId = -1;
        float minVal = FLT_MAX;
        for (int a = 0; a < (int) agents.size(); ++a) {
            if (agents[a].id == id || agents[a].plans.size() == 0 || agents[a].plans[0].binId != plans[p].binId)
                continue;
            if (agents[a].plans[0].value < minVal) {
                minId = agents[a].id;
                minVal = agents[a].plans[0].value;
            }
        }
        
        if (minId == -1 || (minId != -1 && plans[p].value <= minVal)) {
            activePlan.binId = plans[p].binId;
            activePlan.value = plans[p].value;
            // Set target bin ID and location
            int idx = getBinIndexById(bins, activePlan.binId);
            targetBinId = plans[p].binId;
            targetLoc = bins[idx].loc;
            printf("A%d select plan: B%d at (%d,%d) (score: %4.2f).\n", id, targetBinId, targetLoc.x, targetLoc.y, 
                activePlan.value);
            // Broadcast to other agents that the bin is taken
            for (int a = 0; a < (int) agents.size(); ++a) {
                if (agents[a].id != id)
                    agents[a].removePlan(targetBinId);
            }
            return;
        }
    }
}

bool AutoAgent::areSameStates(AutoState s1, AutoState s2)
{
    if (s1.binStepCount == s2.binStepCount && s1.locStepCount == s2.locStepCount 
        && s1.binToLocStepCount == s2.binToLocStepCount && s1.binEstFullTime == s2.binEstFullTime)
        return true;
    return false;
}

int AutoAgent::getStateIndex(AutoState s)
{
    for (int i = 0; i < (int) states.size(); ++i) {
        if (areSameStates(states[i], s))
            return i;
    }
    return -1;
}

bool AutoAgent::isLocationServed(Coordinate loc, std::vector<AutoAgent> agents)
{
    for (int a = 0; a < (int) agents.size(); ++a) {
        if (agents[a].activeLocation.x == loc.x && agents[a].activeLocation.y == loc.y)
            return true;
    }
    return false;
}

Coordinate AutoAgent::selectClosestLocationRequest(Coordinate loc, std::vector<LocationRequest> requests, 
    std::vector<AutoAgent> agents)
{
    int minIdx = -1;
    int minStep = INT_MAX;
    for (int i = 0; i < (int) requests.size(); ++i) {
        if (isLocationServed(requests[i].loc, agents))
            continue;
        int tmp = getStepCount(loc, requests[i].loc);
        if (tmp < minStep) {
            minIdx = i;
            minStep = tmp;
        }
    }
    
    if (requests.size() == 0 || minIdx == -1) {
        return Coordinate(-1, -1);
    }
    
    return requests[minIdx].loc;
}

Coordinate AutoAgent::selectLocationRequest(std::vector<LocationRequest> requests, AppleBin ab, 
    std::vector<AutoAgent> agents, int *stateIndex)
{
    if (requests.size() == 0)
        return Coordinate(-1, -1);
    
    std::vector<int> tmpIndexes;
    std::vector<AutoState> tmpStates;
    for (int i = 0; i < (int) requests.size(); ++i) {
        if (isLocationServed(requests[i].loc, agents))
            continue;
        int binSC = (targetBinId == -1) ? 0 : getStepCount(curLoc, targetLoc);
        int locSC = getStepCount(curLoc, requests[i].loc);
        int diffSC = (targetBinId == -1) ? 0 : getStepCount(targetLoc, requests[i].loc);
        float remCapacity = BIN_CAPACITY - round(ab.capacity);
        int estTime = ceil(remCapacity / ab.fillRate);
        AutoState s = AutoState(binSC, locSC, diffSC, estTime);
        int idx = getStateIndex(s);
        if (idx == -1) { // add new state to learning vector
            states.push_back(s);
            idx = states.size() - 1;
        }
        s.reward = states[idx].reward;
        tmpIndexes.push_back(idx);
        tmpStates.push_back(s);
    }
    
    if (tmpStates.size() == 0)
        return Coordinate(-1, -1);
    
    int maxIdx = 0;
    float maxReward = tmpStates[maxIdx].reward;
    for (int i = 1; i < (int) tmpStates.size(); ++i) {
        if (tmpStates[i].reward > maxReward) {
            maxIdx = i;
            maxReward = tmpStates[i].reward;
        }
    }
    
    (*stateIndex) = tmpIndexes[maxIdx];
    return requests[maxIdx].loc;
}

bool AutoAgent::isLocationValid(Coordinate l)
{
    return (l.x >= 0 && l.x < ORCH_COLS && l.y >= 0 && l.y < ORCH_ROWS);
}

void AutoAgent::move(Coordinate loc, std::vector<AppleBin> &bins, int index)
{
    if (!isLocationValid(loc))
        return;
    
    int speed = AGENT_SPEED_H;
    if (curBinId != -1 && index != -1 && bins[index].capacity > 0)
        speed = AGENT_SPEED_L;
    
    for (int s = 0; s < speed; ++s) {
        if (loc.x == 0) { // Target location is the repo
            if (curLoc.x != loc.x)
                curLoc.x = (curLoc.x - 1 <= loc.x) ? loc.x : curLoc.x - 1; // Move left
        } else {
            if (curLoc.y == loc.y) {
                if (loc.x > curLoc.x)
                    curLoc.x = (curLoc.x + 1 >= loc.x) ? loc.x : curLoc.x + 1; // Move right
                else
                    curLoc.x = (curLoc.x - 1 <= loc.x) ? loc.x : curLoc.x - 1; // Move left
            } else { // Need to travel between rows
                if (curLoc.x == 0 || curLoc.x == ORCH_COLS - 1) { // At left/rightmost column; can travel between rows
                    if (loc.y > curLoc.y)
                        curLoc.y = (curLoc.y + 1 >= loc.y) ? loc.y : curLoc.y + 1; // Move down
                    else
                        curLoc.y = (curLoc.y - 1 <= loc.y) ? loc.y : curLoc.y - 1; // Move up
                } else {
                    int leftDist = curLoc.x;
                    int rightDist = ORCH_COLS - 1 - curLoc.x;
                    if (leftDist < rightDist) // Travel between rows through the leftmost column
                        curLoc.x = (curLoc.x - 1 <= loc.x) ? curLoc.x - 1 : loc.x; // Move left
                    else
                        curLoc.x = (curLoc.x + 1 >= loc.x) ? curLoc.x + 1 : loc.x; // Move right
                }
            }
        }
    }
    
    if (index >= 0 && index < (int) bins.size())
        bins[index].loc = curLoc;
}

void AutoAgent::removeLocationRequest(Coordinate loc, std::vector<LocationRequest> &requests)
{
    for (int i = 0; i < (int) requests.size(); ++i) {
        if (requests[i].loc.x == loc.x && requests[i].loc.y == loc.y) {
            requests.erase(requests.begin() + i);
            --i;
        }
    }
}

int AutoAgent::getRequestTime(Coordinate loc, std::vector<LocationRequest> requests)
{
    for (int i = 0; i < (int) requests.size(); ++i) {
        if (requests[i].loc.x == loc.x && requests[i].loc.y == loc.y)
            return requests[i].regisTime;
    }
    return -1;
}

float AutoAgent::getCFReward(std::vector<LocationRequest> requests, AppleBin ab)
{
    float sum = 0;
    float count = 0;
    for (int i = 0; i < (int) requests.size(); ++i) {
        if (requests[i].regisTime > lastDecisionTime)
            continue;
        float hTime = ((float) getStepCount(lastDecisionLoc, requests[i].loc)) / AGENT_SPEED_H;
        float bTime = hTime + ((float) getStepCount(requests[i].loc, ab.loc)) / AGENT_SPEED_H;
        sum += -(hTime * C_H + bTime + C_B);
        count++;
    }
    
    return sum / count;
}

void AutoAgent::takeAction(int *binCounter, std::vector<AppleBin> &bins, std::vector<LocationRequest> &requests, 
    std::vector<AutoAgent> &agents, std::vector<AppleBin> &repo, Orchard env, int curTime)
{
    int tIdx = getBinIndexById(bins, targetBinId);
    if (tIdx != -1 && curLoc.x == 0) {
        if (env.getApplesAt(bins[tIdx].loc) - BIN_CAPACITY > 0 && curLoc.x == 0) {
            curBinId = (*binCounter)++;
            bins.push_back(AppleBin(curBinId, curLoc.x, curLoc.y));
            activeLocation = targetLoc;
            // save history for calculating reward
            lastDecisionTime = curTime;
            lastDecisionLoc = curLoc;
            lastActiveLoc = activeLocation;
        }
    }
    
    if (activeStateIndex == -1 && curLoc.x == 0 && curBinId == -1) {
        if (useLearning)
            activeLocation = selectLocationRequest(requests, bins[tIdx], agents, &activeStateIndex);
        else
            activeLocation = selectClosestLocationRequest(bins[tIdx].loc, requests, agents);
        printf("A%d selects location request (%d,%d).\n", id, activeLocation.x, activeLocation.y);
        // save history for calculating reward
        lastDecisionTime = curTime;
        lastDecisionLoc = curLoc;
        lastActiveLoc = activeLocation;
    }
    
    //removeLocationRequest(activeLocation, locRequests);
    
    if (isLocationValid(activeLocation)) {
        if (curBinId == -1 && curLoc.x == 0) { // get a new bin
            curBinId = (*binCounter)++;
            bins.push_back(AppleBin(curBinId, curLoc.x, curLoc.y));
            printf("A%d takes a new bin B%d to (%d,%d).\n", id, curBinId, activeLocation.x, activeLocation.y);
        }
        
        int cIdx = getBinIndexById(bins, curBinId);
        if (curLoc.x == activeLocation.x && curLoc.y == activeLocation.y) {
            if (curBinId != -1) { // arrived at requested location; drop the new bin
                bins[cIdx].loc = curLoc;
                bins[cIdx].onGround = true;
                printf("A%d(%d,%d) drops B%d at (%d,%d).\n", id, curLoc.x, curLoc.y, bins[cIdx].id, 
                    bins[cIdx].loc.x, bins[cIdx].loc.y);
                int regisTime = getRequestTime(activeLocation, requests);
                humanWaitTime = (regisTime == -1) ? 0 : curTime - regisTime;
                curBinId = -1;
                cIdx = -1;
                activeLocation = Coordinate(-1, -1); // reset
            }
        } else {
            move(activeLocation, bins, cIdx);
            printf("A%d moves to (%d,%d). Active location: (%d,%d).\n", id, curLoc.x, curLoc.y, 
                activeLocation.x, activeLocation.y);
            return;
        }
    }
    
    if (isLocationValid(targetLoc)) {
        if (curLoc.x == targetLoc.x && curLoc.y == targetLoc.y) { // arrived at target bin location
            if (round(bins[tIdx].capacity) >= BIN_CAPACITY) { // bin is full; pick it up
                curBinId = targetBinId;
                int cIdx = getBinIndexById(bins, curBinId);
                bins[cIdx].onGround = false;
                targetBinId = -1; // reset
                targetLoc.x = 0; // destination is set to repo
                printf("A%d picks up B%d at (%d,%d). Current destination: Repo (%d,%d).\n", id, curBinId, 
                    curLoc.x, curLoc.y, targetLoc.x, targetLoc.y);
                binWaitTime = (bins[cIdx].filledTime == -1) ? 0 : curTime - bins[cIdx].filledTime;
            } else { // bin is not full yet; wait
                printf("A%d(%d,%d) waits for B%d(%d,%d) to be full.\n", id, curLoc.x, curLoc.y, targetBinId, 
                    bins[tIdx].loc.x, bins[tIdx].loc.y);
                return;
            }
        } else {
            int cIdx = getBinIndexById(bins, curBinId);
            move(targetLoc, bins, cIdx);
            printf("A%d moves to (%d,%d). Target location: (%d,%d).\n", id, curLoc.x, curLoc.y, targetLoc.x, targetLoc.y);
            return;
        }
    } else { // no active location request and no target bin; return to repo
        targetLoc.x = 0;
    }
    
    if (targetBinId != -1)
        targetLoc = bins[tIdx].loc;
    
    int idx = getBinIndexById(bins, curBinId);
    move(targetLoc, bins, idx);
    printf("A%d moves to (%d,%d).\n", id, curLoc.x, curLoc.y);
    
    if (curLoc.x == 0 && curBinId != -1 && bins[idx].capacity > 0) { // arrived at repo with full bin
        if (idx >= 0 && idx < (int) bins.size()) {
            if (activeStateIndex != -1) { // Observe reward
                float rA = -(humanWaitTime * C_H + binWaitTime * C_B);
                float rCF = getCFReward(requests, bins[idx]);
                float reward = rA - rCF;
                states[activeStateIndex].reward += reward; 
            }
            // Put carried bin in repo
            repo.push_back(copyBin(bins[idx]));
            printf("A%d(%d,%d) put B%d in Repo.\n", id, curLoc.x, curLoc.y, curBinId);
            bins.erase(bins.begin() + idx);
            // Reset all
            curBinId = -1;
            targetBinId = -1;
            targetLoc = Coordinate(-1, -1);
            activeStateIndex = -1;
            lastDecisionTime = -1;
            lastDecisionLoc = Coordinate(-1, -1);
            lastActiveLoc = Coordinate(-1, -1);
            binWaitTime = 0;
            humanWaitTime = 0;
        }
    }
}

AppleBin AutoAgent::copyBin(AppleBin ab)
{
    AppleBin nb(ab.id, ab.loc.x, ab.loc.y);
    nb.capacity = ab.capacity;
    return nb;
}



