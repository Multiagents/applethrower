#include <algorithm>
#include <cmath>
#include <cstddef>
#include <climits>
#include <cfloat>
#include "params.hpp"
#include "agent.hpp"

Agent::Agent(int i, Coordinate c)
{
    id = i;
    curLoc = c;
    targetLoc = Coordinate(0, 0);
    curBinId = -1;
    targetBinId = -1;
}

Agent::~Agent()
{
    curBinId = -1;
    targetBinId = -1;
}

int Agent::getBinIndexById(std::vector<AppleBin> bins, int id)
{
    for (int b = 0; b < (int) bins.size(); ++b) {
        if (bins[b].id == id)
            return b;
    }
    return -1;
}

std::vector<int> Agent::getIdleBins(std::vector<AppleBin> bins, std::vector<Agent> agents)
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
        int stepCount = getStepCount(agents[a].curLoc, agents[a].targetLoc);
        if ((bIdx != -1 && stepCount == 0)) {
            printf("This one should be good: B%d\n", agents[a].getCurBinId());
            idleBins[bIdx] = bIdx;
        }  
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

int Agent::getStepCount(Coordinate src, Coordinate dst)
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

int Agent::getFirstEstFullBin(std::vector<int> indexes, std::vector<AppleBin> bins)
{
    if (indexes.size() == 1)
        return indexes[0];
    
    int maxBinIdx = -1;
    float maxEstCap = 0;
    float minDist = FLT_MAX;
    
    for (int i = 0; i < (int) indexes.size(); ++i) {
        float dist = getStepCount(curLoc, bins[indexes[i]].loc);
        float estTime = dist / AGENT_SPEED_L;
        float estIncrease = estTime * bins[indexes[i]].fillRate;
        float estCap = bins[indexes[i]].capacity + estIncrease;
        estCap = (estCap >= BIN_CAPACITY) ? BIN_CAPACITY : estCap;
        if ((estCap == maxEstCap && dist < minDist) || (estCap > maxEstCap)) {
            maxBinIdx = indexes[i];
            maxEstCap = estCap;
            minDist = dist;
        }
    }
    
    return maxBinIdx;
}

int Agent::getClosestFullBin(std::vector<int> indexes, std::vector<AppleBin> bins)
{
    if (indexes.size() == 0)
        return -1;
    
    int minBinIdx = -1;
    int minDist = INT_MAX;
    for (int b = 0; b < (int) indexes.size(); ++b) {
        if (bins[indexes[b]].capacity < BIN_CAPACITY)
            continue;
        int dist = getStepCount(curLoc, bins[indexes[b]].loc);
        if (dist < minDist) {
            minBinIdx = indexes[b];
            minDist = dist;
        }
    }
    return minBinIdx;
}

bool Agent::isLocationValid(Coordinate loc)
{
    return (loc.x >= 0 && loc.x < ORCH_COLS && loc.y >= 0 && loc.y < ORCH_ROWS);
}

void Agent::move(AppleBin curBin)
{
    // Check if all locations are valid
    if (!isLocationValid(curLoc) || !isLocationValid(targetLoc))
        return;
    
    int speed = AGENT_SPEED_H;
    if (curBinId != -1 && curBin.capacity > 0)
        speed = AGENT_SPEED_L;
    
    for (int s = 0; s < speed; ++s) {
        if (targetLoc.x == 0) { // Target location is the repo
            if (curLoc.x != targetLoc.x)
                curLoc.x = (curLoc.x - 1 <= targetLoc.x) ? targetLoc.x : curLoc.x - 1; // Move left
        } else {
            if (curLoc.y == targetLoc.y) {
                if (targetLoc.x > curLoc.x)
                    curLoc.x = (curLoc.x + 1 >= targetLoc.x) ? targetLoc.x : curLoc.x + 1; // Move right
                else
                    curLoc.x = (curLoc.x - 1 <= targetLoc.x) ? targetLoc.x : curLoc.x - 1; // Move left
            } else { // Need to travel between rows
                if (curLoc.x == 0 || curLoc.x == ORCH_COLS - 1) { // At left/rightmost column; can travel between rows
                    if (targetLoc.y > curLoc.y)
                        curLoc.y = (curLoc.y + 1 >= targetLoc.y) ? targetLoc.y : curLoc.y + 1; // Move down
                    else
                        curLoc.y = (curLoc.y - 1 <= targetLoc.y) ? targetLoc.y : curLoc.y - 1; // Move up
                } else {
                    int leftDist = curLoc.x;
                    int rightDist = ORCH_COLS - 1 - curLoc.x;
                    if (leftDist < rightDist) // Travel between rows through the leftmost column
                        curLoc.x = (curLoc.x - 1 <= targetLoc.x) ? curLoc.x - 1 : targetLoc.x; // Move left
                    else
                        curLoc.x = (curLoc.x + 1 >= targetLoc.x) ? curLoc.x + 1 : targetLoc.x; // Move right
                }
            }
        }
    } // end for
}

void Agent::filterRegisteredLocations(std::vector<LocationRequest> &requests)
{
    for (int n = 0; n < (int) requests.size(); ++n) {
        if (requests[n].loc.x == targetLoc.x && requests[n].loc.y == targetLoc.y) {
            requests.erase(requests.begin() + n);
            --n;
        }
    }
}

int Agent::checkIfCarryingBin(std::vector<Agent> &agents, std::vector<AppleBin> &bins, Coordinate loc)
{
    for (int a = 0; a < (int) agents.size(); ++a) {
        Coordinate tmp = agents[a].getTargetLoc();
        if (tmp.x == loc.x && tmp.y == loc.y) {
            printf("This one is going there: A%d\n", a);
            int binId = agents[a].getCurBinId();
            printf("A%d 's bin ID = %d\n", a, binId);
            if (binId != -1) {
                return binId;
            }    
        }
    }
    return -1;
}

int Agent::getBinIndexByLocation(std::vector<Agent> &agents, std::vector<AppleBin> &bins, Coordinate loc)
{
    for (int b = 0; b < (int) bins.size(); ++b) {
        // bug fixed here. If the bin on ground is not full, return b.
        if (bins[b].loc.x == loc.x && bins[b].loc.y == loc.y) {
            int carry = checkIfCarryingBin(agents, bins, loc);
            if (carry != -1) {
                return b;
            }
        }     
    }
    return -1;
}

Coordinate Agent::selectNewLocation(std::vector<Agent> &agents, std::vector<AppleBin> &bins, 
    std::vector<LocationRequest> &requests)
{
    for (int n = 0; n < (int) requests.size(); ++n) {
        int idx = getBinIndexByLocation(agents, bins, requests[n].loc);
        if (idx == -1) {
            return requests[n].loc;
        }
    }
    return Coordinate(-1,-1);
}

void Agent::takeAction(int *binCounter, std::vector<AppleBin> &bins, std::vector<AppleBin> &repo, 
    std::vector<Agent> &agents, Orchard env, std::vector<LocationRequest> &requests)
{
    if (targetBinId == -1 && curBinId == -1) { // Agent is idle
        // Find an idle bin to be picked up
        std::vector<int> idleBins = getIdleBins(bins, agents);
        if (idleBins.size() > 0) { // There are idle bins
            printf("A%d(%d,%d) sees %d idle bins.\n", id, curLoc.x, curLoc.y, (int) idleBins.size());
            // Choose an existing bin to pick up
            int tmpIdx = getClosestFullBin(idleBins, bins);
            if (tmpIdx != -1) {
                targetBinId = bins[tmpIdx].id;
                targetLoc = bins[tmpIdx].loc;
            } else {
                tmpIdx = getFirstEstFullBin(idleBins, bins);
                if (tmpIdx != -1) {
                    targetBinId = bins[tmpIdx].id;
                    targetLoc = bins[tmpIdx].loc;
                }
            }
            if (targetBinId != -1) {
                int tIdx = getBinIndexById(bins, targetBinId);
                if (env.getApplesAt(bins[tIdx].loc) - BIN_CAPACITY > 0 && curLoc.x == 0) {
                    curBinId = (*binCounter)++;
                    printf("A%d takes a new bin B%d to (%d,%d).\n", id, curBinId, targetLoc.x, targetLoc.y);
                    bins.push_back(AppleBin(curBinId, curLoc.x, curLoc.y));
                }
                int cIdx = getBinIndexById(bins, curBinId);
                move(bins[cIdx]);
                if (cIdx != -1)
                    bins[cIdx].loc = curLoc;
                printf("A%d(%d,%d) moves to pick up B%d at (%d,%d).\n", id, curLoc.x, curLoc.y, targetBinId, 
                    targetLoc.x, targetLoc.y);
            }
        } else if (requests.size() > 0) { // There's a new harvest location without bin
            printf("A%d sees %d new locations without bins.\n", id, (int) requests.size());
            Coordinate newLoc = selectNewLocation(agents, bins, requests);
            if (newLoc.x != -1 && newLoc.y != -1) { // There's a registered location without any bin
                if (curLoc.x != 0 && curBinId == -1){ // Agent is in orchard and carries no bin
                     targetLoc = getRepoLocation();
                     int cIdx = getBinIndexById(bins, curBinId);
                     move(bins[cIdx]);
                     printf("A%d sees %d new locations without bins, moves back to repo to get a new bin. (%d,%d)\n", 
                        id, (int) requests.size(),curLoc.x,curLoc.y);
                } else {
                        curBinId = (*binCounter);
                        (*binCounter)++;
                        bins.push_back(AppleBin(curBinId, curLoc.x, curLoc.y));
                        targetBinId = -1;
                        targetLoc = requests[0].loc;
                        requests.erase(requests.begin());
                        int cIdx = getBinIndexById(bins, curBinId);
                        move(bins[cIdx]);
                        if (cIdx != -1)
                            bins[cIdx].loc = curLoc;
                        printf("A%d(%d,%d) carries new bin B%d to (%d,%d).\n", id, curLoc.x, curLoc.y, bins[cIdx].id, 
                            targetLoc.x, targetLoc.y);
                }
            } else {
                printf("Another agent is taking care of that. A%d (%d,%d) is idle.\n", id, curLoc.x, curLoc.y);
            }
        } else {
            printf("A%d(%d,%d) is idle.\n", id, curLoc.x, curLoc.y);
        }
    } else {
        /* Agent is not idle (i.e. moving towards a bin or waiting for a bin) */
        int curBinIdx = getBinIndexById(bins, curBinId);
        if (curBinId != -1 && bins[curBinIdx].capacity == 0) // Agent is carrying an empty bin to a location
            filterRegisteredLocations(requests); // If the target location is in the new location list, remove it
        if (curBinId == targetBinId || (curBinId != -1 && bins[curBinIdx].capacity >= BIN_CAPACITY)) {
            // Agent is carrying the target bin, go to repo (column 0 at every row)
            targetLoc = getRepoLocation();
            int cIdx = getBinIndexById(bins, curBinId);
            move(bins[cIdx]);
            if (cIdx != -1)
                bins[cIdx].loc = curLoc;
            printf("A%d moves to (%d,%d). Target: (%d,%d). Destination: REPO.\n", id, curLoc.x, curLoc.y, 
                targetLoc.x, targetLoc.y);
        } else { // Agent is on the way to pick up the target bin; it may or may not be carrying an empty bin
            if (curLoc.x == targetLoc.x && curLoc.y == targetLoc.y) { // Arrived at target location
                printf("A%d arrives at target (%d,%d). CurBinId: %d\n", id, targetLoc.x, targetLoc.y, curBinId);
                if (targetBinId == -1 && curBinId != -1) {
                    int eIdx = getBinIndexById(bins, curBinId);
                    bins[eIdx].loc = curLoc;
                    bins[eIdx].onGround = true;
                    printf("A%d(%d,%d) drops B%d at (%d,%d).\n", id, curLoc.x, curLoc.y, bins[eIdx].id, 
                        bins[eIdx].loc.x, bins[eIdx].loc.y);
                    curBinId = -1;
                } else {
                    int tIdx = getBinIndexById(bins, targetBinId);
                    if (round(bins[tIdx].capacity) >= BIN_CAPACITY) { // Target bin is full
                        if (curBinId != -1) { // Agent is carrying an empty bin
                            int eIdx = getBinIndexById(bins, curBinId);
                            bins[eIdx].loc = curLoc;
                            bins[eIdx].onGround = true;
                            printf("A%d(%d,%d) drops B%d at (%d,%d).\n", id, curLoc.x, curLoc.y, bins[eIdx].id, 
                                bins[eIdx].loc.x, bins[eIdx].loc.y);
                        }
                        curBinId = targetBinId; // pick up target bin
                        int cIdx = getBinIndexById(bins, curBinId);
                        printf("A%d(%d,%d) picks up B%d(%d,%d).\n", id, curLoc.x, curLoc.y, curBinId, 
                            bins[cIdx].loc.x, bins[cIdx].loc.y);
                        targetLoc = getRepoLocation();
                        move(bins[cIdx]);
                        if (cIdx != -1) {
                            bins[cIdx].capacity = round(bins[cIdx].capacity);
                            bins[cIdx].loc = curLoc;
                            bins[cIdx].onGround = false;
                        }
                        printf("A%d moves to (%d,%d). TargetBin: B%d.\n", id, curLoc.x, curLoc.y, targetBinId);
                    } else { // agent arrived at target location, but target bin is not full yet; agent waits
                        int tIdx = getBinIndexById(bins, targetBinId);
                        printf("A%d(%d,%d) is waiting for B%d(%d,%d) to be filled.\n", id, curLoc.x, curLoc.y, 
                            bins[tIdx].id, bins[tIdx].loc.x, bins[tIdx].loc.y);
                    }
                }
            } else {
                int cIdx = getBinIndexById(bins, curBinId);
                move(bins[cIdx]);
                if (cIdx != -1)
                    bins[cIdx].loc = curLoc;
                printf("A%d moves to (%d,%d). CurBin: B%d. Target: B%d.\n", id, curLoc.x, curLoc.y, 
                    curBinId, targetBinId);
            }
        }
    }
    
    if (curLoc.x == 0) {
        // Arrived at REPO
        int carriedCapacity = round(bins[getBinIndexById(bins, curBinId)].capacity);
        if (curBinId != -1 && carriedCapacity >= BIN_CAPACITY) { // Carrying a full bin
            int idx = getBinIndexById(bins, curBinId);
            if (idx >= 0 && idx < (int) bins.size()) {
                repo.push_back(copyBin(bins[idx])); // Put carried bin in repo
                printf("A%d(%d,%d) put B%d in REPO.\n", id, curLoc.x, curLoc.y, curBinId);
                bins.erase(bins.begin() + idx);
                // Reset all
                curBinId = -1;
                targetBinId = -1;
                targetLoc = Coordinate(-1, -1);
            } else {
                printf("A%d current bin ID B%d, index %d?\n", id, curBinId, idx);
            }
        }
    }
}

AppleBin Agent::copyBin(AppleBin ab)
{
    AppleBin nb(ab.id, ab.loc.x, ab.loc.y);
    nb.capacity = ab.capacity;
    return nb;
}



