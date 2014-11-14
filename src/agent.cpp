#include <algorithm>
#include <cstddef>
#include <climits>
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

std::vector<AppleBin> Agent::getIdleBins(std::vector<AppleBin> bins, std::vector<Agent> agents)
{
    std::vector<AppleBin> idleBins = bins;
    
    for (int a = 0; a < NUM_AGENTS; ++a) {
        if (agents[a].id == id)
            continue;
        // Remove bins that are being carried by another agent
        int bIdx = getBinIndexById(idleBins, agents[a].getCurBinId());
        if (bIdx != -1)
            idleBins.erase(idleBins.begin() + bIdx);
        // Remove bins that are being targeted by another agent
        int tIdx = getBinIndexById(idleBins, agents[a].getTargetBinId());
        if (tIdx != -1)
            idleBins.erase(idleBins.begin() + tIdx);
    }
    
    return idleBins;
}

int Agent::getDistance(Coordinate loc1, Coordinate loc2)
{
    return abs(loc1.x - loc2.x) + abs(loc1.y - loc2.y);
}

bool binCapacityComparator(AppleBin b1, AppleBin b2)
{
    return b1.capacity > b2.capacity;
}

bool binEstFullnessComparator(AppleBin b1, AppleBin b2)
{
    return b1.estCapacity > b2.estCapacity;
}

void Agent::sortByEstFullness(std::vector<AppleBin> &bins)
{
    for (int b = 0; b < (int) bins.size(); ++b) {
        float estTime = getDistance(curLoc, bins[b].loc) / AGENT_SPEED_L;
        float estIncrease = estTime * bins[b].fillRate;
        bins[b].estCapacity = bins[b].capacity + (estIncrease / BIN_CAPACITY);
    }
    sort(bins.begin(), bins.end(), binEstFullnessComparator);
}

int Agent::getClosestFullBin(std::vector<AppleBin> bins)
{
    if (bins.size() == 0)
        return -1;
    
    int minBinIdx = -1;
    int minDist = INT_MAX;
    for (int b = 0; b < (int) bins.size(); ++b) {
        if (bins[b].capacity < BIN_CAPACITY)
            continue;
        int dist = getDistance(curLoc, bins[b].loc);
        if (dist < minDist) {
            minBinIdx = getBinIndexById(bins, b);
            minDist = dist;
        }
    }
    return minBinIdx;
}

bool Agent::isLocationValid(Coordinate loc)
{
    return (loc.x >= 0 && loc.x < ORCH_COLS && loc.y >= 0 && loc.y < ORCH_ROWS);
}

void Agent::move(AppleBin *curBin)
{
    // Check if all locations are valid
    if (!isLocationValid(curLoc) || !isLocationValid(targetLoc))
        return;
    
    int speed = AGENT_SPEED_H;
    if (curBinId != -1 && curBin->capacity > 0)
        speed = AGENT_SPEED_L;
    
    if (targetLoc.x == 0) { // Target location is the repo
        if (curLoc.x != targetLoc.x)
            curLoc.x = (curLoc.x - speed <= targetLoc.x) ? targetLoc.x : curLoc.x - speed; // Move left
    } else {
        if (curLoc.y == targetLoc.y) {
            if (targetLoc.x > curLoc.x)
                curLoc.x = (curLoc.x + speed >= targetLoc.x) ? targetLoc.x : curLoc.x + speed; // Move right
            else
                curLoc.x = (curLoc.x - speed <= targetLoc.x) ? targetLoc.x : curLoc.x - speed; // Move left
        } else { // Need to travel between rows
            if (curLoc.x == 0 || curLoc.x == ORCH_COLS - 1) { // At left/rightmost column; can travel between rows
                if (targetLoc.y > curLoc.y)
                    curLoc.y = (curLoc.y + speed >= targetLoc.y) ? targetLoc.y : curLoc.y + speed; // Move down
                else
                    curLoc.y = (curLoc.y - speed <= targetLoc.y) ? targetLoc.y : curLoc.y - speed; // Move up
            } else {
                int leftDist = curLoc.x;
                int rightDist = ORCH_COLS - 1 - curLoc.x;
                if (leftDist < rightDist) // Travel between rows through the leftmost column
                    curLoc.x = (curLoc.x - speed <= targetLoc.x) ? targetLoc.x : curLoc.x - speed; // Move left
                else
                    curLoc.x = (curLoc.x + speed >= targetLoc.x) ? targetLoc.x : curLoc.x + speed; // Move right
            }
        }
    }
    
    if (curBinId != -1)
        curBin->loc = curLoc;
}

void Agent::takeAction(int *binCounter, std::vector<AppleBin> &bins, std::vector<AppleBin> &repo, 
    std::vector<Agent> &agents, Orchard env, std::vector<Coordinate> &newLocs)
{
    if (targetLoc.x == -1 || targetLoc.y == -1 || curLoc.x == 0) {
        /* Agent is idle or at repo (or target is not a valid location in orchard, in which case agent is corrupted) */
        if (curBinId != -1) {
            int idx = getBinIndexById(bins, curBinId);
            repo.push_back(bins[idx]); // Put carried bin in repo
            printf("A%d put B%d in repo.\n", id, curBinId);
        }
        if (newLocs.size() > 0) {
            Coordinate newLoc = newLocs[0];
            bool newLocFlag = false;
            for (int a = 0; a < (int) agents.size() && !newLocFlag; ++a) {
                Coordinate tmp = agents[a].getTargetLoc();
                int tmpBinId = agents[a].getCurBinId();
                if (tmp.x == newLoc.x && tmp.y == newLoc.y && tmpBinId != -1 && bins[tmpBinId].capacity == 0)
                    newLocFlag = true; // An agent is already on the way to the new location with empty bin
            }
            if (!newLocFlag) {
                int newBinId = bins.size() + repo.size();
                bins.push_back(AppleBin(newBinId, newLoc.x, newLoc.y));
                targetBinId = newBinId;
                targetLoc = newLoc;
                newLocs.erase(newLocs.begin());
                int cIdx = getBinIndexById(bins, curBinId);
                move(&bins[cIdx]);
                printf("A%d carries new bin to (%d,%d).\n", id, targetLoc.x, targetLoc.y);
            }
        } else {
            // Find a new bin to be picked up
            std::vector<AppleBin> idleBins = getIdleBins(bins, agents);
            if (idleBins.size() > 0) { // There are idle bins
                // Choose an existing bin to pick up
                int tmpIdx = getClosestFullBin(idleBins);
                if (tmpIdx != -1) {
                    targetBinId = idleBins[tmpIdx].id;
                    targetLoc = idleBins[tmpIdx].loc;
                } else {
                    sortByEstFullness(idleBins);
                    targetBinId = idleBins[0].id;
                    targetLoc = idleBins[0].loc;
                }
                if (env.getApplesAt(targetLoc) > 0) // There are still apples in target location; need a new bin
                    bins.push_back(AppleBin((*binCounter)++, curLoc.x, curLoc.y));
                int cIdx = getBinIndexById(bins, curBinId);
                move(&bins[cIdx]);
                printf("A%d moves to pick up B%d at (%d,%d).\n", id, targetBinId, targetLoc.x, targetLoc.y);
            }
        }
    } else {
        /* Agent is not idle (i.e. moving towards a bin or waiting for a bin) */
        if (curBinId == targetBinId) { // Agent is carrying the target bin, go to repo (column 0 at every row)
            targetLoc = getRepoLocation();
            int cIdx = getBinIndexById(bins, curBinId);
            move(&bins[cIdx]);
            printf("A%d moves to (%d,%d).\n", id, curLoc.x, curLoc.y);
        } else { // Agent is on the way to pick up the target bin; it may or may not be carrying an empty bin
            if (curLoc.x == targetLoc.x && curLoc.y == targetLoc.y) { // Arrived at target location
                printf("A%d arrives at target (%d,%d).\n", id, targetLoc.x, targetLoc.y);
                int tIdx = getBinIndexById(bins, targetBinId);
                float cap = bins[tIdx].capacity;
                if (cap >= BIN_CAPACITY) { // Target bin is full
                    if (curBinId != -1) { // Agent is carrying an empty bin
                        int eIdx = getBinIndexById(bins, curBinId);
                        bins[eIdx].loc = curLoc;
                    }
                    curBinId = targetBinId; // pick up target bin
                    printf("A%d picks up B%d.\n", id, curBinId);
                    targetLoc = getRepoLocation();
                    int cIdx = getBinIndexById(bins, curBinId);
                    move(&bins[cIdx]);
                    printf("A%d moves to (%d,%d).\n", id, curLoc.x, curLoc.y);
                } else { // else, if agent has arrived at target location, but target bin is not full yet, agent waits
                    int cIdx = getBinIndexById(bins, curBinId);
                    printf("A%d is waiting for B%d to be filled.\n", id, bins[cIdx].id);
                }
            } else {
                int cIdx = getBinIndexById(bins, curBinId);
                move(&bins[cIdx]);
                printf("A%d moves to (%d,%d).\n", id, curLoc.x, curLoc.y);
            }
        }
    }
}


