#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <ctime>
#include <cstring>
#include <vector>
#include "data_structs.hpp"
#include "params.hpp"
#include "orchard.hpp"
#include "agent.hpp"
#include "auto_agent.hpp"

FILE *logFile;
FILE *repoFile;
std::vector<FILE*> agentFiles;

int parseArgInt(char *arg)
{
    char *tmp = strtok(arg, "=");
    tmp = strtok(NULL, "=");
    return atoi(tmp);
}

int getNumWorkersAt(std::vector<Worker> workers, Coordinate loc)
{
    int count = 0;
    for (int w = 0; w < (int) workers.size(); ++w) {
        if (workers[w].loc.x == loc.x && workers[w].loc.y == loc.y)
            count++;
    }
    return count;
}

Coordinate findNewAppleLocation(std::vector<Worker> workers, Coordinate curLoc, Orchard env)
{
    // Search location at the same row, to the right columns
    for (int c = curLoc.x + 1; c < ORCH_COLS - 1; ++c) {
        Coordinate tmp(c, curLoc.y);
        if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
            return tmp;
    }
    
    // Search location at the same row, to the left columns
    for (int c = curLoc.x - 1; c > 0; --c) {
        Coordinate tmp(c, curLoc.y);
        if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
            return tmp;
    }
    
    // Search location at a different row (down)
    for (int r = curLoc.y + 1; r < ORCH_ROWS; ++r) {
        for (int c = ORCH_COLS - 2; c > 0; --c) {
            Coordinate tmp(c, r); // Starts from the rightmost column
            if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
                return tmp;
        }
    }
    
    // Search location at a different row (up)
    for (int r = curLoc.y - 1; r >= 0; --r) {
        for (int c = ORCH_COLS - 2; c > 0; --c) {
            Coordinate tmp(c, r); // Starts from the rightmost column
            if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
                return tmp;
        }
    }
    
    // Still can't find a good location, join another group with the max ratio between apples and workers
    float maxRatio = 0;
    Coordinate maxLoc(-1, -1);
    for (int r = 0; r < ORCH_ROWS; ++r) {
        for (int c = 1; c < ORCH_COLS - 1; c++) {
            Coordinate tmp(c, r);
            if (env.getApplesAt(tmp) == 0)
                continue;
            if (getNumWorkersAt(workers, tmp) == 0) {
                return tmp;
            } else {
                float ratio = env.getApplesAt(tmp) / (float) getNumWorkersAt(workers, tmp);
                if (ratio > maxRatio) {
                    maxRatio = ratio;
                    maxLoc = tmp;
                }
            }
        }
    }
    return maxLoc;
}

Coordinate distributeWorkers(std::vector<Worker> &workers, std::vector<AppleBin> bins, Coordinate loc, Orchard env)
{
    Coordinate newLoc = findNewAppleLocation(workers, loc, env);
    
    // Move workers
    for (int w = 0; w < (int) workers.size(); ++w) {
        if (workers[w].loc.x != loc.x || workers[w].loc.y != loc.y)
            continue;
        workers[w].loc = newLoc;
    }
    
    return newLoc;
}

void writeBinInfo(const char *subdir, int time, AppleBin ab)
{
    char fname[50];
    sprintf(fname, "logs/%s/bins/bin%d.csv", subdir, ab.id);
    FILE *fp = fopen(fname, "a");
    fprintf(fp, "%d,%d,%d,%4.2f\n", time, ab.loc.x, ab.loc.y, ab.capacity);
    fclose(fp);
}

void registerLocation(Coordinate loc, std::vector<LocationRequest> &requests, int time)
{
    for (int i = 0; i < (int) requests.size(); ++i) {
        if (requests[i].loc.x == loc.x && requests[i].loc.y == loc.y)
            return;
    }
    requests.push_back(loc);
}

std::vector<Worker> initWorkers()
{
    std::vector<Worker> workers;
    for (int i = 0; i < NUM_WORKERS; ++i)
        workers.push_back(Worker(i, 0, 0));
    return workers;
}

std::vector<Coordinate> initWorkerGroupsRandom(std::vector<Worker> &workers)
{
    std::vector<Coordinate> workerGroups;
    int count = 0;
    
    while (count < NUM_WORKERS) {
        // Get random coordinate
        int x = rand() % (ORCH_COLS - 1) + 1;
        int y = rand() % ORCH_ROWS;
        // Get random number of workers for a group
        int num = rand() % 5 + 1;
        // Register workers' locations
        int prevCount = count;
        for (int n = prevCount; n < (prevCount + num) && n < NUM_WORKERS; ++n) {
            workers[n].loc.x = x;
            workers[n].loc.y = y;
            ++count;
            //printf("workers %d at location (%d, %d).\n", n, x, y);
        }
        workerGroups.push_back(Coordinate(x, y));
    }
    
    return workerGroups;
}

std::vector<Coordinate> initWorkerGroupsFixed(std::vector<Worker> &workers)
{
    std::vector<Coordinate> workerGroups;
    
    workerGroups.push_back(Coordinate(3, 2));
    workerGroups.push_back(Coordinate(4, 3));
    
    // Register workers' locations
    for (int i = 0; i < 5; ++i)
        workers[i].loc = workerGroups[0];
    for (int i = 5; i < 10; ++i)
        workers[i].loc = workerGroups[1];
    
    return workerGroups;
}

std::vector<AppleBin> initBins(std::vector<Coordinate> workerGroups, int *binCounter)
{
    std::vector<AppleBin> bins;
    
    for (int i = 0; i < (int) workerGroups.size(); ++i)
        bins.push_back(AppleBin((*binCounter)++, workerGroups[i].x, workerGroups[i].y));
    
    printf("Initial bin locations:\n");
    for (int b = 0; b < (int) bins.size(); ++b) {
        bins[b].onGround = true;
        printf("B%d at (%d,%d)\n", bins[b].id, bins[b].loc.x, bins[b].loc.y);
    }
    
    return bins;
}

bool isRequestFulfilled(Coordinate loc, std::vector<AppleBin> bins)
{
    for (int i = 0; i < (int) bins.size(); ++i) {
        if (bins[i].loc.x == loc.x && bins[i].loc.y == loc.y)
            return true;
    }
    return false;
}

void runBase(const int NUM_AGENTS, const int TIME_LIMIT)
{
    system("rm -rf logs/base");
    system("mkdir logs/base");
    system("mkdir -p logs/base/agents");
    system("mkdir -p logs/base/bins");
    
    // Prepare log files
    FILE *repoFile = fopen("logs/base/repo.csv", "w");
    
    int binCounter = 0;
    std::vector<Worker> workers = initWorkers();
    std::vector<Coordinate> workerGroups = initWorkerGroupsFixed(workers);
    std::vector<AppleBin> bins = initBins(workerGroups, &binCounter);
    
    std::vector<Agent> agents;
    for (int i = 0; i < NUM_AGENTS; ++i) {
        agents.push_back(Agent(i, Coordinate(0, 0)));
        char fname[50];
        sprintf(fname, "logs/base/agents/agent%d.csv", i);
        FILE *fp = fopen(fname, "w");
        agentFiles.push_back(fp);
    }
    
    /* Initialize orchard environment with uniform distribution of apples */
    Orchard env;
    std::vector<AppleBin> repo;
    std::vector<LocationRequest> requests;
    
    /* Run simulator */
    for (int t = 0; t < TIME_LIMIT; ++t) {
        printf("------------ T = %d ------------\n", t);
        // Simulate bins and workers
        // Harvest only happens when there's bin on the location. Downside: workers will have to wait for bins.
        for (int b = 0; b < (int) bins.size(); ++b) {
            int num = getNumWorkersAt(workers, bins[b].loc);
            int tmp1 = round(bins[b].capacity);
            int tmp2 = BIN_CAPACITY;
            if (env.getApplesAt(bins[b].loc) > 0 && tmp1 < tmp2 && bins[b].onGround) {
                bins[b].fillRate = num * PICK_RATE;
                bins[b].capacity += bins[b].fillRate; // capacity increase for each time step = fill rate * 1
                env.decreaseApplesAt(bins[b].loc, bins[b].fillRate);
                if (bins[b].capacity > BIN_CAPACITY)
                    bins[b].capacity = BIN_CAPACITY;
            }
            printf("[%d] B%d (%d,%d) capacity: %4.2f. (# workers: %d)\n", t, bins[b].id, bins[b].loc.x, 
                bins[b].loc.y, bins[b].capacity, num);
            if (bins[b].onGround) {
                printf("[%d] Remaining apples at (%d,%d): %4.2f\n", t, bins[b].loc.x, bins[b].loc.y, 
                    env.getApplesAt(bins[b].loc));
            }
            
            if (num > 0 && round(env.getApplesAt(bins[b].loc)) <= 0) { // No more apples at current location
                Coordinate tmp = distributeWorkers(workers, bins, bins[b].loc, env);
                if (tmp.x > 0 && tmp.x < ORCH_COLS - 1 && tmp.y >= 0 && tmp.y < ORCH_ROWS) {
                    registerLocation(tmp, requests, t);
                    printf("[%d] No more apples at (%d,%d). %d workers move to (%d,%d).\n", t, bins[b].loc.x, 
                        bins[b].loc.y, num, tmp.x, tmp.y);
                }
            } else if (num > 0 && env.getApplesAt(bins[b].loc) > 0 && round(bins[b].capacity) >= BIN_CAPACITY) {
                registerLocation(bins[b].loc, requests, t);
                printf("There are still %4.2f apples at (%d,%d).\n", env.getApplesAt(bins[b].loc), bins[b].loc.x, bins[b].loc.y);
            }
        }
        
        for (int n = 0; n < (int) requests.size(); ++n) {
            printf("[%d] New location request: (%d,%d). Remaining apples: %4.2f\n", t, requests[n].loc.x, 
                requests[n].loc.y, env.getApplesAt(requests[n].loc));
        }
        
        // Simulate agents
        for (int a = 0; a < NUM_AGENTS; ++a) {
            agents[a].takeAction(&binCounter, bins, repo, agents, env, requests);
            Coordinate atmp = agents[a].getCurLoc();
            fprintf(agentFiles[a], "%d,%d,%d\n", t, atmp.x, atmp.y);
        }
        
        for (int b = 0; b < (int) bins.size(); ++b)
            writeBinInfo("base", t, bins[b]);
        
        fprintf(repoFile, "%d,%d\n", t, (int) repo.size());
        
        int appleLocCount = 0;
        if (env.getTotalApples(&appleLocCount) == 0 && repo.size() >= 80) {
            printf("No more apples in orchard. Terminating simulation.\n");
            break;
        }
    }
    
    printf("------------ END OF SIMULATION ------------\n");
    printf("Total bins: %d\n", (int) repo.size());
}

void runAutonomous(const int NUM_AGENTS, const int NUM_LAYERS, const int TIME_LIMIT, bool learn)
{
    system("rm -rf logs/auto");
    system("mkdir logs/auto");
    system("mkdir -p logs/auto/agents");
    system("mkdir -p logs/auto/bins");
    
    // Prepare log files
    FILE *repoFile = fopen("logs/auto/repo.csv", "w");
    
    int binCounter = 0;
    std::vector<Worker> workers = initWorkers();
    std::vector<Coordinate> workerGroups = initWorkerGroupsFixed(workers);
    std::vector<AppleBin> bins = initBins(workerGroups, &binCounter);
    
    /* Initialize agents */
    std::vector<AutoAgent> agents;
    for (int i = 0; i < NUM_AGENTS; ++i) {
        agents.push_back(AutoAgent(i, Coordinate(0, 0), NUM_LAYERS, learn));
        char fname[50];
        sprintf(fname, "logs/auto/agents/agent%d.csv", i);
        FILE *fp = fopen(fname, "w");
        agentFiles.push_back(fp);
    }
    
    /* Initialize orchard environment with uniform distribution of apples */
    Orchard env;
    std::vector<AppleBin> repo;
    std::vector<LocationRequest> requests;
    
    /* Run simulator */
    for (int t = 0; t < TIME_LIMIT; ++t) {
        printf("------------ T = %d ------------\n", t);
        // Simulate bins and workers
        for (int b = 0; b < (int) bins.size(); ++b) {
            int num = getNumWorkersAt(workers, bins[b].loc);
            int tmp1 = round(bins[b].capacity);
            int tmp2 = BIN_CAPACITY;
            if (env.getApplesAt(bins[b].loc) > 0 && tmp1 < tmp2 && bins[b].onGround) {
                bins[b].fillRate = num * PICK_RATE;
                bins[b].capacity += bins[b].fillRate; // capacity increase for each time step = fill rate * 1
                env.decreaseApplesAt(bins[b].loc, bins[b].fillRate);
                if (bins[b].capacity >= BIN_CAPACITY) {
                    bins[b].capacity = BIN_CAPACITY;
                    bins[b].filledTime = t;
                }
            }
            
            const char *str = (bins[b].onGround) ? "on ground" : "carried";
            printf("[%d] B%d (%d,%d) %s, capacity: %4.2f. (# workers: %d)\n", t, bins[b].id, bins[b].loc.x, 
                bins[b].loc.y, str, bins[b].capacity, num);
            if (bins[b].onGround) {
                printf("[%d] Remaining apples at (%d,%d): %4.2f\n", t, bins[b].loc.x, bins[b].loc.y, 
                    env.getApplesAt(bins[b].loc));
            }
            
            if (num > 0 && round(env.getApplesAt(bins[b].loc)) <= 0) { // No more apples at current location
                Coordinate tmp = distributeWorkers(workers, bins, bins[b].loc, env);
                if (tmp.x > 0 && tmp.x < ORCH_COLS - 1 && tmp.y >= 0 && tmp.y < ORCH_ROWS) {
                    registerLocation(tmp, requests, t);
                    printf("[%d] No more apples at (%d,%d). %d workers move to (%d,%d).\n", t, bins[b].loc.x, 
                        bins[b].loc.y, num, tmp.x, tmp.y);
                }
            } else if (num > 0 && env.getApplesAt(bins[b].loc) > 0 && round(bins[b].capacity) >= BIN_CAPACITY) {
                registerLocation(bins[b].loc, requests, t);
            }
        }
        
        for (int n = 0; n < (int) requests.size(); ++n) {
            if (env.getApplesAt(requests[n].loc) == 0) {
                requests.erase(requests.begin() + n);
                --n;
            } else {
                printf("[%d] Location requests: (%d,%d). Remaining apples: %4.2f\n", t, requests[n].loc.x, requests[n].loc.y, 
                    env.getApplesAt(requests[n].loc));
            }
        }
        
        // Simulate agents
        for (int a = 0; a < NUM_AGENTS; ++a)
            agents[a].makePlans(agents, bins, env, workers); // Each agent create plans
        
        for (int a = 0; a < NUM_AGENTS; ++a) {
            agents[a].selectPlan(agents, bins); // Each agent selects a plan by negotiating conflict with other agents
            agents[a].takeAction(&binCounter, bins, requests, agents, repo, env, t);
        }
        
        for (int r = 0; r < (int) requests.size(); ++r) {
            if (isRequestFulfilled(requests[r].loc, bins)) {
                requests.erase(requests.begin() + r); // filter out fulfilled requests
                --r;
            }
        }
        
        for (int a = 0; a < NUM_AGENTS; ++a) {
            Coordinate atmp = agents[a].getCurLoc();
            fprintf(agentFiles[a], "%d,%d,%d\n", t, atmp.x, atmp.y);
        }
        
        for (int b = 0; b < (int) bins.size(); ++b)
            writeBinInfo("auto", t, bins[b]);
        
        fprintf(repoFile, "%d,%d\n", t, (int) repo.size());
        
        int appleLocCount = 0;
        if (env.getTotalApples(&appleLocCount) == 0 && repo.size() >= 80) {
            printf("No more apples in orchard. Terminating simulation.\n");
            break;
        }
    }
    
    printf("------------ END OF SIMULATION ------------\n");
    printf("Total bins: %d\n", (int) repo.size());
    int cellCount = 0;
    printf("Remaining apples in orchard: %4.2f in %d locations\n", env.getTotalApples(&cellCount), cellCount);
}

int main(int argc, char **argv)
{
    srand(time(NULL));
    
    int timeLimit = 10; // Default time limit
    int numAgents = DEFAULT_NUM_AGENTS;
    
    if (strcmp(argv[1], "-base") == 0) {
        for (int i = 2; i < argc; ++i) {
            if (argv[i][1] == 'a')
                numAgents = parseArgInt(argv[i]);
            else if (argv[i][1] == 't')
                timeLimit = parseArgInt(argv[i]);
        }
        printf("---------- Starting simulation with baseline algorithm ----------\n");
        runBase(numAgents, timeLimit);
    } else if (strcmp(argv[1], "-auto") == 0) {
        int numLayers = DEFAULT_NUM_LAYERS;
        bool learn = false;
        for (int i = 2; i < argc; ++i) {
            if (strcmp(argv[i], "-learn") == 0)
                learn = true;
            else if (argv[i][1] == 'a')
                numAgents = parseArgInt(argv[i]);
            else if (argv[i][1] == 'l')
                numLayers = parseArgInt(argv[i]);
            else if (argv[i][1] == 't')
                timeLimit = parseArgInt(argv[i]);
        }
        printf("---------- Starting simulation with autonomous agents ----------\n");
        if (learn)
            printf("Learning is used to select location request.\n");
        runAutonomous(numAgents, numLayers, timeLimit, learn);
    }
    
    return 0;
}



