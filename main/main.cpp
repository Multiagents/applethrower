#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <ctime>
#include <vector>
#include "data_structs.hpp"
#include "params.hpp"
#include "orchard.hpp"
#include "agent.hpp"

const int TIME_LIMIT = 500;

FILE *logFile;
FILE *repoFile;
std::vector<FILE*> agentFiles;

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
    for (int c = curLoc.x + 1; c < ORCH_COLS; ++c) {
        Coordinate tmp(c, curLoc.y);
        if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
            return tmp;
    }
    
    // Search location at the same row, to the left columns
    for (int c = curLoc.x - 1; c >= 0; --c) {
        Coordinate tmp(c, curLoc.y);
        if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
            return tmp;
    }
    
    // Search location at a different row (down)
    for (int r = curLoc.y + 1; r < ORCH_ROWS; ++r) {
        Coordinate tmp(ORCH_COLS - 1, r); // Starts from the rightmost column
        if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
            return tmp;
    }
    
    // Search location at a different row (up)
    for (int r = curLoc.y - 1; r >= 0; --r) {
        Coordinate tmp(ORCH_COLS - 1, r); // Starts from the rightmost column
        if (env.getApplesAt(tmp) > 0 && getNumWorkersAt(workers, tmp) == 0)
            return tmp;
    }
    
    // Still can't find a good location, join another group with the max ratio between apples and workers
    float maxRatio = 0;
    Coordinate maxLoc(-1, -1);
    for (int r = 0; r < ORCH_ROWS; ++r) {
        for (int c = 0; c < ORCH_COLS; c++) {
            Coordinate tmp(r, c);
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

void writeBinInfo(int time, AppleBin ab)
{
    char fname[50];
    sprintf(fname, "logs/bins/bin%d.csv", ab.id);
    FILE *fp = fopen(fname, "a");
    fprintf(fp, "%d,%d,%d,%4.2f\n", time, ab.loc.x, ab.loc.y, ab.capacity);
    fclose(fp);
}

void run()
{
    system("rm -r logs");
    system("mkdir logs");
    system("mkdir -p logs/agents");
    system("mkdir -p logs/bins");
    
    // Prepare log files
    FILE *repoFile = fopen("logs/repo.csv", "w");
    
    int binCounter = 0;
    
    /* Initialize individual workers */
    std::vector<Worker> workers;
    for (int i = 0; i < NUM_WORKERS; ++i)
        workers.push_back(Worker(i, 0, 0));
    
    /* Initialize groups of workers */
    std::vector<Coordinate> binLocations;
    int count = 0;
    while (count < NUM_WORKERS) {
        // Get random coordinate
        int x = rand() % (ORCH_COLS - 1) + 1;
        int y = rand() % ORCH_ROWS;
        // Get random number of workers for a group
        int num = rand() % 5 + 1;
        // Register workers' locations
        for (int n = 0; n < num && count < NUM_WORKERS; ++n) {
            workers[count].loc.x = x;
            workers[count].loc.y = y;
            ++count;
        }
        binLocations.push_back(Coordinate(x, y));
    }
    
    /* Initialize bins with the locations of groups of people */
    printf("Initial bin locations:\n");
    std::vector<AppleBin> bins;
    for (int i = 0; i < (int) binLocations.size(); ++i) {
        bins.push_back(AppleBin(binCounter++, binLocations[i].x, binLocations[i].y));
        writeBinInfo(-1, bins[i]);
        printf("B%d at (%d,%d)\n", bins[i].id, bins[i].loc.x, bins[i].loc.y);
    }
    printf("----------\n");
    
    /* Initialize agents */
    std::vector<Agent> agents;
    for (int i = 0; i < NUM_AGENTS; ++i) {
        agents.push_back(Agent(i, Coordinate(0, 0)));
        char fname[50];
        sprintf(fname, "logs/agents/agent%d.csv", i);
        FILE *fp = fopen(fname, "w");
        agentFiles.push_back(fp);
    }
    
    /* Initialize orchard environment with uniform distribution of apples */
    Orchard env;
    std::vector<AppleBin> repo;
    
    /* Run simulator */
    std::vector<Coordinate> newLocs;
    for (int t = 0; t < TIME_LIMIT; ++t) {
        // Simulate bins and workers
        Coordinate newLoc(-1, -1);
        for (int b = 0; b < (int) bins.size(); ++b) {
            int num = getNumWorkersAt(workers, bins[b].loc);
            bins[b].fillRate = num * PICK_RATE;
            if (env.getApplesAt(bins[b].loc) > 0) {
                bins[b].capacity += bins[b].fillRate; // capacity increase for each time step = fill rate * 1
                env.decreaseApplesAt(bins[b].loc, bins[b].fillRate);
                if (bins[b].capacity > BIN_CAPACITY)
                    bins[b].capacity = BIN_CAPACITY;
                printf("[%d] B%d (%d,%d) capacity: %4.2f\n", t, bins[b].id, bins[b].loc.x, bins[b].loc.y, bins[b].capacity);
            } else { // No more apples at current location
                Coordinate tmp = distributeWorkers(workers, bins, bins[b].loc, env);
                newLocs.push_back(tmp);
            }
        }
        
        // Simulate agents
        for (int a = 0; a < NUM_AGENTS; ++a) {
            agents[a].takeAction(&binCounter, bins, repo, agents, env, newLocs);
            Coordinate atmp = agents[a].getCurLoc();
            fprintf(agentFiles[a], "%d,%d,%d\n", t, atmp.x, atmp.y);
        }
        
        for (int b = 0; b < (int) bins.size(); ++b)
            writeBinInfo(t, bins[b]);
        
        fprintf(repoFile, "%d,%d\n", t, (int) repo.size());
        int locCount = 0;
        float totalApples = env.getTotalApples(&locCount);
        printf("Remaining apples: %4.2f at %d locations.\n", totalApples, locCount);
    }
    
    fclose(repoFile);
    for (int a = 0; a < (int) agentFiles.size(); ++a)
        fclose(agentFiles[a]);
}

int main(int argc, char **argv)
{
    srand(time(NULL));
    
    run();
    
    return 0;
}
