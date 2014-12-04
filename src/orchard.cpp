#include <cstdio>
#include <cstdlib>
#include "params.hpp"
#include "orchard.hpp"

Orchard::Orchard()
{
    /* Initialize numbers of apples in each grid (uniform distribution). */
    for (int i = 0; i < ORCH_ROWS; ++i) {
        for (int j = 0; j < ORCH_COLS; ++j) {
            if (j == 0 || j == ORCH_COLS - 1)
                appleDist[i][j] = 0;
            else
                appleDist[i][j] = NUM_APPLES_PER_LOC;
        }
    }
}

Orchard::~Orchard()
{
    // TODO
}

float Orchard::getApplesAt(Coordinate loc)
{
    if (loc.x >= 0 && loc.x < ORCH_COLS && loc.y >= 0 && loc.y < ORCH_ROWS)
        return appleDist[loc.y][loc.x];
    return 0;
}

void Orchard::decreaseApplesAt(Coordinate loc, float fillRate)
{
    if (loc.x < 0 || loc.x >= ORCH_COLS || loc.y < 0 || loc.y >= ORCH_ROWS)
        return;
    appleDist[loc.y][loc.x] = (fillRate >= appleDist[loc.y][loc.x]) ? 0 : appleDist[loc.y][loc.x] - fillRate;
    /*if (appleDist[loc.y][loc.x] <= BIN_CAPACITY) {
        appleDist[loc.y][loc.x] = (fillRate >= appleDist[loc.y][loc.x]) ? 0 : appleDist[loc.y][loc.x] - fillRate;
    } else {
        appleDist[loc.y][loc.x] = (fillRate >= BIN_CAPACITY) ? appleDist[loc.y][loc.x] - BIN_CAPACITY : appleDist[loc.y][loc.x] - fillRate;
    }*/

}

double Orchard::getTotalApples(int *count)
{
    (*count) = 0;
    double total = 0;
    for (int i = 0; i < ORCH_ROWS; ++i) {
        for (int j = 0; j < ORCH_COLS; ++j) {
            total += appleDist[i][j];
            if (appleDist[i][j] > 0)
                (*count)++;
        }
    }
    return total;
}

float Orchard::getEstApplesRemaining(Coordinate loc, float estTime, float fillRate)
{
    float amount = appleDist[loc.y][loc.x] - (estTime * fillRate);
    amount = (amount >= 0) ? amount : 0;
    return amount;
}



