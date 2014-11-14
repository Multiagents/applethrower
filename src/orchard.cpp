#include <cstdio>
#include <cstdlib>
#include "params.hpp"
#include "orchard.hpp"

Orchard::Orchard()
{
    /* Initialize numbers of apples in each grid (uniform distribution). */
    for (int i = 0; i < ORCH_ROWS; ++i)
        for (int j = 0; j < ORCH_COLS; ++j)
            appleDist[i][j] = (50 * BIN_CAPACITY) / ORCH_COLS;
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
    appleDist[loc.y][loc.x] -= fillRate;
}

float Orchard::getTotalApples(int *count)
{
    (*count) = 0;
    float total = 0;
    for (int i = 0; i < ORCH_ROWS; ++i) {
        for (int j = 0; j < ORCH_COLS; ++j) {
            total += appleDist[i][j];
            if (appleDist[i][j] > 0)
                (*count)++;
        }
    }
    return total;
}

/*void Orchard::print()
{
    for (int i = 0; i < ORCH_ROWS; ++i) {
        for (int j = 0; j < ORCH_COLS; ++j) {
            if (i == ORCH_COLS - 1)
                printf("%4.2f\n", appleDist[i][j]);
            else
                printf("%4.2f,", appleDist[i][j]);
        }
    }
}

std::string Orchard::toString()
{
    std::string str;
    for (int i = 0; i < ORCH_ROWS; ++i) {
        for (int j = 0; j < ORCH_COLS; ++j) {
            char buf[20];
            if (i == ORCH_COLS - 1)
                sprintf(buf, "%4.2f\n", appleDist[i][j]);
            else
                sprintf(buf, "%4.2f,", appleDist[i][j]);
            str += buf;
        }
    }
    return str;
}*/



