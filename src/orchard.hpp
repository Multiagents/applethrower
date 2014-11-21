#ifndef ORCHARD_HPP_
#define ORCHARD_HPP_

#include <cstdio>
#include <cstdlib>
#include <string>
#include "params.hpp"
#include "data_structs.hpp"

class Orchard
{
public:
    Orchard();
    
    ~Orchard();
    
    void print();
    
    std::string toString();
    
    float getApplesAt(Coordinate loc);
    
    void decreaseApplesAt(Coordinate loc, float fillRate);
    
    double getTotalApples(int *count);
    
    float getEstApplesRemaining(Coordinate loc, float estTime, float fillRate);

private:
    float appleDist[ORCH_ROWS][ORCH_COLS];
};

#endif // ORCHARD_HPP_
