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
    
    float getTotalApples(int *count);

private:
    float appleDist[ORCH_ROWS][ORCH_COLS];
};

#endif // ORCHARD_HPP_
