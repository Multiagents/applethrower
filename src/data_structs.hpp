#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

struct Coordinate
{
    int x;
    int y;
    Coordinate() : x(0), y(0) {}
    Coordinate(int p, int q) : x(p), y(q) {}
};

struct AppleBin
{
    int id;
    float capacity;
    Coordinate loc;
    float fillRate;
    bool onGround;
    AppleBin(int i, int x, int y) : id(i), loc(x, y) { capacity = 0; onGround = false; }
};

struct Worker
{
    int id;
    Coordinate loc;
    Worker(int i, int x, int y) : id(i), loc(x, y) {}
};

#endif // DATA_STRUCTS_HPP_
