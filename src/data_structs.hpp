#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

struct Coordinate
{
    int x;
    int y;
    Coordinate(int p = 0, int q = 0) : x(p), y(q) {}
};

struct LocationRequest
{
    Coordinate loc;
    int regisTime;
    LocationRequest(Coordinate l, int r = -1) : loc(l), regisTime(r) {}
};

struct AppleBin
{
    int id;
    float capacity;
    Coordinate loc;
    float fillRate;
    bool onGround;
    int filledTime;
    AppleBin(int i, int x, int y) : id(i), loc(x, y) { capacity = 0; onGround = false;  filledTime = -1; }
};

struct Worker
{
    int id;
    Coordinate loc;
    Worker(int i, int x, int y) : id(i), loc(x, y) {}
};

#endif // DATA_STRUCTS_HPP_
