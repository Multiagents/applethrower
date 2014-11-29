#ifndef PARAMS_HPP_
#define PARAMS_HPP_

const int ORCH_ROWS          = 5;
const int ORCH_COLS          = 10;

const int DEFAULT_NUM_AGENTS = 2;
const int NUM_WORKERS        = 10;
const int MAX_BIN_PER_WORKER = 100; // The maximum number of bins that one worker can fill in one day
const int NUM_APPLES_PER_LOC = 20;

const float BIN_CAPACITY     = 10.0f; // One bin can contain at most 100 apples
const float PICK_RATE        = 1;//BIN_CAPACITY / 60.0f; // In apples per time step; one worker can fill one bin in 1 hour

const float AGENT_SPEED_H    = 2; // Agent speed when not carrying a bin: 9 grids per time step
const float AGENT_SPEED_L    = 1; // Agent speed when carrying a bin: 6 grids per time step

const int DEFAULT_NUM_LAYERS = 3;

#endif // PARAMS_HPP_
