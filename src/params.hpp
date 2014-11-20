#ifndef PARAMS_HPP_
#define PARAMS_HPP_

const int ORCH_ROWS          = 10;
const int ORCH_COLS          = 1000;

const int NUM_AGENTS         = 8;
const int NUM_WORKERS        = 20;
const int MAX_BIN_PER_WORKER = 10; // The maximum number of bins that one worker can fill in one day
const int NUM_APPLES_PER_LOC = 200;

const float BIN_CAPACITY     = 100; // One bin can contain at most 100 apples
const float PICK_RATE        = BIN_CAPACITY / 60.0f; // In apples per time step; one worker can fill one bin in 1 hour

const float AGENT_SPEED_H    = 9; // Agent speed when not carrying a bin: 9 grids per time step
const float AGENT_SPEED_L    = 6; // Agent speed when carrying a bin: 6 grids per time step

#endif // PARAMS_HPP_
