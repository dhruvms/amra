#include <amra/constants.hpp>

const int AMRA::MAX_PLANNING_TIME_MS = 30000;
const int AMRA::COST_MULT = 100;
const std::map<char, int> AMRA::MOVINGAI_DICT = {
	{'.', 1},
	{'G', 1},
	{'@', -1},
	{'O', -1},
	{'T', 0},
	{'S', 1},
	{'W', 2}, // water is only traversible from water
	{'(', 1000}, // start
	{'*', 1001}, // path
	{')', 1002}, // goal
	{'E', 1003}, // expanded state
};

// discretisation resolutions for mid- and low-level grids
// high-level grids assume a 1x1 discretisation
const int AMRA::MIDRES_MULT = 3;
const int AMRA::LOWRES_MULT = 9;
// number of resolutions to use in the search (>= 1, <= 3)
const int AMRA::NUM_RES = 2;
// 4-connected or 8-connected grid
const int AMRA::GRID = 4;
// set true if using maps with non-uniform cell costs
const bool AMRA::COSTMAP = false;

// UAV experiment parameters
const double AMRA::TURNING_RADIUS = 20.0;
const double AMRA::MAX_VEL = 8.0;
const int AMRA::WP_TIME = 50; // milliseconds

// use dubins or dijkstra heuristics?
const bool AMRA::DUBINS = false;
const bool AMRA::DIJKSTRA = false;

// run successive search iterations from scratch?
// (with no reuse of previous search effort)
const bool AMRA::SUCCESSIVE = false;
