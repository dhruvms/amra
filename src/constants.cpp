#include <amra/constants.hpp>

const int AMRA::MAX_PLANNING_TIME_MS = 30000;
const int AMRA::COST_MULT = 1000;
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

const int AMRA::MIDRES_MULT = 7;
const int AMRA::LOWRES_MULT = 21;
const int AMRA::GRID = 4;
const int AMRA::NUM_RES = 3;

const bool AMRA::SAVE_ALL = false;
const bool AMRA::COSTMAP = false;

const int AMRA::TURN_PENALTY = 1;
const double AMRA::TURNING_RADIUS = 20.0;
const double AMRA::MAX_VEL = 8.0;

const bool AMRA::DUBINS = false;
const bool AMRA::DIJKSTRA = false;

const bool AMRA::SUCCESSIVE = false;
