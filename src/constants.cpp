#include <amra/constants.hpp>

const int AMRA::MAX_PLANNING_TIME_MS = 4000;
const int AMRA::COST_MULT = 1000;
const std::map<char, int> AMRA::MOVINGAI_DICT = {
	{'.', 1},
	{'G', 1},
	{'@', -1},
	{'O', -1},
	{'T', 0},
	{'S', 1},
	{'W', 2}, // water is only traversible from water
	{'(', 4}, // start
	{'*', 6}, // path
	{')', 8}, // goal
	{'E', 10}, // expanded state
};

const int AMRA::MIDRES_MULT = 3;
const int AMRA::LOWRES_MULT = 9;
const int AMRA::GRID = 8;
const int AMRA::NUM_RES = 3;

const bool AMRA::SAVE_ALL = false;
