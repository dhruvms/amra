#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// standard includes
#include <map>

namespace AMRA
{

extern const int MAX_PLANNING_TIME_MS;
extern const int COST_MULT;
extern const std::map<char, int> MOVINGAI_DICT;

extern const int MIDRES_MULT;
extern const int LOWRES_MULT;
extern const int GRID;
extern const int NUM_RES;

extern const bool SAVE_ALL;

extern const int TURN_PENALTY;

} // namespace SimPlan


#endif // CONSTANTS_HPP
