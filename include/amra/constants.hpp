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
extern const bool COSTMAP;

extern const double TURNING_RADIUS;
extern const double MAX_VEL;
extern const int WP_TIME;

extern const bool DUBINS;
extern const bool DIJKSTRA;

extern const bool SUCCESSIVE;

} // namespace SimPlan


#endif // CONSTANTS_HPP
