#ifndef MOVINGAI_HPP
#define MOVINGAI_HPP

// project includes
#include <amra/types.hpp>

// system includes

// standard includes
#include <random>

namespace AMRA
{

class MovingAI
{
public:
	MovingAI(const std::string& fname);
	~MovingAI();

	void GetRandomState(int& d1, int& d2);
	void SaveMapAndPath(const std::vector<MapState>& solpath);

	bool IsValid(const int& dim1, const int& dim2) const;
	bool IsTraversible(const int& dim1, const int& dim2) const;

	int CellType(const int& dim1, const int& dim2) const;
	int CellType(const int& dim1, const int& dim2, char& c) const;

private:
	std::string m_fname;
	Map_t m_map;
	int m_h, m_w;

	std::random_device m_dev;
	std::mt19937 m_rng;
	std::uniform_real_distribution<double> m_distD;

	void readFile();
};

}  // namespace AMRA

#endif  // MOVINGAI_HPP
