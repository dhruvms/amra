// project includes
#include <amra/movingai.hpp>
#include <amra/helpers.hpp>
#include <amra/constants.hpp>

// system includes

// standard includes
#include <fstream>
#include <cassert>
#include <cstring>

namespace AMRA
{

MovingAI::MovingAI(const std::string& fname)
:
m_fname(fname),
m_rng(m_dev())
{
	readFile();
	m_distD = std::uniform_real_distribution<double>(0.0, 1.0);
};

MovingAI::~MovingAI()
{
	free(m_map);
}

void MovingAI::GetRandomState(int& d1, int& d2)
{
	d1 = (int)std::round(m_distD(m_rng) * (m_h - 1));
	d2 = (int)std::round(m_distD(m_rng) * (m_w - 1));
	while (!IsTraversible(d1, d2))
	{
		d1 = (int)std::round(m_distD(m_rng) * (m_h - 1));
		d2 = (int)std::round(m_distD(m_rng) * (m_w - 1));
	}
}


void MovingAI::SaveMapAndPath(const std::vector<MapState>& solpath)
{
	Map_t outmap;
	outmap = (Map_t)calloc(m_h * m_w, sizeof(decltype(*outmap)));
	std::memcpy(outmap, m_map, m_h * m_w * sizeof(decltype(*outmap)));
	for (const auto& state: solpath) {
		outmap[GETMAPINDEX(state.d1, state.d2, m_h, m_w)] = MOVINGAI_DICT.find('*')->second;
	}
	outmap[GETMAPINDEX(solpath.at(0).d1, solpath.at(0).d2, m_h, m_w)] = MOVINGAI_DICT.find('(')->second;
	outmap[GETMAPINDEX(solpath.back().d1, solpath.back().d2, m_h, m_w)] = MOVINGAI_DICT.find(')')->second;

	std::string outfile_map(m_fname), outfile_path(m_fname);
	outfile_map.insert(outfile_map.find_last_of('.'), "_");
	outfile_map.insert(outfile_map.find_last_of('.'), "solution");
	outfile_path.insert(outfile_path.find_last_of('.'), "_");
	outfile_path.insert(outfile_path.find_last_of('.'), "path");

	std::ofstream OUT_MAP, OUT_PATH;
	OUT_MAP.open(outfile_map, std::ofstream::out);

	for (int r = 0; r < m_h; ++r)
	{
		for (int c = 0; c < m_w; ++c)
		{
			OUT_MAP << outmap[GETMAPINDEX(r, c, m_h, m_w)];

			if (c < m_w - 1) {
				OUT_MAP << ',';
			}
		}

		if (r < m_h - 1) {
			OUT_MAP << '\n';
		}
	}

	OUT_MAP.close();
	free(outmap);

	OUT_PATH.open(outfile_path, std::ofstream::out);
	for (const auto& s: solpath) {
		OUT_PATH << s;
	}
	OUT_PATH.close();
}

bool MovingAI::IsValid(const int& dim1, const int& dim2) const
{
	return (dim1 >= 0 && dim1 < m_h) && (dim2 >= 0 && dim2 < m_w);
}

bool MovingAI::IsTraversible(const int& dim1, const int& dim2) const
{
	if (!IsValid(dim1, dim2)) {
		return false;
	}
	return m_map[GETMAPINDEX(dim1, dim2, m_h, m_w)] > 0;
}

int MovingAI::CellType(const int& dim1, const int& dim2) const
{
	if (!IsValid(dim1, dim2)) {
		return -99;
	}
	return m_map[GETMAPINDEX(dim1, dim2, m_h, m_w)];
}

int MovingAI::CellType(const int& dim1, const int& dim2, char& c) const
{
	if (!IsValid(dim1, dim2)) {
		c = '!';
		return -99;
	}

	int val = m_map[GETMAPINDEX(dim1, dim2, m_h, m_w)];
	for (auto itr = MOVINGAI_DICT.begin(); itr != MOVINGAI_DICT.end(); ++itr)
	{
		if (itr->second == val) {
			c = itr->first;
		}
	}

	return val;
}

void MovingAI::readFile()
{
	std::ifstream FILE(m_fname);
	std::string line, word, temp;
	std::stringstream ss;

	std::getline(FILE, line);
	assert(line.compare("type octile") == 0);

	// read height/width
	std::getline(FILE, line);
	reset(ss);
	ss.str(line);
	std::getline(ss, word, ' ');
	if (word.compare("height") == 0)
	{
		std::getline(ss, word, ' ');
		m_h = std::stoi(word);
	}
	else if (word.compare("width") == 0)
	{
		std::getline(ss, word, ' ');
		m_w = std::stoi(word);
	}

	// read width/height
	std::getline(FILE, line);
	reset(ss);
	ss.str(line);
	std::getline(ss, word, ' ');
	if (word.compare("height") == 0)
	{
		std::getline(ss, word, ' ');
		m_h = std::stoi(word);
	}
	else if (word.compare("width") == 0)
	{
		std::getline(ss, word, ' ');
		m_w = std::stoi(word);
	}

	std::getline(FILE, line);
	assert(line.compare("map") == 0);

	m_map = (Map_t)calloc(m_h * m_w, sizeof(decltype(*m_map)));
	for (int r = 0; r < m_h; ++r)
	{
		std::getline(FILE, line);
		for (int c = 0; c < m_w; ++c)
		{
			m_map[GETMAPINDEX(r, c, m_h, m_w)] = MOVINGAI_DICT.find(line[c])->second;
		}
	}
}

}  // namespace CMUPlanner
