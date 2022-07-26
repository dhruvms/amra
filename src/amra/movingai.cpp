// project includes
#include <amra/movingai.hpp>
#include <amra/helpers.hpp>
#include <amra/constants.hpp>

// system includes

// standard includes
#include <fstream>
#include <cassert>
#include <cstring>
#include <iomanip>

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
	while (true)
	{
		d1 = (int)std::round(m_distD(m_rng) * (m_h - 1));
		d2 = (int)std::round(m_distD(m_rng) * (m_w - 1));

		if (NUM_RES == 2)
		{
			if ((d1 % MIDRES_MULT != 0 || d2 % MIDRES_MULT != 0)) {
				continue;
			}
		}
		if (NUM_RES == 3)
		{
			if ((d1 % LOWRES_MULT != 0 || d2 % LOWRES_MULT != 0)) {
				continue;
			}
		}

		if (IsTraversible(d1, d2)) {
			break;
		}
	}
}

void MovingAI::SavePath(
	const std::vector<MapState>& solpath,
	int iter,
	const std::vector<int>& expand_ts,
	const std::vector<int>& f_vals)
{
	std::string filename(__FILE__);
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/solutions/";

	found = m_fname.find_last_of("/\\");
	filename += m_fname.substr(found + 1);

	std::string pathfile(filename), exp_ts_file(filename), f_vals_file(filename);
	pathfile.insert(pathfile.find_last_of('.'), "_");
	pathfile.insert(pathfile.find_last_of('.'), "path");
	exp_ts_file.insert(exp_ts_file.find_last_of('.'), "_");
	exp_ts_file.insert(exp_ts_file.find_last_of('.'), "texpands");
	f_vals_file.insert(f_vals_file.find_last_of('.'), "_");
	f_vals_file.insert(f_vals_file.find_last_of('.'), "fvals");

	if (iter >= 0)
	{
		std::stringstream ss;
		ss << std::setw(4) << std::setfill('0') << iter << '_';
		std::string s = ss.str();

		pathfile.insert(pathfile.find_last_of('/')+1, s);

		reset(ss);
	}

	std::ofstream OUT_PATH, OUT_TEXPANDS, OUT_FVALS;
	OUT_PATH.open(pathfile, std::ofstream::out);
	for (const auto& s: solpath) {
		OUT_PATH << s;
	}
	OUT_PATH.close();

	OUT_TEXPANDS.open(exp_ts_file, std::ofstream::out);
	for (size_t i = 0; i < expand_ts.size(); ++i)
	{
		OUT_TEXPANDS << expand_ts[i];
		if (i < expand_ts.size() - 1) {
			OUT_TEXPANDS << '\n';
		}
	}
	OUT_TEXPANDS.close();

	OUT_FVALS.open(f_vals_file, std::ofstream::out);
	for (size_t i = 0; i < f_vals.size(); ++i)
	{
		OUT_FVALS << f_vals[i];
		if (i < f_vals.size() - 1) {
			OUT_FVALS << '\n';
		}
	}
	OUT_FVALS.close();
}

void MovingAI::SaveExpansions(
	int iter, double w1, double w2,
	const EXPANDS_t& expansions)
{
	std::string filename(__FILE__), expfile;
	auto found = filename.find_last_of("/\\");
	filename = filename.substr(0, found + 1) + "../../dat/expansions/";

	std::stringstream ss;
	ss << std::setw(4) << std::setfill('0') << iter << '_';
	ss << w1 << '_';
	ss << w2;
	std::string s = ss.str();

	filename += s;
	reset(ss);

	MAP_t expmap;
	expmap = (MAP_t)calloc(m_h * m_w, sizeof(decltype(*expmap)));
	for (const auto& q: expansions)
	{
		std::memcpy(expmap, m_map, m_h * m_w * sizeof(decltype(*expmap)));
		for (const auto& s: q.second) {
			expmap[GETMAPINDEX(s->coord.at(0), s->coord.at(1), m_h, m_w)] = MOVINGAI_DICT.find('E')->second;
			if (COSTMAP) {
				expmap[GETMAPINDEX(s->coord.at(0), s->coord.at(1), m_h, m_w)] *= 10;
			}
		}

		expfile = filename;
		if (q.first >= 0)
		{
			ss << std::setw(4) << std::setfill('0') << q.first << '_';
			found = expfile.find_last_of("/\\");
			expfile.insert(found+1+4+1, ss.str());
			reset(ss);
		}

		std::ofstream EXP_MAP;
		EXP_MAP.open(expfile, std::ofstream::out);
		for (int r = 0; r < m_h; ++r)
		{
			for (int c = 0; c < m_w; ++c)
			{
				EXP_MAP << expmap[GETMAPINDEX(r, c, m_h, m_w)];

				if (c < m_w - 1) {
					EXP_MAP << ',';
				}
			}

			if (r < m_h - 1) {
				EXP_MAP << '\n';
			}
		}

		EXP_MAP.close();
	}

	free(expmap);
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
	c = '!';
	if (!IsValid(dim1, dim2)) {
		return -99;
	}

	int val = m_map[GETMAPINDEX(dim1, dim2, m_h, m_w)];
	for (auto itr = MOVINGAI_DICT.begin(); itr != MOVINGAI_DICT.end(); ++itr)
	{
		if (itr->second == val) {
			c = itr->first;
		}
	}
	if (c == '!') {
		c = char(val);
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

	m_map = (MAP_t)calloc(m_h * m_w, sizeof(decltype(*m_map)));
	for (int r = 0; r < m_h; ++r)
	{
		std::getline(FILE, line);
		for (int c = 0; c < m_w; ++c)
		{
			auto itr = MOVINGAI_DICT.find(line[c]);
			if (itr != MOVINGAI_DICT.end()) {
				m_map[GETMAPINDEX(r, c, m_h, m_w)] = MOVINGAI_DICT.find(line[c])->second;
			}
			else {
				int val = int(line[c]) - int('a') + 1;
				m_map[GETMAPINDEX(r, c, m_h, m_w)] = val * 10;
			}
		}
	}
}

}  // namespace AMRA
