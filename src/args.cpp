#include "args.h"
#include <algorithm>
#include <sstream>
#include <cassert>

Arguments::Arguments(int argc, const char** argv)
{
	assert(argc > 0);
	assert(argv);

	for(int i = 0; i < argc; ++i)
		m_ArgSet.push_back(argv[i]);
}

size_t Arguments::count(bool inclPath) const
{
	const int pathShift = inclPath ? 0 : -1;
	return m_ArgSet.size() + pathShift;
}

std::optional<std::string> Arguments::get(std::string_view arg) const
{
	auto argIter = std::find(m_ArgSet.cbegin(), m_ArgSet.cend(), arg);
	if(argIter == m_ArgSet.end())
		return std::nullopt;
	else
	{
		++argIter;
		if(argIter == m_ArgSet.end())
			return std::nullopt;

		return *argIter;
	}
}

bool Arguments::getFlag(std::string_view flag) const
{
	return std::find(m_ArgSet.cbegin(), m_ArgSet.cend(), flag) 
		!= m_ArgSet.end();
}

std::optional<int> Arguments::getInt(std::string_view arg) const
{
	auto argIter = std::find(m_ArgSet.cbegin(), m_ArgSet.cend(), arg);
	if(argIter == m_ArgSet.end())
		return std::nullopt;
	else
	{
		++argIter;
		if(argIter == m_ArgSet.end())
			return std::nullopt;

		std::stringstream ss(*argIter);
		int outInt;
		ss >> outInt;
		return outInt;
	}
}

std::optional<float> Arguments::getFloat(std::string_view arg) const
{
	auto argIter = std::find(m_ArgSet.cbegin(), m_ArgSet.cend(), arg);
	if(argIter == m_ArgSet.end())
		return std::nullopt;
	else
	{
		++argIter;
		if(argIter == m_ArgSet.end())
			return std::nullopt;

		std::stringstream ss(*argIter);
		float outFloat;
		ss >> outFloat;
		return outFloat;
	}
}

 