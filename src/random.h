#pragma once

#include <random>
#include <optional>

extern std::random_device randDev;
extern std::mt19937 randEng;

void initRandom(std::optional<unsigned int> seed);

template<class T>
using RNGReal = std::uniform_real_distribution<T>;

template<class T> RNGReal<T> makeRNGReal(T min, T max)
{
	return std::uniform_real_distribution<T>(min, max);
}

template<class T> T fetchRandom(RNGReal<T> rng)
{
	return rng(randEng);
}

template<class T>
using RNGInt = std::uniform_int_distribution<T>;

template<class T> RNGInt<T> makeRNGInt(T min, T max)
{
	return std::uniform_int_distribution<T>(min, max);
}

template<class T> T fetchRandom(RNGInt<T> rng)
{
	return rng(randEng);
}
 