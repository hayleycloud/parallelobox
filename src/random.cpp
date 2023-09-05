#include "random.h"
#include <random>

std::random_device randDev;
std::mt19937 randEng(randDev());

template<class T> RNG<T> makeRNG(T min, T max)
{
	return std::uniform_real_distribution<T>(min, max);
}

T fetchRandom(RNG<T> rng)
{
	return rng(randEng);
}

