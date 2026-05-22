#include "random.h"

std::random_device randDev;
std::mt19937 randEng;

Seed initRandom(std::optional<Seed> seed)
{
	Seed actualSeed = seed ? *seed : randDev();
	randEng.seed(actualSeed);
	return actualSeed;
}

 
