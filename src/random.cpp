#include "random.h"

std::random_device randDev;
std::mt19937 randEng/*(randDev())*/;

void initRandom(std::optional<unsigned int> seed)
{
	if(seed)
		randEng.seed(*seed);
	else
		randEng = std::mt19937(randDev());
}

 