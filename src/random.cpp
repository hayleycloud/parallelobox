#include "random.h"

std::random_device randDev;
std::mt19937 randEng(randDev());

