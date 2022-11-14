#pragma once

#include <string>

struct Config 
{
	std::string inputFile;
	std::string outputDir;
};

Config handleArguments(int argc, char* argv[]);

