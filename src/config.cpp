#include "config.h"
#include <string_view>
#include <cstring>
#include <stdexcept>

void validateInputFile(std::string_view inputFile)
{
	if(inputFile.length() == 0)
		throw std::runtime_error("--in required to be a mesh file!");

	if(!inputFile.ends_with(".stl") &&
	   !inputFile.ends_with(".off") &&
	   !inputFile.ends_with(".ply") &&
	   !inputFile.ends_with(".obj"))
	{
		throw std::runtime_error("unsupported mesh file for --in");
	}
}

void validateOutputDir(std::string_view outputDir)
{
	if(outputDir.length() == 0)
		throw std::runtime_error("--out required to be a directory name!");
}

Config handleArguments(int argc, char* argv[]) 
{
	Config config;

	for(int i = 0; i < argc; ++i)
	{
		if(!strcmp(argv[i], "--in"))
		{
			config.inputFile = argv[++i];
		}
		else if(!strcmp(argv[i], "--out"))
		{
			config.outputDir = argv[++i];
		}
	}

	validateInputFile(config.inputFile);
	validateOutputDir(config.outputDir);

	return config;
}
