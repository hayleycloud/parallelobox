#pragma once

#include "args.h"
#include <string>

struct PrinterSpec
{
	struct Dims {
		double width, height, depth;
	} volume;

	double shellSpeed;
	double infillSpeed;

	double overhangTolerance;
};

struct Config 
{
	std::string inputFile;
	std::string outputDir;

	PrinterSpec printer;

	int numPrinters;

	int numBoxes;

	double infillRate;
};

Config handleArguments(const Arguments& args);

void printConfig(const Config& config);

