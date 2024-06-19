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

	int sampleTries;

	double granularityScale;

	double infillRate;

	double proximityFactor;

	std::optional<unsigned int> seed;

	bool symmetrySkip;
	bool cleanupOutDirAfter;
};

Config handleArguments(const Arguments& args);

void printConfig(const Config& config);

