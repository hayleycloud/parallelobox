#pragma once

#include "args.h"
#include <string>

struct PrinterSpec
{
	struct Dims {
		double width, height, depth;
	} volume;

	struct Speeds {
		double innerWall;
		double outerWall;
		double infill;
		double support;
	} speeds;

	double nozzleSize;

	double overhangTolerance;
};

struct Config 
{
	std::string inputFile;
	std::string outputDir;
	std::string printerFile;

	PrinterSpec printer;

	int numPrinters;

	int sampleTries;

	double granularityScale;

	double layerHeight;
	int wallCount;
	double wallThickness;

	double infillDensity;
	double supportDensity;

	double proximityFactor;

	std::optional<unsigned long> seed;

	bool symmetrySkip;
	bool relaxSafeties;
	bool failOnInsufficientPrinters;
	bool cleanupOutDirAfter;
};

Config handleArguments(const Arguments& args);

void printConfig(const Config& config);

 
