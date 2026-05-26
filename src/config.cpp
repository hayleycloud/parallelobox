#include "config.h"
#include "args.h"
#include "defaults.h"
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstring>
#include <stdexcept>
#include <iostream>


struct INIFile
{
	typedef std::unordered_map<std::string,std::string> Section;

	std::unordered_map<std::string,Section> sections;
};

void trim(std::string& str)
{
	str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());
}

INIFile loadINIFile(std::string_view filename)
{
	INIFile iniFile;

	std::ifstream f(static_cast<std::string>(filename));
	
	std::string section = "";
	std::string line;
	while(std::getline(f, line))
	{
		trim(line);

		if(line.starts_with(';'))
			continue;
		else if(line.starts_with('[') && line.ends_with(']'))
			section = line.substr(1, line.rfind(']') - 1);
		else if(line.length() > 0)
		{
			size_t assignIndex = line.find('=');
			if(assignIndex == std::string::npos)
				continue;

			std::string key = line.substr(0, assignIndex);
			std::string data = line.substr(assignIndex + 1);
			trim(key);
			trim(data);

			//std::cout << "[" << section << "] " << key << " = " << data << std::endl;

			iniFile.sections[section][key] = data;
		}
	}

	return iniFile;
}

PrinterSpec makePrinterSpec(const INIFile& iniFile)
{
	PrinterSpec spec;

	try {
		const INIFile::Section& dims = iniFile.sections.at("dimensions");
		const std::string& width = dims.at("width");
		const std::string& height = dims.at("height");
		const std::string& depth = dims.at("depth");

		std::stringstream ss1("");
		ss1 << width;
		ss1 >> spec.volume.width;

		std::stringstream ss2("");
		ss2 << height;
		ss2 >> spec.volume.height;

		std::stringstream ss3("");
		ss3 << depth;
		ss3 >> spec.volume.depth;
	} catch(std::exception ex) {
		throw std::runtime_error("Error loading printer config: \"dimensions\"");
	}

	try {
		const INIFile::Section& speed = iniFile.sections.at("speed");
		const std::string& innerWall = speed.at("inner_wall");
		const std::string& outerWall = speed.at("outer_wall");
		const std::string& infill = speed.at("infill");
		const std::string& support = speed.at("support");

		std::stringstream ss1("");
		ss1 << innerWall;
		ss1 >> spec.speeds.innerWall;

		std::stringstream ss2("");
		ss2 << outerWall;
		ss2 >> spec.speeds.outerWall;

		std::stringstream ss3("");
		ss3 << infill;
		ss3 >> spec.speeds.infill;

		std::stringstream ss4("");
		ss4 << support;
		ss4 >> spec.speeds.support;
	} catch(std::exception ex) {
		throw std::runtime_error("Error loading printer config: \"speeds\"");
	}

	try {
		const INIFile::Section& other = iniFile.sections.at("other");
		const std::string& nozzleSize = other.at("nozzle_size");
		const std::string& overhangTolerance = other.at("overhang_tolerance");

		std::stringstream ss("");
		ss << nozzleSize;
		ss >> spec.nozzleSize;

		std::stringstream ss1("");
		ss1 << overhangTolerance;
		ss1 >> spec.overhangTolerance;
	} catch(std::exception ex) {
		throw std::runtime_error("Error loading printer config: \"other\"");
	}

	return spec;
}

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

void validateNumPrinters(int numPrinters)
{
	if(numPrinters <= 0)
		throw std::runtime_error("--num required to be a positive integer!");
}

void printUsage() {
	std::cout << "Usage: parallelobox [OPTIONS]" << std::endl;
	std::cout << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "  --in <file>             Path to an input model to process." << std::endl;
	std::cout << "  --out <directory>       The directory to store the partitioned models." << std::endl;
	std::cout << "                          Uses model name if not specified" << std::endl;
	std::cout << "  --num <printers>        Number of printers available." << std::endl;
	std::cout << "" << std::endl;
	std::cout << "  --granularity <mode>    Degree of initial voxelisation." << std::endl;
	std::cout << "                          [coarse, regular (default), fine, vfine]." << std::endl;
	std::cout << "  --sample-tries <num>    Number of times to try km++ seeding." << std::endl;
	std::cout << "" << std::endl;
	std::cout << "  --printer <file>        Load printer settings from <file>." << std::endl;
	std::cout << "                          [default: \"printers/default.ini\"]." << std::endl;
	std::cout << "  --quality <height>      Print quality (layer height in mm)." << std::endl;
	std::cout << "  --walls <num>           Number of walls (default: 3)." << std::endl;
	std::cout << "  --infill <rate>         Infill density (1.0 = 100%, 0.5 = 50%)." << std::endl;
	std::cout << "  --support <density>     Support density (1.0 = 100%, 0.5 = 50%)." << std::endl;
	std::cout << "" << std::endl;
	std::cout << "  --clean-out             Remove intermediates from output dir." << std::endl;
	std::cout << "  --relax-safeties        Skip the more aggressive mesh validation checks." << std::endl;
	std::cout << "  --no-min-printers       Continue even if there are probably not enough printers available." << std::endl;
	std::cout << "  --skip-symmetry         Skip the initial symmetry partition." << std::endl;
	std::cout << "  --help                  Print this usage documentation." << std::endl;
	std::cout << std::endl;
	std::cout << "Options --in and --num are mandatory." << std::endl;
	std::cout << std::endl;
	std::cout << "For bug reporting and enquiries, please see:" << std::endl;
	std::cout << "<http://www.hayleyhatton.co.uk/contact.htm>" << std::endl;
}

[[nodiscard]] double scaleFromCubeRoot(double root)
{
	return 1.0 / (root * root * root);
}

[[nodiscard]] double granularityScaleFrom(const std::string& granularityStr)
{
	const std::unordered_map<std::string,double> types{
		{ "ucoarse", scaleFromCubeRoot(2.0) },
		{ "xcoarse", scaleFromCubeRoot(3.0) },

		{ "vcoarse",  scaleFromCubeRoot(8.0) },
		{ "coarse", scaleFromCubeRoot(9.0) },
		{ "regular", scaleFromCubeRoot(10.0) },
		{ "fine",    scaleFromCubeRoot(12.0) },
		{ "vfine",   scaleFromCubeRoot(15.0) }
	};

	if(types.count(granularityStr) == 0)
		return types.at(DEFAULT_GRANULARITY);

	return types.at(granularityStr);
}

[[nodiscard]] size_t getDirSeparator(const std::string& filename)
{
	size_t unixDirSeparator = filename.find_last_of('/');
	size_t winDirSeparator = filename.find_last_of('\\');

	if(unixDirSeparator == std::string::npos 
		&& winDirSeparator == std::string::npos)
	{
		return std::string::npos;
	}
	else if(unixDirSeparator == std::string::npos)
		return winDirSeparator;
	else if(winDirSeparator == std::string::npos)
		return unixDirSeparator;
	
	return unixDirSeparator > winDirSeparator 
		? unixDirSeparator : winDirSeparator;
}

[[nodiscard]] std::string getLocal(const std::string& filename)
{
	size_t dirSeparator = getDirSeparator(filename);
	return filename.substr(
		dirSeparator == std::string::npos ? 0 : dirSeparator + 1);
}

[[nodiscard]] std::string extractModelName(const std::string& modelFilePath)
{
	std::string localFileName = getLocal(modelFilePath);

	size_t extSeparator = localFileName.find_last_of('.');

	std::string name = extSeparator == 0 
		? localFileName.substr(1) : localFileName.substr(0, extSeparator);

	size_t firstValidChar = name.find_first_not_of('.');
	if(firstValidChar == std::string::npos)
		throw std::runtime_error("Model file name not valid!");

	std::string trimmedName = name.substr(firstValidChar);

	for(size_t i = 0; i < trimmedName.length(); ++i)
		if(trimmedName[i] == '.') trimmedName[i] = '-';

	return trimmedName;
}

[[nodiscard]] Config handleArguments(const Arguments& args) 
{
	Config config;

	if(args.count(false) <= 0 || args.getFlag("--help"))
	{
		printUsage();
		exit(EXIT_SUCCESS);
	}

	auto inputFile = args.get("--in");
	if(inputFile)
		config.inputFile = *inputFile;

	auto outputDir = args.get("--out");
	if(outputDir)
		config.outputDir = *outputDir;
	else
		config.outputDir = extractModelName(config.inputFile);
	config.outputDir.erase(config.outputDir.find_last_not_of("\\/")+1);

	int numPrinters = -1;	// Require this from the user
	auto numPrintersArg = args.getInt("--num");
	if(numPrintersArg)
		numPrinters = *numPrintersArg;
	config.numPrinters = numPrinters;

	validateInputFile(config.inputFile);
	validateOutputDir(config.outputDir);
	validateNumPrinters(config.numPrinters);

	std::string printerConfigFile(DEFAULT_PRINTER_CONFIG_PATH);
	auto printerConfigPath = args.get("--printer");
	if(printerConfigPath)
		printerConfigFile = *printerConfigPath;
	config.printerFile = printerConfigFile;

	INIFile printerConfig = loadINIFile(printerConfigFile);
	config.printer = makePrinterSpec(printerConfig);


	double granularity = granularityScaleFrom(DEFAULT_GRANULARITY);
	const auto granularityArg = args.get("--granularity");
	if(granularityArg)
		granularity = granularityScaleFrom(*granularityArg);
	config.granularityScale = granularity;

	int sampleTries = DEFAULT_SAMPLES;
	auto sampleTriesArg = args.getInt("--sample-tries");
	if(sampleTriesArg)
		sampleTries = *sampleTriesArg;
	config.sampleTries = sampleTries;


	float layerHeight = config.printer.nozzleSize * DEFAULT_LAYER_HEIGHT_SCALE;
	auto layerHeightArg = args.getFloat("--quality");
	if(layerHeightArg)
		layerHeight = *layerHeightArg;
	config.layerHeight = layerHeight;

	int walls = DEFAULT_WALL_COUNT;
	auto wallsArg = args.getInt("--walls");
	if(wallsArg)
		walls = *wallsArg;
	config.wallCount = walls;
	config.wallThickness = walls * config.printer.nozzleSize;

	float infillDensity = DEFAULT_INFILL_DENSITY;
	auto infillDensityArg = args.getFloat("--infill");
	if(infillDensityArg)
		infillDensity = *infillDensityArg;
	config.infillDensity = infillDensity;

	float supportDensity = DEFAULT_SUPPORT_DENSITY;
	auto supportDensityArg = args.getFloat("--support");
	if(supportDensityArg)
		supportDensity = *supportDensityArg;
	config.supportDensity = supportDensity;

	config.seed = args.getLong("--seed");

	auto cleanAfter = args.getFlag("--clean-out");
	if(cleanAfter)
		config.cleanupOutDirAfter = true;
	else
		config.cleanupOutDirAfter = false;

	auto relaxSafeties = args.getFlag("--relax-safeties");
	if(relaxSafeties)
		config.relaxSafeties = true;
	else
		config.relaxSafeties = false;

	auto noMinPrinters = args.getFlag("--no-min-printers");
	if(noMinPrinters)
		config.failOnInsufficientPrinters = false;
	else
		config.failOnInsufficientPrinters = true;

	auto symmetrySkipArg = args.getFlag("--skip-symmetry");
	if(symmetrySkipArg)
		config.symmetrySkip = true;
	else
		config.symmetrySkip = false;


	return config;
}

void printSentence(const std::vector<std::string>& words)
{
	auto itr = words.cbegin(); 
	while(itr != words.cend())
	{
		std::cout << *itr;
		++itr;
		if(itr != words.cend())
			std::cout << ", ";
	}
	std::cout << "." << std::endl;
}

void printConfig(const Config& config)
{
	std::cout << "Config" << std::endl;
	std::cout << "------------------------" << std::endl;

#ifdef VERBOSE
	std::cout << "Running with VERBOSE enabled." << std::endl;
#endif

	const PrinterSpec& printer = config.printer;
	std::cout << "Printer specifications:" << std::endl;
	std::cout << "Printer config: " << getLocal(config.printerFile) << std::endl;

	std::cout << "    Print volume: [" 
		      << printer.volume.width << " mm, "
			  << printer.volume.height << " mm, "
			  << printer.volume.depth << " mm]" << std::endl;

	std::cout << "    Inner wall speed:  " 
		      << printer.speeds.innerWall << " mm/s" << std::endl;
	std::cout << "    Outer wall speed:  " 
		      << printer.speeds.outerWall << " mm/s" << std::endl;
	std::cout << "    Infill speed: " 
		      << printer.speeds.infill << " mm/s" << std::endl;
	std::cout << "    Support speed: " 
		      << printer.speeds.support << " mm/s" << std::endl;

	std::cout << "    Nozzle size: " 
		      << printer.nozzleSize << " mm" << std::endl;
	std::cout << "    Overhang tolerance: " 
		      << printer.overhangTolerance << "°" << std::endl;

	std::cout << "Layer height: " << config.layerHeight << " mm" << std::endl;
	std::cout << "Numer of walls: " << config.wallCount; 
	std::cout << " (" << config.wallThickness << " mm)" << std::endl;

	std::cout << "Infill density: " << config.infillDensity * 100 << "\%" << std::endl;
	std::cout << "Support density: " << config.supportDensity * 100 << "\%" << std::endl;

	int numBoxes = static_cast<int>(1.0 / config.granularityScale);
	std::cout << "Number of initial boxes: ~" << numBoxes << std::endl;

	std::cout << "Number of sample attempts: " << config.sampleTries << std::endl;

	std::cout << "Number of printers: " << config.numPrinters << std::endl;

	if(config.seed)
		std::cout << "Custom seed: " << *config.seed << std::endl;

	std::vector<std::string> flags;
	if(config.symmetrySkip)
		flags.push_back("[Symmetry Skipping]");
	if(config.relaxSafeties)
		flags.push_back("[Relaxed Mesh Safeties]");
	if(!config.failOnInsufficientPrinters)
		flags.push_back("[Proceed With Insufficient Printers]");
	if(config.cleanupOutDirAfter) flags.push_back("[Cleaning Output After]");

	std::cout << "Flags: ";
	printSentence(flags);
	std::cout << std::endl;

	std::cout << "Output directory: " << config.outputDir << std::endl;

	std::cout << std::endl;
}
 
