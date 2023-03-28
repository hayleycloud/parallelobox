#include "config.h"
#include "args.h"
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
		const std::string& shellSpeed = speed.at("shell");
		const std::string& infillSpeed = speed.at("infill");

		std::stringstream ss1("");
		ss1 << shellSpeed;
		ss1 >> spec.shellSpeed;

		std::stringstream ss2("");
		ss2 << infillSpeed;
		ss2 >> spec.infillSpeed;
	} catch(std::exception ex) {
		throw std::runtime_error("Error loading printer config: \"speeds\"");
	}

	try {
		const INIFile::Section& other = iniFile.sections.at("other");
		const std::string& overhangTolerance = other.at("overhang_tolerance");

		std::stringstream ss("");
		ss << overhangTolerance;
		ss >> spec.overhangTolerance;
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

Config handleArguments(const Arguments& args) 
{
	Config config;

	auto inputFile = args.get("--in");
	if(inputFile)
		config.inputFile = *inputFile;

	auto outputDir = args.get("--out");
	if(outputDir)
		config.outputDir = *outputDir;

	std::string printerConfigFile("printers/default.ini");
	auto printerConfigPath = args.get("--printer");
	if(printerConfigPath)
		printerConfigFile = *printerConfigPath;

	float infillRate = 0.2;
	auto infillRateArg = args.getFloat("--infill");
	if(infillRateArg)
		infillRate = *infillRateArg;
	config.infillRate = infillRate;

	INIFile printerConfig = loadINIFile(printerConfigFile);
	config.printer = makePrinterSpec(printerConfig);

	validateInputFile(config.inputFile);
	validateOutputDir(config.outputDir);

	return config;
}

void printConfig(const Config& config)
{
	std::cout << "Config" << std::endl;
	std::cout << "------------------------" << std::endl;

	const PrinterSpec& printer = config.printer;
	std::cout << "Printer specifications:" << std::endl;

	std::cout << "    Print volume: [" 
		      << printer.volume.width << " mm, "
			  << printer.volume.height << " mm, "
			  << printer.volume.depth << " mm]" << std::endl;

	std::cout << "    Shell speed:  " 
		      << printer.shellSpeed << " mm/s" << std::endl;
	std::cout << "    Infill speed: " 
		      << printer.infillSpeed << " mm/s" << std::endl;

	std::cout << "    Overhang tolerance: " 
		      << printer.overhangTolerance << "°" << std::endl;

	std::cout << "Infill rate: " << config.infillRate * 100 << "\%" << std::endl;
}
