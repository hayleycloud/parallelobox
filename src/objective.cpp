#include "objective.h"
#include "align.h"
#include "debug.h"

int calcMinNumPrinters(const Config& config, const Mesh& mesh)
{
	K::Point_3 min, max;
	bounds(mesh, min, max);

	K::Vector_3 dims = max - min;

	std::list<double> sortedMeshDims;
	sortedMeshDims.push_back(dims.x());
	sortedMeshDims.push_back(dims.y());
	sortedMeshDims.push_back(dims.z());
	sortedMeshDims.sort();

	std::list<double> sortedPrinterDims;
	sortedPrinterDims.push_back(config.printer.volume.width);
	sortedPrinterDims.push_back(config.printer.volume.height);
	sortedPrinterDims.push_back(config.printer.volume.depth);
	sortedPrinterDims.sort();

	int numPrinters = 1;
	auto meshDimItr = sortedMeshDims.begin();
	auto printDimItr = sortedPrinterDims.begin();
	while(meshDimItr != sortedMeshDims.end() && printDimItr != sortedPrinterDims.end())
	{
		numPrinters *= std::ceil(*meshDimItr / *printDimItr);
		++meshDimItr;
		++printDimItr;
	}
	return numPrinters >= 2 ? numPrinters : 2;
}

double featureSpeed(const Config& config, double speed)
{
	return config.printer.nozzleSize * config.layerHeight * speed;
}

double rawPrintingCost(const Config& config, const Mesh& mesh)
{
	const double volume = PMP::volume(mesh);
	const double area = PMP::area(mesh);

	const double innerWallVol = area * config.printer.nozzleSize;
	const double outerWallVol = 
		area * (config.wallThickness - config.printer.nozzleSize);

	const double infillVol = 
		(volume - (config.wallThickness * area)) * config.infillDensity;

	const double innerWallCost = innerWallVol / 
		featureSpeed(config, config.printer.speeds.innerWall);
	const double outerWallCost = outerWallVol / 
		featureSpeed(config, config.printer.speeds.outerWall);
	const double infillCost = infillVol / 
		featureSpeed(config, config.printer.speeds.infill); 

	return innerWallCost + outerWallCost + infillCost;
}

double overhangCost(
	const Config& config,
	const Mesh& mesh,
	const MeshNormalsMap& fnormals,
	const K::Vector_3& up,
	const K::Vector_3& floor)
{
	double supportVolume = overhangVolume(config, mesh, fnormals, up, floor);
	return (config.supportDensity * supportVolume) 
		/ featureSpeed(config, config.printer.speeds.support);
}

bool fitsVolume(
	double width, double height, double depth,
	double allowedWidth, double allowedHeight, double allowedDepth)
{
	if(height < allowedHeight)
	{
		if((width < allowedWidth && depth < allowedDepth)
			|| (width < allowedDepth && depth < allowedWidth))
		{
			return true;
		}
	}

	return false;
}

bool fitsVolume(const Config& config, double width, double height, double depth)
{
	double biggest = std::max(width, std::max(height, depth));
	double smallest = 
		std::min(config.printer.volume.width, std::min(
				config.printer.volume.height, config.printer.volume.depth));
	return biggest <= smallest;
}

std::vector<Direction> calcOrientationsThatFit(const Config& config, const Mesh& mesh)
{
	K::Point_3 min, max;
	boundsLocal(mesh, min, max);

	const double width  = abs(max.x() - min.x());
	const double height = abs(max.y() - min.y());
	const double depth  = abs(max.z() - min.z());

	if(fitsVolume(config, width, height, depth))
	{
		return {
			Direction::Left, Direction::Right,
			Direction::Up, Direction::Down,
			Direction::In, Direction::Out
		};
	}
	else {
		return {};
	}

	const double allowedWidth = config.printer.volume.width;
	const double allowedHeight = config.printer.volume.height;
	const double allowedDepth = config.printer.volume.depth;
	
	std::vector<Direction> allowedUpDirs;

	if(fitsVolume(
		height, width, depth, 
		allowedWidth, allowedHeight, allowedDepth))
	{
		allowedUpDirs.push_back(Direction::Left);
		allowedUpDirs.push_back(Direction::Right);
	}

	if(fitsVolume(
		width, height, depth, 
		allowedWidth, allowedHeight, allowedDepth))
	{
		allowedUpDirs.push_back(Direction::Up);
		allowedUpDirs.push_back(Direction::Down);
	}

	if(fitsVolume(
		width, depth, height, 
		allowedWidth, allowedHeight, allowedDepth))
	{
		allowedUpDirs.push_back(Direction::In);
		allowedUpDirs.push_back(Direction::Out);
	}

	return allowedUpDirs;
}

bool fitsVolume(const Config& config, const Mesh& mesh)
{
	return !calcOrientationsThatFit(config, mesh).empty();
}

double computeBestOverhangCost(
	const Config& config, 
	const Grid& grid, 
	const std::vector<Direction>& allowedUpDirs,
	const MeshBox& meshBox)
{
	auto nmap = meshBox.mesh.property_map<face_descriptor,K::Vector_3>("f:normals");
	auto fnormals = *nmap;
	
	// Determine the floors of the orientations that fit the printer
	std::vector<K::Vector_3> floors;
	getFloorVectorsFrom(grid, meshBox, allowedUpDirs, floors);

	double bestOverhangCost = std::numeric_limits<double>::max();
	for(size_t floorIndex = 0; floorIndex < floors.size(); ++floorIndex)
	{
		K::Vector_3 up = toVector(allowedUpDirs[floorIndex]);
		double cost = overhangCost(
			config, meshBox.mesh, fnormals, up, floors[floorIndex]);
#ifdef XVERBOSE
		std::cout << "\tOverhang Oriented " << toText(allowedUpDirs[floorIndex])
			<< ": " << cost << std::endl;
#endif
		if(cost < 0.0)
		{
			std::cerr << "OOPS?" << std::endl;
		}

		if(cost < bestOverhangCost)
			bestOverhangCost = cost;
	}

	return bestOverhangCost;
}

double printingCost(
	const Config& config, 
	const Grid& grid, 
	const MeshBox& meshBox)
{
	std::vector<Direction> allowedUpDirs = 
		calcOrientationsThatFit(config, meshBox.mesh);

	double simplePrintingCost = rawPrintingCost(config, meshBox.mesh);

	double overhangCost = computeBestOverhangCost(
		config, grid, allowedUpDirs, meshBox);

	return std::abs(simplePrintingCost + overhangCost);
}

void pruneCache(
	const std::vector<const MeshBox*>& meshBoxPtrs,
	PrintingCostCache& cache)
{
	std::vector<const MeshBox*> deadEntries;
	for(auto& cacheEntry: cache)
	{
		if(std::find(meshBoxPtrs.cbegin(), meshBoxPtrs.cend(), cacheEntry.first) == meshBoxPtrs.cend())
			deadEntries.push_back(cacheEntry.first);
	}

	for(const MeshBox* dead: deadEntries)
		cache.erase(dead);
}

void pruneCache(
	const std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	PrintingCostCache& cache)
{
	std::vector<const MeshBox*> mbPtrs;
	mbPtrs.reserve(meshBoxes.size());
	for(auto& meshBox: meshBoxes)
		mbPtrs.push_back(meshBox.get());
	pruneCache(mbPtrs, cache);
}

inline double computePrintingCostAndCache(
	const Config& config, 
	const Grid& grid, 
	const MeshBox& meshBox,
	PrintingCostCache& cache)
{
	double printCost = printingCost(config, grid, meshBox);
	cache.insert_or_assign(
		std::addressof(meshBox), 
		(PrintingCostCacheData) { meshBox.dims, printCost });
	return printCost;
}

double printingCost(
	const Config& config, 
	const Grid& grid, 
	const MeshBox& meshBox,
	PrintingCostCache& cache)
{
	const MeshBox* mbPtr = std::addressof(meshBox);
	auto cacheEntry = cache.find(mbPtr);
	if(cacheEntry == cache.end())
		return computePrintingCostAndCache(config, grid, meshBox, cache);

	if(cacheEntry->second.lastDims == meshBox.dims)
		return cacheEntry->second.printCost;

	return computePrintingCostAndCache(config, grid, meshBox, cache);
}

double parallelPrintingCost(
	const Config& config, 
	const Grid& grid, 
	const std::vector<const MeshBox*>& printers,
	PrintingCostCache& cache)
{
	double highestCost = std::numeric_limits<double>::lowest();

	for(const MeshBox* mesh: printers)
	{
		double printCost = printingCost(config, grid, *mesh, cache);
		if(printCost > highestCost)
			highestCost = printCost;
	}

	return highestCost;
}
 
double parallelPrintingCost(
	const Config& config, 
	const Grid& grid, 
	const std::vector<std::unique_ptr<MeshBox>>& printers,
	PrintingCostCache& cache)
{
	double highestCost = std::numeric_limits<double>::lowest();

	for(const auto& mesh: printers)
	{
		double printCost = printingCost(config, grid, *mesh, cache);
		if(printCost > highestCost)
			highestCost = printCost;
	}

	return highestCost;
}

