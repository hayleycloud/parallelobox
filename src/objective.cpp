#include "objective.h"
#include "align.h"
#include "debug.h"

int calcMinNumPrinters(const Config& config, const Mesh& mesh)
{
	double meshVolume = PMP::volume(mesh);
	double printerVolume = 
		config.printer.volume.width * 
		config.printer.volume.height * 
		config.printer.volume.depth;

	std::cout << "MVol: " << meshVolume << std::endl;
	std::cout << "PVol: " << printerVolume << std::endl;

	return static_cast<int>(meshVolume / printerVolume);
}

double overhangCost(
	const Config& config,
	const Mesh& mesh,
	const MeshNormalsMap& fnormals,
	const K::Vector_3& up,
	const K::Vector_3& floor)
{
	return overhangArea(config, mesh, fnormals, up, floor);
}

double printingCost(const Config& config, const Mesh& mesh)
{
	const double volumeCost = PMP::volume(mesh) / config.printer.infillSpeed; 
	const double surfaceCost = PMP::area(mesh) / config.printer.shellSpeed;
	// TODO: squared_face_area for improved performance?
	//DEBUG_EXEC(std::cout << "V: " << volumeCost << ", SA: " << surfaceCost << " = " << volumeCost + surfaceCost << std::endl);
	return volumeCost + surfaceCost;
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

std::vector<Direction> calcOrientationsThatFit(const Config& config, const Mesh& mesh)
{
	K::Point_3 min, max;
	boundsLocal(mesh, min, max);

	const double width  = abs(max.x() - min.x());
	const double height = abs(max.y() - min.y());
	const double depth  = abs(max.z() - min.z());

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

/*double fitness(const Config& config, const Mesh& mesh)
{
	const double printCost = printingCost(config, mesh);
	if(printCost == 0.0)
		return 0.0;

	const double fitVolumeCost = fitsVolume(config, mesh);

	return printCost + fitVolumeCost;
}*/

double printingCost(const Config& config, const Grid& grid, const MeshBox& meshBox)
{
	std::vector<Direction> allowedUpDirs = 
		calcOrientationsThatFit(config, meshBox.mesh);

	double simplePrintingCost = printingCost(config, meshBox.mesh);

	// Calculate best orientation overhang
	///////////////////////////////////////////////////////////////////////////

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

	/*int x;
	if(rand() % 100 == 0)
	{
		K::Point_3 min, max;
		bounds(meshBox.mesh, min, max);
		std::cout << "Min: " << min << ", Max: " << max << " = " << max - min << std::endl;
	const double volumeCost = PMP::volume(meshBox.mesh) / config.printer.infillSpeed; 
	const double surfaceCost = PMP::area(meshBox.mesh) / config.printer.shellSpeed;
		std::cout << "V: " << volumeCost << ", SA: " << surfaceCost << " = " << volumeCost + surfaceCost << std::endl;
		CGAL::IO::write_STL("dfgh.stl", meshBox.mesh);
		std::cin >> x;
	}*/

	return std::abs(simplePrintingCost + bestOverhangCost);
}

double fullScore(const Config& config, const std::vector<const Mesh*>& printers)
{
	double highestCost = std::numeric_limits<double>::min();

	for(const Mesh* mesh: printers)
	{
		double printCost = printingCost(config, *mesh);
		if(printCost > highestCost)
			highestCost = printCost;
	}

	return highestCost;
}
 
