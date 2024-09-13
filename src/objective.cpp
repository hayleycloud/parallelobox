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

	return static_cast<int>(meshVolume / printerVolume);
}

double overhangCost(
	const Config& config, 
	const Mesh& mesh, 
	const auto& fnormals,
	const K::Vector_3& floor)
{
	return overhangArea(config, mesh, fnormals, floor);
}

double printingCost(const Config& config, const Mesh& mesh)
{
	const double volumeCost = config.printer.infillSpeed * PMP::volume(mesh);
	const double surfaceCost = config.printer.shellSpeed * PMP::area(mesh);
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

bool fitsVolume(
	const Config& config, const std::vector<Direction>& up, const Mesh& mesh)
{
	K::Point_3 min, max;
	boundsLocal(mesh, min, max);

	const double width  = abs(max.x() - min.x());
	const double height = abs(max.y() - min.y());
	const double depth  = abs(max.z() - min.z());

	const double allowedWidth = config.printer.volume.width;
	const double allowedHeight = config.printer.volume.height;
	const double allowedDepth = config.printer.volume.depth;

	//std::cout << "FV: " << width << ", " << height << ", " << depth;
	//std::cout << " | " << allowedWidth << ", " << allowedHeight << ", " << allowedDepth;
	//std::cout << std::endl;

	for(Direction direction: up)
	{
		switch(direction)
		{
			case Direction::Left:
			case Direction::Right:
			{
				if(fitsVolume(
					height, width, depth, 
					allowedWidth, allowedHeight, allowedDepth))
					return true;
			}
			break;
			case Direction::Up:
			case Direction::Down:
			{
				if(fitsVolume(
					width, height, depth, 
					allowedWidth, allowedHeight, allowedDepth))
					return true;
			}
			break;
			case Direction::In:
			case Direction::Out:
			{
				if(fitsVolume(
					width, depth, height, 
					allowedWidth, allowedHeight, allowedDepth))
					return true;
			}
			break;
		}
	}

	return false;
}

/*double fitness(const Config& config, const Mesh& mesh)
{
	const double printCost = printingCost(config, mesh);
	if(printCost == 0.0)
		return 0.0;

	const double fitVolumeCost = fitsVolume(config, mesh);

	return printCost + fitVolumeCost;
}*/

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
