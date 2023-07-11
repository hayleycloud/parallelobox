#include "objective.h"
#include "align.h"

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
	return volumeCost + surfaceCost;
}

double fitsVolumeCost(const Config& config, const Mesh& mesh)
{
	K::Point_3 min, max;
	bounds(mesh, min, max);

	const double width  = abs(max.x() - min.x());
	const double height = abs(max.y() - min.y());
	const double depth  = abs(max.z() - min.z());

	if(width < config.printer.volume.width &&
	   height < config.printer.volume.height &&
	   depth < config.printer.volume.depth)
		return 0.0;
	else
		return std::numeric_limits<double>::max();
}

double fitness(const Config& config, const Mesh& mesh)
{
	return printingCost(config, mesh) + fitsVolumeCost(config, mesh);
}
