#include "meshbox.h"

MeshBox::ContentType determineContentType(
	const Mesh& mesh, 
	const Mesh& surfMesh, 
	const Mesh& volMesh,
	const Grid& grid, 
	int x, int y, int z)
{
	if(surfMesh.number_of_vertices() > 0)
		return MeshBox::ContentType::Boundary;
	else 
	{
		if(volMesh.number_of_vertices() > 0)
			return MeshBox::ContentType::Internal;
		else
			return MeshBox::ContentType::Empty;
	}
}

mv::vector3<MeshBox> getSurfaceBoxes(Mesh& mesh, const Grid& grid)
{
	mv::vector3<MeshBox> gridBoxes;
	gridBoxes.resize(grid.getThickness());
	
	#pragma omp parallel for
	for(int z = 0; z < grid.getThickness(); ++z)
	{
		gridBoxes[z].resize(grid.getHeight());
		for(int y = 0; y < grid.getHeight(); ++y)
		{
			gridBoxes[z][y].resize(grid.getWidth());
			for(int x = 0; x < grid.getWidth(); ++x)
			{
				const auto& dims = grid.get(x, y, z);

				Mesh surfMesh(mesh);
				grid.clipSurface(surfMesh, x, y, z);

				Mesh volMesh(mesh);
				grid.clipVolume(volMesh, x, y, z);

				auto contentType = determineContentType(
					mesh, surfMesh, volMesh,
					grid, 
					x, y, z);

				gridBoxes[z][y][x] = { 
					contentType == MeshBox::ContentType::Boundary ?
						volMesh : surfMesh, 
					dims, 
					contentType 
				};
			}
		}
	}

	return gridBoxes;
}

