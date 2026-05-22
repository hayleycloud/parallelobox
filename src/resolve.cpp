#include "reporting.h"
#include "resolve.h"

size_t idOf(const MeshBox* meshBox, const std::vector<MeshBox*>& meshBoxes)
{
	size_t index = 0;

	for(const MeshBox* other: meshBoxes)
	{
		if(meshBox == other)
			return index;
		++index;
	}

	return -1;
}

/*MeshBox* mergeEmptyRegion(
	const Config& config,
	MeshBox& emptyFiller,
	std::vector<const MeshBox*>& meshBoxes,
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	std::vector<MeshBox*> neighbors = getNeighbours(grid, emptyFiller, gridCells);
	size_t neighborCount = neighbors.size();

	std::vector<std::unique_ptr<MeshBox>> options(neighborCount);
	std::vector<MeshBox*> neighborSrc(neighborCount);

	#pragma omp parallel for default(none) shared(config, emptyFiller, neighborCount, options, neighbors, neighborSrc)
	for(size_t i = 0; i < neighborCount; ++i)
	{
		MeshBox* neighbor = neighbors[i];
		std::unique_ptr<MeshBox> merger = 
			std::make_unique<MeshBox>(emptyFiller.mesh, emptyFiller.dims);

		//std::cout << "Testing #" << 
		//	idOf(std::addressof(emptyFiller), meshBoxes) << " with #" <<
		//	idOf(neighbor, meshBoxes) << std::endl;

		bool succeeded = false;
		if(mergeSoft(*merger, *neighbor))
		{
			if(fitsVolume(config, merger->mesh))
			{
				options[i] = std::move(merger);
				neighborSrc[i] = neighbor;
				succeeded = true;
			}
		}

		if(!succeeded)
		{
			options[i] = nullptr;
			neighborSrc[i] = nullptr;
		}
	}

	double bestScore = std::numeric_limits<double>::max();
	size_t bestIndex = -1;
	for(size_t i = 0; i < neighborCount; ++i)
	{
		if(!options[i])
			continue;

		double score = printingCost(config, grid, *options[i], printCostCache);
		if(score < bestScore)
		{
			bestScore = score;
			bestIndex = i;
		}
	}

	if(bestIndex >= 0 
		&& bestScore < std::numeric_limits<double>::max() 
		&& bestScore <= currentParallelScore)
	{
		MeshBox& bestOption = *options[bestIndex];
		MeshBox& neighbor = *neighborSrc[bestIndex];
		neighbor.mesh = bestOption.mesh;
		neighbor.dims = bestOption.dims;
		transferChildren(neighbor, emptyFiller);

		return neighborSrc[bestIndex];
	}

	return nullptr;
}

*/
void updateMeshStorage(
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<MeshBox*>& meshBoxPtrs,
	std::vector<MeshBox*> expired)
{
	std::vector<std::unique_ptr<MeshBox>> newFillerData;
	for(auto& filler: fillers)
	{
		if(std::find(expired.begin(), expired.end(), filler.get()) 
			== expired.end())
			newFillerData.emplace_back(std::move(filler));
	}
	fillers.clear();
	for(auto& filler: newFillerData)
		fillers.emplace_back(std::move(filler));

	std::vector<std::unique_ptr<MeshBox>> newMeshBoxes;
	for(auto& meshBox: meshBoxes)
	{
		if(std::find(expired.begin(), expired.end(), meshBox.get()) 
			== expired.end())
			newMeshBoxes.push_back(std::move(meshBox));
	}
	meshBoxes.clear();
	for(auto& meshBox: newMeshBoxes)
		meshBoxes.emplace_back(std::move(meshBox));

	std::vector<MeshBox*> newMeshBoxPtrs;
	for(MeshBox* meshBox: meshBoxPtrs)
	{
		if(std::find(expired.begin(), expired.end(), meshBox) 
			== expired.end())
			newMeshBoxPtrs.push_back(meshBox);
	}
	meshBoxPtrs = newMeshBoxPtrs;
}
/*

struct ScoredMeshBox 
{
	MeshBox* meshBox;
	double printCost;
};

std::vector<MeshBox*> sortFillers(
	const Config& config,
	const Grid& grid,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	PrintingCostCache& printCostCache)
{
	std::vector<ScoredMeshBox> meshBoxScores;
	meshBoxScores.reserve(fillers.size());

	std::list<ScoredMeshBox*> sortList;

	for(auto& filler: fillers)
	{
		ScoredMeshBox smb = {
			filler.get(), 
			printingCost(config, grid, *filler, printCostCache)
		};
		meshBoxScores.push_back(smb);
		sortList.push_back(std::addressof(meshBoxScores.back()));
	}

	sortList.sort([](ScoredMeshBox* a, ScoredMeshBox* b) {
		return a->printCost > b->printCost;
	});

	std::vector<MeshBox*> sorted;
	sorted.reserve(fillers.size());

	for(ScoredMeshBox* filler: sortList)
		sorted.push_back(filler->meshBox);

	return sorted;
}

int mergeEmptyRegions(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<const MeshBox*>& meshBoxPtrs,
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	std::vector<MeshBox*> expired;

	std::vector<MeshBox*> sortedFillers = 
		sortFillers(config, grid, fillers, printCostCache);

	for(MeshBox* filler: sortedFillers)
	{
		if(std::find(expired.begin(), expired.end(), filler) != expired.end())
			continue;

#if REPORT_VERBOSE
		int id = idOf(filler, meshBoxPtrs);
		//std::cout << "Attempting to merge empty " << i << " (#" << id << ")" << std::endl;
#endif

		MeshBox* target = mergeEmptyRegion(
			config, 
			*filler, 
			meshBoxPtrs, 
			currentParallelScore, 
			grid, gridCells,
			printCostCache);
		if(target)
		{
			expired.push_back(filler);
#if REPORT_VERBOSE
			std::cout << "Merged empty #" << id << " with #" << idOf(target, meshBoxPtrs) << std::endl;
#endif
		}
	}

	updateMeshStorage(meshBoxes, fillers, meshBoxPtrs, expired);
	pruneCache(meshBoxPtrs, printCostCache);

	return expired.size();
}*/


/*struct MergeOption
{
	std::unique_ptr<MeshBox> newMeshBox;
	MeshBox* to;
	MeshBox* from;
	double printingCost;
};

void commitRelaxedMerge(MergeOption& mergeOp)
{
	mergeOp.to->mesh = mergeOp.newMeshBox->mesh;
	mergeOp.to->dims = mergeOp.newMeshBox->dims;
	transferChildren(*mergeOp.to, *mergeOp.from);
}

std::unique_ptr<MergeOption> mergeEmptyRegionRelaxed(
	const Config& config,
	MeshBox& emptyFiller,
	std::vector<const MeshBox*>& meshBoxes,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	std::vector<MeshBox*> neighbors = getNeighbours(grid, emptyFiller, gridCells);
	size_t neighborCount = neighbors.size();

	std::vector<std::unique_ptr<MeshBox>> options(neighborCount);
	std::vector<MeshBox*> neighborSrc(neighborCount);

	#pragma omp parallel for default(none) shared(config, emptyFiller, neighborCount, options, neighbors, neighborSrc)
	for(size_t i = 0; i < neighborCount; ++i)
	{
		MeshBox* neighbor = neighbors[i];
		std::unique_ptr<MeshBox> merger = 
			std::make_unique<MeshBox>(emptyFiller.mesh, emptyFiller.dims);

		bool succeeded = false;
		if(mergeSoft(*merger, *neighbor))
		{
			if(fitsVolume(config, merger->mesh))
			{
				options[i] = std::move(merger);
				neighborSrc[i] = neighbor;
				succeeded = true;
			}
		}

		if(!succeeded)
		{
			options[i] = nullptr;
			neighborSrc[i] = nullptr;
		}
	}

	double bestScore = std::numeric_limits<double>::max();
	size_t bestIndex = -1;
	for(size_t i = 0; i < neighborCount; ++i)
	{
		if(!options[i])
			continue;

		double score = printingCost(config, grid, *options[i], printCostCache);
		if(score < bestScore)
		{
			bestScore = score;
			bestIndex = i;
		}
	}

	if(bestIndex >= 0 && bestScore < std::numeric_limits<double>::max()) 
	{
		return std::make_unique<MergeOption>(
			std::move(options[bestIndex]),
			std::addressof(emptyFiller),
			neighborSrc[bestIndex],
			bestScore
		);
	}

	return nullptr;
}

std::unique_ptr<MergeOption> findBestMergeOption(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<const MeshBox*>& meshBoxPtrs,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	std::vector<std::unique_ptr<MergeOption>> mergeOpts;
	for(int i = 0; i < fillers.size(); ++i)
	{
		auto& filler = fillers[i];

#if REPORT_VERBOSE
		int id = idOf(filler.get(), meshBoxPtrs);
		std::cout << "Attempting to merge empty " << i << " (#" << id << ")" << std::endl;
#endif

		std::unique_ptr<MergeOption> mergeOpt = mergeEmptyRegionRelaxed(
			config, 
			*filler, 
			meshBoxPtrs, 
			grid, gridCells,
			printCostCache);
		if(mergeOpt)
		{
			mergeOpts.emplace_back(std::move(mergeOpt));
			std::cout << "Valid merge option detected." << std::endl;
#if REPORT_VERBOSE
			//std::cout << "Merged empty #" << id << " with #" << idOf(target, meshBoxPtrs) << std::endl;
#endif
		}
	}

	std::unique_ptr<MergeOption> bestOption = nullptr;
	for(auto& mergeOpt: mergeOpts)
	{
		if(!bestOption)
			bestOption = std::move(mergeOpt);
		else if(mergeOpt->printingCost < bestOption->printingCost)
			bestOption = std::move(mergeOpt);
	}

	return bestOption;
}


int mergeEmptyRegionsRelaxed(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<const MeshBox*>& meshBoxPtrs,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache)
{
	int numMerged = 0;
	bool optionFound = true;
	while(optionFound)
	{
		std::unique_ptr<MergeOption> mergeOpt = findBestMergeOption(
			config,
			meshBoxes, fillers, meshBoxPtrs,
			grid, gridCells,
			printCostCache);

		if(mergeOpt)
		{
			commitRelaxedMerge(*mergeOpt);
			updateMeshStorage(meshBoxes, fillers, meshBoxPtrs, {mergeOpt->from});
			pruneCache(meshBoxPtrs, printCostCache);
			++numMerged;
		}
		else
			optionFound = false;
	}

	return numMerged;
}*/

struct MergeOptionCacheData
{
	Mesh* a;
	Mesh* b;
	double printCost;
};

typedef std::list<MergeOptionCacheData> MergeOptionCache;

std::optional<double> getCostFrom(
	const MergeOptionCache& cache, Mesh& a, Mesh& b)
{
	Mesh *aPtr = std::addressof(a), *bPtr = std::addressof(b);
	for(const MergeOptionCacheData& entry: cache)
	{
		if((entry.a == aPtr && entry.b == bPtr) 
			|| (entry.a == bPtr && entry.b == aPtr))
			return entry.printCost;
	}

	return std::nullopt;
}

void storeIn(Mesh& a, Mesh& b, double printCost, MergeOptionCache& cache)
{
	cache.emplace_back((MergeOptionCacheData) { 
		std::addressof(a), std::addressof(b), printCost });
}

struct MergeOption
{
	std::unique_ptr<MeshBox> newMeshBox;
	MeshBox* to;
	MeshBox* from;
	double printingCost;
};

void commitRelaxedMerge(MergeOption& mergeOp)
{
	mergeOp.to->mesh = mergeOp.newMeshBox->mesh;
	mergeOp.to->dims = mergeOp.newMeshBox->dims;
	transferChildren(*mergeOp.to, *mergeOp.from);
}

std::unique_ptr<MergeOption> getMergeOption(
	const Config& config,
	MeshBox& target,
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	MergeOptionCache& cache)
{
	bool enforceParallelScore = currentParallelScore > 0.0;

	std::vector<MeshBox*> neighbors = getNeighbours(grid, target, gridCells);
	size_t neighborCount = neighbors.size();

	std::vector<std::unique_ptr<MeshBox>> options(neighborCount);
	std::vector<MeshBox*> neighborSrc(neighborCount);
	std::vector<double> scores(neighborCount);

	//#pragma omp parallel for default(none) shared(config, grid, target, neighborCount, options, neighbors, scores, neighborSrc, cache)
	for(size_t i = 0; i < neighborCount; ++i)
	{
		MeshBox* neighbor = neighbors[i];

		std::optional<double> cachedScore = getCostFrom(
			cache, target.mesh, neighbor->mesh);
		if(cachedScore)
		{
			options[i] = nullptr;
			neighborSrc[i] = neighbor;
			scores[i] = *cachedScore;
		}
		else
		{
			std::unique_ptr<MeshBox> merger = 
				std::make_unique<MeshBox>(target.mesh, target.dims);

			bool succeeded = false;
			if(mergeSoft(*merger, *neighbor))
			{
				if(fitsVolume(config, merger->mesh))
				{
					double score = printingCost(config, grid, *merger);
					options[i] = std::move(merger);
					neighborSrc[i] = neighbor;
					scores[i] = score;
					succeeded = true;
				}
			}

			if(!succeeded)
			{
				options[i] = nullptr;
				neighborSrc[i] = nullptr;
				scores[i] = -1.0;
			}

			storeIn(target.mesh, neighbor->mesh, scores[i], cache);
		}
	}

	double bestScore = std::numeric_limits<double>::max();
	int bestIndex = -1;
	for(int i = 0; i < neighborCount; ++i)
	{
		if(neighborSrc[i] == nullptr)
			continue;

		double score = scores[i];
		if(score > 0.0 && score < bestScore)
		{
			bestScore = score;
			bestIndex = i;
		}
	}

	if(bestIndex >= 0 && bestScore < std::numeric_limits<double>::max()) 
	{
		if(!(enforceParallelScore && bestScore > currentParallelScore))
		{
			if(options[bestIndex])
				return std::make_unique<MergeOption>(
					std::move(options[bestIndex]),
					std::addressof(target),
					neighborSrc[bestIndex],
					bestScore
				);
			else
			{
				std::unique_ptr<MeshBox> newMesh =
					std::make_unique<MeshBox>(target.mesh, target.dims); 

				bool succeeded = false;
				if(mergeSoft(*newMesh, *neighborSrc[bestIndex]))
				{
					if(fitsVolume(config, newMesh->mesh))
						succeeded = true;
				}

				if(succeeded)
					return std::make_unique<MergeOption>(
						std::move(newMesh),
						std::addressof(target),
						neighborSrc[bestIndex],
						bestScore
					);
			}
		}
	}

	return nullptr;
}

std::unique_ptr<MergeOption> findBestMergeOption(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<MeshBox*>& meshBoxPtrs,
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	MergeOptionCache& cache)
{
	std::vector<std::unique_ptr<MergeOption>> mergeOpts(meshBoxPtrs.size());
	#pragma omp parallel for default(none) shared(config, mergeOpts, meshBoxPtrs, grid, gridCells, cache, currentParallelScore)
	for(size_t i = 0; i < meshBoxPtrs.size(); ++i)
	//for(MeshBox* target: meshBoxPtrs)
	{
		MeshBox* target = meshBoxPtrs[i];
		mergeOpts[i] = getMergeOption(
			config, 
			*target, 
			currentParallelScore,
			grid, gridCells,
			cache);
	}

	std::unique_ptr<MergeOption> bestOption = nullptr;
	for(auto& mergeOpt: mergeOpts)
	{
		if(mergeOpt)
		{
			if(!bestOption)
				bestOption = std::move(mergeOpt);
			else if(mergeOpt->printingCost < std::numeric_limits<double>::max() 
					&& mergeOpt->printingCost < bestOption->printingCost)
				bestOption = std::move(mergeOpt);
		}
	}

	return bestOption;
}


int mergeRegions(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<MeshBox*>& meshBoxPtrs,
	double currentParallelScore,
	int numAvailablePrinters,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	MergeOptionCache& cache)
{
	int numMerged = 0;
	bool optionFound = true;
	while(optionFound && meshBoxPtrs.size() > numAvailablePrinters)
	{
		std::unique_ptr<MergeOption> mergeOpt = findBestMergeOption(
			config,
			meshBoxes, fillers, meshBoxPtrs,
			currentParallelScore,
			grid, gridCells,
			cache);

		if(mergeOpt && mergeOpt->printingCost < std::numeric_limits<double>::max())
		{
			commitRelaxedMerge(*mergeOpt);
			updateMeshStorage(meshBoxes, fillers, meshBoxPtrs, {mergeOpt->from});

			//std::vector<const MeshBox*> mbPtrs;
			//for(auto& mb: meshBoxPtrs)
			//	mbPtrs.push_back(mb);
			//for(auto& mb: meshBoxPtrs)
			//	mbPtrs.push_back(mb);
			//pruneCache(mbPtrs, printCostCache);
			++numMerged;
		}
		else
			optionFound = false;
	}

	return numMerged;
}

bool mergeEmptyRegions(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<std::unique_ptr<MeshBox>>& fillers,
	std::vector<const MeshBox*>& meshBoxPtrs,
	double currentParallelScore,
	int numPrintersAvailable,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	PrintingCostCache& printCostCache,
	bool constrainParallelScore)
{
	std::vector<MeshBox*> mbPtrs;
	for(auto& mb: meshBoxes)
		mbPtrs.push_back(mb.get());
	for(auto& mb: fillers)
		mbPtrs.push_back(mb.get());

	size_t numMeshBoxes = meshBoxPtrs.size();
	if(numMeshBoxes <= numPrintersAvailable)
	{
		std::cout << "No need to merge." << std::endl;
		return true;
	}

	MergeOptionCache cache;
	int numMerged = mergeRegions(
		config,
		meshBoxes, fillers, mbPtrs,
		currentParallelScore,
		numPrintersAvailable,
		grid, gridCells,
		cache);

	meshBoxPtrs.clear();
	for(MeshBox* mb: mbPtrs)
		meshBoxPtrs.push_back(mb);

	bool success = meshBoxPtrs.size() <= numPrintersAvailable;

	size_t newNumMeshBoxes = meshBoxPtrs.size();
		std::cout << "Merged " << numMerged << " meshes: " 
			<< numMeshBoxes << " meshes => " 
			<< newNumMeshBoxes << " meshes (Merge "
		    << (success ? "succeeded" : "failed") << ")." << std::endl;

	if(!constrainParallelScore && !success)
	{
		numMeshBoxes = newNumMeshBoxes;

		numMerged = mergeRegions(
			config,
			meshBoxes, fillers, mbPtrs,
			-1.0,
			numPrintersAvailable,
			grid, gridCells,
			cache);

		meshBoxPtrs.clear();
		for(MeshBox* mb: mbPtrs)
			meshBoxPtrs.push_back(mb);

		success = meshBoxPtrs.size() <= numPrintersAvailable;

		newNumMeshBoxes = meshBoxPtrs.size();
		std::cout << "Aggressively merged " << numMerged << " meshes: " 
			<< numMeshBoxes << " meshes => " 
			<< newNumMeshBoxes << " meshes (Merge "
			<< (success ? "succeeded" : "failed") << ")." << std::endl;
	}
	/*
	int numChanges = 1;
	size_t numMeshBoxes = meshBoxPtrs.size();
	while(numChanges > 0 && numMeshBoxes > numPrintersAvailable)
	{
		numChanges = mergeEmptyRegions(
			config, 
			meshBoxes, fillers, meshBoxPtrs, 
			currentParallelScore, 
			grid, gridCells,
			printCostCache);
		size_t newNumMeshBoxes = meshBoxPtrs.size();
//#if REPORT_EXTRA
		std::cout << "Merged: " << numMeshBoxes << " meshes => " 
			<< newNumMeshBoxes << " meshes" << std::endl;
//#endif
		numMeshBoxes = newNumMeshBoxes;
	}

	numChanges = 1;
	while(!constrainParallelScore 
		&& numChanges > 0 
		&& numMeshBoxes > numPrintersAvailable)
	{
		numChanges = mergeEmptyRegionsRelaxed(
			config, 
			meshBoxes, fillers, meshBoxPtrs, 
			grid, gridCells,
			printCostCache);
		size_t newNumMeshBoxes = meshBoxPtrs.size();
//#if REPORT_EXTRA
		std::cout << "Aggressively Merged: " << numMeshBoxes << " meshes => " 
			<< newNumMeshBoxes << " meshes" << std::endl;
//#endif
		numMeshBoxes = newNumMeshBoxes;
	}
	*/


	return success;
}
