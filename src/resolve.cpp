#include "reporting.h"
#include "resolve.h"
#include "objective.h"

#define PARALLEL_RESOLVE_OFF	0
#define PARALLEL_RESOLVE_NARROW	1
#define PARALLEL_RESOLVE_BROAD	2

void updateStorage(
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	std::vector<MeshBox*> expired)
{
	std::vector<std::unique_ptr<MeshBox>> newMeshBoxes;
	for(auto& meshBox: meshBoxes)
	{
		if(std::find(expired.begin(), expired.end(), meshBox.get()) 
			== expired.end())
			newMeshBoxes.push_back(std::move(meshBox));
	}
	meshBoxes.clear();
	for(auto& meshBox: newMeshBoxes)
		meshBoxes.push_back(std::move(meshBox));
}

struct MergeOptionCacheData
{
	Mesh* a;
	Mesh* b;
	double printCost;
};

typedef std::vector<MergeOptionCacheData> MergeOptionCache;

std::optional<MergeOptionCacheData> lookup(
	const MergeOptionCache& cache, Mesh* a, Mesh* b)
{
	for(const MergeOptionCacheData& entry: cache)
	{
		if((entry.a == a && entry.b == b) 
			|| (entry.a == b && entry.b == a))
			return entry;
	}

	return std::nullopt;
}

std::optional<MergeOptionCacheData> lookup(
	const MergeOptionCache& cache, Mesh& a, Mesh& b)
{
	return lookup(cache, std::addressof(a), std::addressof(b));
}

std::optional<double> getCostFrom(
	const MergeOptionCache& cache, Mesh& a, Mesh& b)
{
	std::optional<MergeOptionCacheData> cacheData = lookup(cache, a, b);
	if(cacheData)
		return cacheData->printCost;
	else
		return std::nullopt;
}

void storeIn(Mesh& a, Mesh& b, double printCost, MergeOptionCache& cache)
{
	cache.emplace_back((MergeOptionCacheData) { 
		std::addressof(a), std::addressof(b), printCost });
}

typedef std::unique_ptr<MergeOptionCacheData> PreparedMergeCacheOp;
typedef std::vector<PreparedMergeCacheOp> PreparedMergeCacheOps;

PreparedMergeCacheOp prepareCacheStore(Mesh& a, Mesh& b, double printCost)
{
	return std::make_unique<MergeOptionCacheData>((MergeOptionCacheData){ 
		std::addressof(a), std::addressof(b), printCost });
}

void prepareCacheStore(Mesh& a, Mesh& b, double printCost, PreparedMergeCacheOps& ops)
{
	ops.push_back(std::move(prepareCacheStore(a, b, printCost)));
}

void commitToCache(PreparedMergeCacheOp op, MergeOptionCache& cache)
{
	std::optional<MergeOptionCacheData> existing = lookup(cache, op->a, op->b);
	if(existing)
		existing->printCost = op->printCost;
	else
		cache.push_back(*op);
}

void commitToCache(PreparedMergeCacheOps& ops, MergeOptionCache& cache)
{
	for(auto& op: ops)
	{
		if(op)
			commitToCache(std::move(op), cache);
	}
}

void synchronize(MergeOptionCache& into, MergeOptionCache& from)
{
	for(MergeOptionCacheData& fromEntry: from)
	{
		bool found = false;
		for(MergeOptionCacheData& intoEntry: into)
		{
			found |= ((fromEntry.a == intoEntry.a && fromEntry.b == intoEntry.b) 
				|| (fromEntry.a == intoEntry.b && fromEntry.b == intoEntry.a));
		}

		if(!found)
			into.push_back(fromEntry);
	}
}

void synchronize(MergeOptionCache& cache, std::vector<MergeOptionCache>& caches)
{
	for(MergeOptionCache& otherCache: caches)
		synchronize(cache, otherCache);
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

void invalidate(const MergeOption& opt, MergeOptionCache& cache)
{
	const Mesh* aPtr = std::addressof(opt.to->mesh);
	const Mesh* bPtr = std::addressof(opt.from->mesh);

	MergeOptionCache freshCache;
	freshCache.reserve(cache.size());

	for(auto& entry: cache)
	{
		if(entry.a != aPtr && entry.a != bPtr 
			&& entry.b != aPtr && entry.b != bPtr)
		{
			freshCache.push_back(entry);
		}
	}

	cache = freshCache;
}

std::unique_ptr<MergeOption> getMergeOption(
	const Config& config,
	MeshBox& target,
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	MergeOptionCache& cache,
	PreparedMergeCacheOps& cacheOps)
{
	bool enforceParallelScore = currentParallelScore > 0.0;

	std::vector<MeshBox*> neighbors = getNeighbours(grid, target, gridCells);

	bool foundBestOption = false;
	std::unique_ptr<MeshBox> bestOption = nullptr;
	double bestScore = std::numeric_limits<double>::max();
	MeshBox* bestNeighbor = nullptr;

	for(MeshBox* neighbor: neighbors)
	{
		if(neighbor == std::addressof(target))
			continue;

		std::optional<double> cachedScore = getCostFrom(
			cache, target.mesh, neighbor->mesh);
		if(cachedScore)
		{
			double score = *cachedScore;
			if(score > 0.0 && score < bestScore)
			{
				bestScore = score;
				bestNeighbor = neighbor;
				foundBestOption = true;
			}
		}
		else
		{
			std::unique_ptr<MeshBox> merger1 = 
				std::make_unique<MeshBox>(target.mesh, target.dims);
			std::unique_ptr<MeshBox> merger2 = 
				std::make_unique<MeshBox>(neighbor->mesh, neighbor->dims);

			double score = -1.0;
			if(mergeSoft(*merger1, *merger2))
			{
				if(fitsVolume(config, merger1->mesh))
				{
					score = printingCost(config, grid, *merger1);
					if(score < bestScore)
					{
						bestScore = score;
						foundBestOption = true;
						bestNeighbor = neighbor;
						bestOption = std::move(merger1);
					}
				}
			}

			prepareCacheStore(target.mesh, neighbor->mesh, score, cacheOps);
			//storeIn(target.mesh, neighbor->mesh, score, cache);
		}
	}

	if(foundBestOption)
	{
		if(enforceParallelScore && bestScore > currentParallelScore)
			return nullptr;

		if(bestOption)
			return std::make_unique<MergeOption>(
				std::move(bestOption),
				std::addressof(target),
				bestNeighbor,
				bestScore
			);
		else
		{
			std::unique_ptr<MeshBox> newMesh =
				std::make_unique<MeshBox>(target.mesh, target.dims); 
			std::unique_ptr<MeshBox> neighborCopy = 
				std::make_unique<MeshBox>(bestNeighbor->mesh, bestNeighbor->dims);

			bool succeeded = false;
			if(mergeSoft(*newMesh, *neighborCopy))
			{
				if(fitsVolume(config, newMesh->mesh))
				{
					succeeded = true;
				}
			}

			if(succeeded)
				return std::make_unique<MergeOption>(
					std::move(newMesh),
					std::addressof(target),
					bestNeighbor,
					bestScore
				);
		}
	}

	return nullptr;
}

std::unique_ptr<MergeOption> getMergeOptionMP(
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
	PreparedMergeCacheOps cacheOps(neighborCount);

	#pragma omp parallel for default(none) shared(config, grid, target, neighborCount, options, neighbors, scores, neighborSrc, cache, cacheOps)
	for(size_t i = 0; i < neighborCount; ++i)
	{
		MeshBox* neighbor = neighbors[i];
		if(neighbor == std::addressof(target))
		{
			options[i] = nullptr;
			neighborSrc[i] = nullptr;
			scores[i] = -1.0;
			cacheOps[i] = nullptr;
			continue;
		}

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

			cacheOps[i] = prepareCacheStore(
				target.mesh, neighbor->mesh, scores[i]);
		}
	}

	commitToCache(cacheOps, cache);

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
		if(enforceParallelScore && bestScore > currentParallelScore)
			return nullptr;

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

			if(mergeSoft(*newMesh, *neighborSrc[bestIndex]))
			{
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
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	MergeOptionCache& cache)
{
	std::unique_ptr<MergeOption> bestOption = nullptr;
	for(auto& target: meshBoxes)
	{
#if PARALLEL_RESOLVE == PARALLEL_RESOLVE_NARROW
		std::unique_ptr<MergeOption> mergeOpt = getMergeOptionMP(
			config, 
			*target, 
			currentParallelScore,
			grid, gridCells,
			cache);
#else
		PreparedMergeCacheOps cacheOps;

		std::unique_ptr<MergeOption> mergeOpt = getMergeOption(
			config, 
			*target, 
			currentParallelScore,
			grid, gridCells,
			cache,
			cacheOps);

		commitToCache(cacheOps, cache);
#endif


		if(mergeOpt)
		{
			if(!bestOption)
				bestOption = std::move(mergeOpt);
			else if(mergeOpt->printingCost < bestOption->printingCost)
				bestOption = std::move(mergeOpt);
		}
	}

	return bestOption;
}

std::unique_ptr<MergeOption> findBestMergeOptionMP(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	MergeOptionCache& cache)
{
	std::vector<std::unique_ptr<MergeOption>> mergeOpts(meshBoxes.size());
	std::vector<PreparedMergeCacheOps> cacheOps(meshBoxes.size());
	#pragma omp parallel for default(none) shared(config, mergeOpts, meshBoxes, grid, gridCells, cache, cacheOps, currentParallelScore)
	for(size_t i = 0; i < meshBoxes.size(); ++i)
	{
		//MergeOptionCache localCache;
		mergeOpts[i] = getMergeOption(
			config, 
			*meshBoxes[i], 
			currentParallelScore,
			grid, gridCells,
			cache,
			cacheOps[i]);
			//localCache);
	}

	for(auto& ops: cacheOps)
		commitToCache(ops, cache);
	//synchronize(cache, caches);

	std::unique_ptr<MergeOption> bestOption = nullptr;
	for(auto& mergeOpt: mergeOpts)
	{
		if(mergeOpt)
		{
			if(!bestOption)
				bestOption = std::move(mergeOpt);
			else if(mergeOpt->printingCost < bestOption->printingCost)
				bestOption = std::move(mergeOpt);
		}
	}

	return bestOption;
}


int mergeRegions(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	int numAvailablePrinters,
	double currentParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells,
	MergeOptionCache& cache)
{
	int numMerged = 0;
	bool optionFound = true;
	while(optionFound && meshBoxes.size() > numAvailablePrinters)
	{
		std::unique_ptr<MergeOption> mergeOpt = 
#if PARALLEL_RESOLVE == PARALLEL_RESOLVE_BROAD
			findBestMergeOptionMP(
#else
			findBestMergeOption(
#endif
				config,
				meshBoxes,
				currentParallelScore,
				grid, gridCells,
				cache);

		if(mergeOpt)
		{
			invalidate(*mergeOpt, cache);
			commitRelaxedMerge(*mergeOpt);
			updateStorage(meshBoxes, {mergeOpt->from});
			++numMerged;
		}
		else
			optionFound = false;
	}

	return numMerged;
}

bool resolveMeshExcess(
	const Config& config,
	std::vector<std::unique_ptr<MeshBox>>& meshBoxes,
	int numPrintersAvailable,
	double currentParallelScore,
	bool constrainParallelScore,
	const Grid& grid,
	mv::vector3<GridCell>& gridCells)
{
	size_t numMeshBoxes = meshBoxes.size();
	if(numMeshBoxes <= numPrintersAvailable)
	{
		std::cout << "No need to merge." << std::endl;
		return true;
	}

	MergeOptionCache cache;
	int numMerged = mergeRegions(
		config,
		meshBoxes,
		numPrintersAvailable,
		currentParallelScore,
		grid, gridCells,
		cache);

	bool success = meshBoxes.size() <= numPrintersAvailable;

	size_t newNumMeshBoxes = meshBoxes.size();
		std::cout << "Merged " << numMerged << " meshes: " 
			<< numMeshBoxes << " meshes => " 
			<< newNumMeshBoxes << " meshes ("
		    << (success ? "resolved" : "unresolved") << ")." << std::endl;

	if(!constrainParallelScore && !success)
	{
		numMeshBoxes = newNumMeshBoxes;

		numMerged = mergeRegions(
			config,
			meshBoxes,
			numPrintersAvailable,
			-1.0,
			grid, gridCells,
			cache);

		success = meshBoxes.size() <= numPrintersAvailable;

		newNumMeshBoxes = meshBoxes.size();
		std::cout << "Aggressively merged " << numMerged << " meshes: " 
			<< numMeshBoxes << " meshes => " 
			<< newNumMeshBoxes << " meshes ("
			<< (success ? "resolved" : "unresolved") << ")." << std::endl;
	}

	return success;
}

