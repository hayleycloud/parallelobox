#pragma once

#include <functional>

namespace mv {

template <class T>
using vector2 = std::vector<std::vector<T>>;

template <class T>
using vector3 = std::vector<std::vector<std::vector<T>>>;

template <class T>
T& get(vector2<T>& vec, int x, int y) 
{
	return vec[y][x];
}

template <class T>
const T& get(const vector2<T>& vec, int x, int y) 
{
	return vec[y][x];
}

template <class T>
T& get(vector3<T>& vec, int x, int y, int z)
{
	return vec[z][y][x];
}

template <class T>
const T& get(const vector3<T>& vec, int x, int y, int z)
{
	return vec[z][y][x];
}

template <class T>
void forEach(std::function<void(T&)> callback, vector2<T>& data)
{
	for(size_t y = 0; y < data.size(); ++y)
	{
		for(size_t x = 0; x < data[y].size(); ++x)
		{
			callback(data[y][x]);
		}
	}
}

template <class T>
void forEach(std::function<void(T&)> callback, vector3<T>& data)
{
	for(size_t z = 0; z < data.size(); ++z)
	{
		for(size_t y = 0; y < data[z].size(); ++y)
		{
			for(size_t x = 0; x < data[z][y].size(); ++x)
			{
				callback(data[z][y][x]);
			}
		}
	}
}

template <class T>
void forEachIndexed(
	std::function<void(T&,int,int)> callback, vector2<T>& data)
{
	for(size_t y = 0; y < data.size(); ++y)
	{
		for(size_t x = 0; x < data[y].size(); ++x)
		{
			callback(data[y][x], x, y);
		}
	}
}

template <class T>
void forEachIndexed(
	std::function<void(T&,int,int,int)> callback, vector3<T>& data)
{
	for(size_t z = 0; z < data.size(); ++z)
	{
		for(size_t y = 0; y < data[z].size(); ++y)
		{
			for(size_t x = 0; x < data[z][y].size(); ++x)
			{
				callback(data[z][y][x], x, y, z);
			}
		}
	}
}

template <class T1, class T2>
vector2<T2> map(std::function<T2(T1&)> callback, vector2<T1>& data)
{
	vector2<T2> out;
	out.resize(data.size());

	for(size_t yIndex = 0; yIndex < data.size(); ++yIndex)
	{
		auto& yData = data[yIndex];
		out[yIndex].resize(yData.size());

		for(size_t xIndex = 0; xIndex < yData.size(); ++xIndex)
		{
			out[yIndex][xIndex] = callback(yData[xIndex]);
		}
	}

	return out;
}

template <class T1, class T2>
vector3<T2> map(std::function<T2(T1&)> callback, vector3<T1>& data)
{
	vector3<T2> out;
	out.resize(data.size());

	for(size_t zIndex = 0; zIndex < data.size(); ++zIndex)
	{
		auto& zData = data[zIndex];
		out[zIndex].resize(zData.size());

		for(size_t yIndex = 0; yIndex < zData.size(); ++yIndex)
		{
			auto& yData = zData[yIndex];
			out[zIndex][yIndex].resize(yData.size());

			for(size_t xIndex = 0; xIndex < yData.size(); ++xIndex)
			{
				out[zIndex][yIndex][xIndex] = callback(yData[xIndex]);
			}
		}
	}

	return out;
}

template <class T1, class T2>
T2 reduce(
	std::function<void(T2&,T1&)> callback, vector3<T1>& data)
{
	T2 out;

	for(size_t z = 0; z < data.size(); ++z)
	{
		for(size_t y = 0; y < data[z].size(); ++y)
		{
			for(size_t x = 0; x < data[z][y].size(); ++x)
			{
				callback(out, data[z][y][x]);
			}
		}
	}

	return out;
}

}
