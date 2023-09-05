#pragma once

#include <random>

typedef std::uniform_real_distribution RNG;

[[nodiscard]] template<class T> RNG<T> makeRNG(T min, T max);

[[nodiscard]] T fetchRandom(RNG<T> rng);
