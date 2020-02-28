#pragma once
#include <random>
#include <omp.h>

std::default_random_engine engine[16];
std::uniform_real_distribution<double> uniform(0, 1);