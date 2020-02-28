#pragma once
#include "ompsetup.h"

inline double get_random() {
	return uniform(engine[omp_get_thread_num()]);
}