/** Detray library, part of the ACTS project (R&D line)
 *
 * (c) 2022 CERN for the benefit of the ACTS project
 *
 * Mozilla Public License Version 2.0
 */

#pragma once

#include <chrono>
#include <fstream>
#include <iostream>

#ifdef DETRAY_LOG_TIME
#define LOG 1
#else
#define LOG 0
#endif

namespace detray::timer {

static std::chrono::high_resolution_clock::time_point start;
static std::chrono::high_resolution_clock::time_point stop;

#define START_TIMER()                                                         \
    {                                                                         \
        if (LOG) {                                                            \
            detray::timer::start = std::chrono::high_resolution_clock::now(); \
        }                                                                     \
    }

#define STOP_TIMER()                                                         \
    {                                                                        \
        if (LOG) {                                                           \
            detray::timer::stop = std::chrono::high_resolution_clock::now(); \
        }                                                                    \
    }

#define PRINT_TIME()                                                    \
    {                                                                   \
        if (LOG) {                                                      \
            std::chrono::duration<double> diff =                        \
                detray::timer::stop - detray::timer::start;             \
            std::cout << "Time: " << diff.count() << " s" << std::endl; \
        }                                                               \
    }

#define PRINT_TIME_TO_FILE(filename)                                   \
    {                                                                  \
        if (LOG) {                                                     \
            std::chrono::duration<double> diff =                       \
                detray::timer::stop - detray::timer::start;            \
            std::ofstream output_file;                                 \
            output_file.open(filename,                                 \
                             std::ofstream::out | std::ofstream::app); \
            output_file << diff.count() << "\n";                       \
            output_file.close();                                       \
        }                                                              \
    }

#define TIME(action)                     \
    {                                    \
        START_TIMER()                    \
        action STOP_TIMER() PRINT_TIME() \
    }

#define TIME_TO_FILE(action, filename)                   \
    {                                                    \
        START_TIMER()                                    \
        action STOP_TIMER() PRINT_TIME_TO_FILE(filename) \
    }

}  // namespace detray::timer
