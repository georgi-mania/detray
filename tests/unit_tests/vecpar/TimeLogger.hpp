#ifndef DETRAY_TIMELOGGER_HPP
#define DETRAY_TIMELOGGER_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <vecmem/containers/vector.hpp>

namespace detray {

// Utility functions to log execution time(s) in an output CSV file
class Logger {

    public:
    // Open/Create the filename with the given 'filename' and
    // print the times separated by commas, on a single line
    static void logTime(std::string filename, double t1) {

        std::ofstream myFile;
        myFile.open(filename, std::ofstream::out | std::ofstream::app);
        myFile << t1 << std::endl;

        myFile.close();
    }

    // Concatenate the given 'keywords' separated by '_'
    // e.g. buildFilename("nTracks", "100") -> "nTracks_100";
    // 	buildFilename("nTracks", "100", "1GeV", "CPU") ->
    // "nTracks_100_1GeV_CPU";
    template <typename... T>
    static std::string buildFilename(const T &...keywords) {
        std::vector<std::string> keysVec = {keywords...};
        std::string filename("Results_");
        auto appFn = [&filename](std::string s) {
            filename.append(s).append("_");
        };

        for (auto key : keysVec) {
            appFn(key);
        }

        filename = filename.substr(0, filename.length() - 1).append(".csv");
        return filename;
    }
};
}
#endif  // DETRAY_TIMELOGGER_HPP
