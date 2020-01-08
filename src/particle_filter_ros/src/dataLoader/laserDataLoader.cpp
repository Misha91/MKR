#include <cstdlib> // For EXIT_FAILURE.
#include <fstream>
#include <iostream>
#include <sstream>

#include <typedefs.h>
#include "laserDataLoader.h"

namespace imr
{

namespace laserDataLoader
{

LaserDataLoader::LaserDataLoader(const char* _dataFile, size_t nMeasurements, const std::string& _keyWord)
{
    size_t measurement_count = 0;

    std::ifstream ifs(_dataFile);
    if (!ifs.is_open())
    {
        std::cerr << "File " << _dataFile << " cannot be opened." << std::endl;
        exit(EXIT_FAILURE);
    }

    while (ifs && measurement_count < nMeasurements)
    {
        // Read a line from file input stream.
        std::string line;
        std::getline(ifs, line);

        // Construct a string stream.
        std::stringstream iss_line(line);
        std::string keyword;
        if ((iss_line >> keyword).fail())
        {
            // Continue to next line if empty line.
            continue;
        }
        if (keyword.compare(_keyWord) == 0)
        {
            std::cout << "Loading measurement " << ++measurement_count;

            std::string temp;

            unsigned int nmeas;
            if ((iss_line >> nmeas).fail())
            {
                std::cerr << "Error parsing type in input\n";
                exit(EXIT_FAILURE);
            }

            std::cout << " containing " << nmeas << " measurements." << std::endl;

            Measurement new_measurement;
            // Read all laser measurements
            for (unsigned int i = 0; i < nmeas; i++)
            {
                double measurement;
                if ((iss_line >> measurement).fail())
                {
                    std::cerr << "Error parsing measurement data in input\n";
                    exit(EXIT_FAILURE);
                }
                new_measurement.scan.push_back(measurement);
            }

            // Read position (x,y,theta)
            if ((iss_line >> new_measurement.position.x).fail())
            {
                std::cerr << "Error parsing position x in input\n";
                exit(EXIT_FAILURE);
            }
            if ((iss_line >> new_measurement.position.y).fail())
            {
                std::cerr << "Error parsing position y in input\n";
                exit(EXIT_FAILURE);
            }
            if ((iss_line >> new_measurement.position.phi).fail())
            {
                std::cerr << "Error parsing position phi in input\n";
                exit(EXIT_FAILURE);
            }

            measurements.push_back(new_measurement);
        }
    }

    if (measurement_count > 0)
    {
        std::cout << "Data loaded ..." << std::endl;
    }
    else
    {
        std::cout << "No data could be loaded!" << std::endl;
    }
}

} // namespace laserDataLoader
} // namespace imr

