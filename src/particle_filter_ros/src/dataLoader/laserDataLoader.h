#ifndef __LASER_DATA_LOADER_H__
#define __LASER_DATA_LOADER_H__

#include <string>
#include <vector>

#include <typedefs.h>

namespace imr
{

using imr::Measurement;

namespace laserDataLoader
{

class LaserDataLoader
{
    private:

        MeasurementList measurements;

    public:

        LaserDataLoader(const char* _dataFile, size_t nMeasurements, const std::string& _keyWord);

        const Measurement& operator[](size_t index) const
        {
            return measurements.at(index);
        }

        inline size_t size() const
        {
            return measurements.size();
        }

        inline bool empty() const
        {
            return measurements.empty();
        }
};

} // namespace laserDataLoader
} // namespace imr

#endif
