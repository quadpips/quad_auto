
#ifndef EGOCYLINDRICAL_RANGE_IMAGE_COMMON_H
#define EGOCYLINDRICAL_RANGE_IMAGE_COMMON_H

#include <limits>


namespace egocylindrical
{
    namespace utils
    {
        namespace temp
        {
            constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
        }

        template<typename T> struct RangeVals {};

        template<>
        struct RangeVals<uint16_t>
        {
            static inline uint scale() { return 1000;}
            static inline bool is_valid(uint16_t v) {return v > 0;}
            static constexpr uint unknown = 0;
        };

        template<>
        struct RangeVals<float>
        {
            static inline uint scale() { return 1;}
            static inline bool is_valid(float v) {return v == v;}
            static constexpr float unknown = temp::dNaN;
        };

    }
}

#endif //EGOCYLINDRICAL_RANGE_IMAGE_COMMON_H