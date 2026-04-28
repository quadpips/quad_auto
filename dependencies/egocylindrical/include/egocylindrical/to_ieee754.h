#ifndef TO_IEEE754_H
#define TO_IEEE754_H

/* The following code is copied from https://stackoverflow.com/q/38278300/2906021 */

#include <limits> // IEEE754 compliance test
#include <type_traits> // enable_if

template<
    typename T, 
    typename = typename std::enable_if<std::is_floating_point<T>::value>::type,
    typename = typename std::enable_if<std::numeric_limits<T>::is_iec559>::type,
    typename u_t = typename std::conditional<std::is_same<T, float>::value, uint32_t, uint64_t>::type
>
struct IEEE754 {

    enum class WIDTH : size_t {
        SIGN = 1, 
        EXPONENT = std::is_same<T, float>::value ? 8 : 11,
        MANTISSA = std::is_same<T, float>::value ? 23 : 52
    };
    enum class MASK : u_t {
        SIGN = (u_t)1 << (sizeof(u_t) * 8 - 1),
        EXPONENT = ((~(u_t)0) << (size_t)WIDTH::MANTISSA) ^ (u_t)MASK::SIGN,
        MANTISSA = (~(u_t)0) >> ((size_t)WIDTH::SIGN + (size_t)WIDTH::EXPONENT)
    };
    union {
        T f;
        u_t u;
    };

    IEEE754(T f) : f(f) {}

    inline u_t sign() const { return u & (u_t)MASK::SIGN >> ((size_t)WIDTH::EXPONENT + (size_t)WIDTH::MANTISSA); }
    inline u_t exponent() const { return u & (u_t)MASK::EXPONENT >> (size_t)WIDTH::MANTISSA; }
    inline u_t mantissa() const { return u & (u_t)MASK::MANTISSA; }

    inline bool isNan() const {
        return (mantissa() != 0) && ((u & ((u_t)MASK::EXPONENT)) == (u_t)MASK::EXPONENT);
    }
};
template<typename T>
inline IEEE754<T> toIEEE754(T val) { return IEEE754<T>(val); }

#endif
