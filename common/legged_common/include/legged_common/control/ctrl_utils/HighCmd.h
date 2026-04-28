#//
#// Created by ziyi on 5/26/21.
#//
#
#pragma once

#include <stdint.h>
// #include "custom_cmd_lcmt.hpp"

namespace legged_software {
namespace legged_common {

struct HighCmdCustom 
{
    HighCmdCustom(){zero();}

    int8_t mode;                      // 0:idle, default stand      1:forced stand     2:walk continuously   9:paasive (for debug)

    void zero()
    {
        mode = 0;
    }
};

} // namespace legged_common
} // namespace legged_software
