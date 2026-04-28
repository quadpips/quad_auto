#ifndef EGOCYLINDRICAL_POINT_TRANSFORMER_H
#define EGOCYLINDRICAL_POINT_TRANSFORMER_H


#include <egocylindrical/ecwrapper.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace egocylindrical
{
    
    namespace utils
    {

      //void transformPoints(utils::ECWrapper& points, const geometry_msgs::msg::TransformStamped& trans);
      
      void transformPoints(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const utils::ECWrapper& new_points, const geometry_msgs::msg::TransformStamped& trans, int num_threads=1);

    }

}

#endif //EGOCYLINDRICAL_POINT_TRANSFORMER_H
