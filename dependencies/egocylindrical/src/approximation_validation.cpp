#include <egocylindrical/ecwrapper.h>


//Max error: 0.0690737 @ 10.3055
float testInverse()
{
  float max_error = 0;
  float max_value = -1;
  
  for(float x=0.001; x < 1000; x+=.001)
  {    
    float truth = 1/x;
    float approx = egocylindrical::utils::inverse_approximation(x);
    float error = std::abs(truth - approx);
    float percent_error = error / truth;
    
    //std::cout << "Truth: " << truth << ", Approx: " << approx << ", Error: " << error << ", %Error: " << percent_error << " @ " << x << std::endl;
    
    
    if(percent_error > max_error)
    {
      max_error = percent_error;
      max_value = x;
    }
  }

  std::cout << "Inverse: Max error: " << max_error << " @ " << max_value << std::endl;
}

//This is probably good enough for just about anything
//max error: 4.79221e-06 @ 42363.5
float testInvSqrt()
{
  float max_error = 0;
  float max_value = -1;
  
  for(float x=0.001; x < 100000; x+=.01)
  {    
    float truth = 1/std::sqrt(x);
    float approx = egocylindrical::utils::inv_sqrt_approximation(x);
    float error = std::abs(truth - approx);
    float percent_error = error / truth;
    
    //std::cout << "Truth: " << truth << ", Approx: " << approx << ", Error: " << error << ", %Error: " << percent_error << " @ " << x << std::endl;
    
    
    if(percent_error > max_error)
    {
      max_error = percent_error;
      max_value = x;
    }
  }
  
  std::cout << "InvSqrt: Max error: " << max_error << " @ " << max_value << std::endl;
}


// Max error: 0.0101497 radians, or 1/620 of a circle. If width about that big, probably shouldn't use this
float testAtan2()
{
  float max_error = 0;
  float max_value = -1;
  float max_y = -1;
  float twopi = 2*std::acos(-1);
  
  for(float t = 0; t < twopi; t+=.0001)
  {
    float x = std::cos(t);
    float y = std::sin(t);
    
    float truth = std::atan2(y,x);
    float approx = egocylindrical::utils::atan2_approximation1(y,x);
    float error = std::abs(truth - approx);
    float percent_error = error / twopi;
    
    //std::cout << "Truth: " << truth << ", Approx: " << approx << ", Error: " << error << ", %Error: " << percent_error << " @ " << x << std::endl;
    
    
    if(error > max_error)
    {
      max_error = error;
      max_value = t;
    }
  
  }
  
  std::cout << "Atan2: Max error: " << max_error << " @ th=" << max_value <<std::endl;
}

// Max error: 0.0101497 radians, or 1/620 of a circle. If width about that big, probably shouldn't use this
float testAtan22()
{
  float max_error = 0;
  float max_value = -1;
  float max_y = -1;
  
  
  for(float x = -20; x<20; x+=.01)
  {
    for(float y=-20; y<20; y+=.01)
    {
      if(x==0 && y==0)
        continue;
      
      float truth = std::atan2(y,x);
      float approx = egocylindrical::utils::atan2_approximation1(y,x);
      float error = std::abs(truth - approx);
      
      //std::cout << "Truth: " << truth << ", Approx: " << approx << ", Error: " << error << ", %Error: " << percent_error << " @ " << x << std::endl;
    
    
      if(error > max_error)
      {
        max_error = error;
        max_value = x;
        max_y = y;
      }
    }
    
  }
  
  std::cout << "Atan22: Max error: " << max_error << " @ th=" << max_value <<std::endl;
}



int main()
{
  #ifndef BOOST_ARCH_ARM_AVAILABLE
    std::cout << "Not ARM!" << std::endl;
  #else
    std::cout << "On ARM!" << std::endl;
  #endif
    
  testInverse();
  testInvSqrt();
  testAtan2();
  testAtan22();
}
