#include <egocylindrical/ecwrapper.h>
#include <functional>


using namespace egocylindrical;
using namespace egocylindrical::utils;

// class Tests
// {
// public:
   
  
    bool point_conversion(ECWrapper& points)
    {
        int num_pts = points.getCols();
        const float* x = (const float*)points.getX();
        const float* y = (const float*)points.getY();
        const float* z = (const float*)points.getZ();
        for(int i = 0; i < num_pts; ++i)
        {
            cv::Point3d p(x[i],y[i],z[i]);
            cv::Point2d pp;
            float range;
            points.project3dToPixelRange(p, pp, range);
            cv::Point3d reproj = range * points.projectPixelTo3dRay(pp);
            double error = cv::norm(p-reproj);
            // ROS_INFO_STREAM("Error=" << error);
        }

      return true;
    }
    
    bool point_conversion2(ECWrapper& points)
    {
        int num_pts = points.getCols();
        double factor = 10.0/num_pts;
//         const float* x = (const float*)points.getX();
//         const float* y = (const float*)points.getY();
//         const float* z = (const float*)points.getZ();
        std::vector<double> range_errors, pix_errors;
        float range=factor;
        for(int i = 0; i < points.getHeight(); ++i)
        {
            for(int j = 0; j < points.getWidth(); ++j)
            {
                cv::Point2d pp(i,j);
//                 double range = points.pixToIdx(pp);
                cv::Point3d reproj = range * points.projectPixelTo3dRay(pp);
                //cv::Point3d p(x[i],y[i],z[i]);
                float proj_range;
                cv::Point2d pp2;
                points.project3dToPixelRange(reproj, pp2, proj_range);
                double range_error = range - proj_range;
                double pix_error = cv::norm(pp-pp2);
                // // ROS_INFO_STREAM("Pixel0=" << pp<< ", Range0=" << range << ", Point=" << reproj << ", Pixel=" << pp2 << ", Range=" << proj_range << ", Range Error=" << range_error << ", Pix Error=" << pix_error);
                range+=factor;
            }
        }

      return true;
    }

    ECWrapper timeCopy(ECWrapper& ec)
    {
      auto t1 = ros::WallTime::now();
      ECWrapper e = ec; //copy constructor
      auto t2 = ros::WallTime::now();

      // ROS_INFO_STREAM("Copy time for " << e.getNumPts() << " points: " << (t2-t1).toSec()*1e3 << "ms");

      return e;
    }

    ECWrapper timeCopy2(ECWrapper& ec)
    {
      auto t1 = ros::WallTime::now();
      ECWrapper e(ec.getParams());
      auto t2 = ros::WallTime::now();
      {
        auto* rhp = ec.getPoints();
        auto* lhp = e.getPoints();
        for(int i = 0; i < e.getNumPts()*3; i++)
        {
          lhp[i] = rhp[i];
        }
      }
      auto t3 = ros::WallTime::now();

      // ROS_INFO_STREAM("Allocation time for " << e.getNumPts() << " points: " << (t2-t1).toSec()*1e3 << "ms");
      // ROS_INFO_STREAM("Copy time for " << e.getNumPts() << " points: " << (t3-t2).toSec()*1e3 << "ms");


      return e;
    }

    bool verifyCopy(const ECWrapper& ec1, const ECWrapper& ec2)
    {
      int num_pts = ec1.getNumPts();
      float* x = (float*)ec1.getX();
      float* y = (float*)ec1.getY();
      float* z = (float*)ec1.getZ();

      int num_pts2 = ec2.getNumPts();
      float* x2 = (float*)ec2.getX();
      float* y2 = (float*)ec2.getY();
      float* z2 = (float*)ec2.getZ();

      if(num_pts != num_pts2)
      {
        return false;
      }

      for(int i=0; i < num_pts; i++)
      {
        if(x[i]!=x2[i] || y[i]!=y2[i] || z[i]!=z2[i])
        {
          return false;
        }
      }

      return true;
    }


    void fill_basic(ECWrapper::Ptr points)
    {
      int num_pts = points->getNumPts();
      float* x = (float*)points->getX();
      float* y = (float*)points->getY();
      float* z = (float*)points->getZ();

      for(int i = 0; i < num_pts; ++i)
      {
        x[i] = i/1000;
        y[i] = i/100;
        z[i] = i/10;
      }
      
    }
    
    
    bool testCopy(std::function<ECWrapper(ECWrapper&)> copy_func)
    {
      ECParams params;
      params.height=64;
      params.width=256;
      params.vfov=2;
      params.can_width=128;

      std::vector<int> heights = {64, 128, 256, 512};
      std::vector<int> widths = {256, 512, 1024, 2048};
      
      for(int i=0; i < heights.size(); i++)
      {
        params.height = heights[i];
        params.width = widths[i];
        params.can_width = params.width/4;

        for(int j=0; j < 5; j++)
        {
          auto points = getECWrapper(params);
          fill_basic(points);
          ECWrapper cp = copy_func(*points);

          if(!verifyCopy(cp, *points))
          {
            // // ROS_ERROR(_STREAM("Copy failed verification!");
            return false;
          }
          point_conversion2(cp);
        }

      }
      return true;
    }

    
    ECWrapper::Ptr getPts()
    {
      ECParams params;
      params.height=64;
      params.width=256;
      params.vfov=2;
      params.can_width=128;
      
      auto points = getECWrapper(params);
      
      return points;
    }

    bool assignmentTest(ECWrapper::Ptr pts)
    {
      auto ec_copy = *pts;
      *pts = ec_copy;
      return true;
    }
    
    
    bool runTests()
    {
      testCopy(&timeCopy);
      testCopy(&timeCopy2);

      auto pts = getPts();
      assignmentTest(pts);

      return point_conversion2(*pts);
    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tester");
    runTests();
}


