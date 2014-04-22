#ifndef BALL_PICKER_LAYER_H_
#define BALL_PICKER_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <ball_picker/Detections.h>

namespace costmap_2d
{

  class BallPickerLayer : public Layer, Costmap2D
  {
    public:
      BallPickerLayer();

      virtual void onInitialize();
      virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      bool isDiscretized()
      {
        return true;
      }

      virtual void matchSize();

    protected:
      void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
      dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;


      std::vector<ball_picker::PointOfInterest> obstacle_buffer; 
      ros::Subscriber obstacles_sub;

      void obstaclesIncomeCallback(const ball_picker::Detections& obstacles);

  };
}
#endif

