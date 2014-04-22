#include<ball_picker/ball_picker_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::BallPickerLayer, costmap_2d::Layer)


using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d 
{

  BallPickerLayer::BallPickerLayer() {}

  void BallPickerLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_), g_nh;

    obstacle_buffer.clear();
    obstacles_sub = g_nh.subscribe("costmap_custom_obstacles", 1, &BallPickerLayer::obstaclesIncomeCallback, this);

    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&BallPickerLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }


  void BallPickerLayer::matchSize()
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
  }


  void BallPickerLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void BallPickerLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (!(enabled_) || (obstacle_buffer.empty()))
      return;
    
    if (layered_costmap_->isRolling())
      updateOrigin(origin_x - getSizeInMetersX() / 2, origin_y - getSizeInMetersY() / 2);

    double mark_x, mark_y;
    unsigned int mx, my;
    unsigned int index;

    for (std::vector<ball_picker::PointOfInterest>::iterator iter = obstacle_buffer.begin() ; iter != obstacle_buffer.end(); iter++)
    {

      //mark_x = origin_x + iter->x;
      //mark_y = origin_y + iter->y;
      mark_x = iter->x;
      mark_y = iter->y;

      std::cout << "mark_x je: " << mark_x << std::endl;
      std::cout << "mark_y je: " << mark_y << std::endl;

      if(worldToMap(mark_x, mark_y, mx, my))
      {
        std::cout << "proslo pres konverzi " << mx << ", " << my << std::endl; 
        index = getIndex(mx, my);
        std::cout << "index je: " << index << std::endl;
        costmap_[index] = LETHAL_OBSTACLE;      

        //setCost(mx, my, LETHAL_OBSTACLE);
      }
      else
      {
         std::cout << "neproslo pres konverzi" << std::endl;
      }
  
      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);

    }

    obstacle_buffer.clear();
  }

  void BallPickerLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    if (!enabled_)
      return;

    const unsigned char* master_array = master_grid.getCharMap();

    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        int index = getIndex(i, j);

        if (costmap_[index] == NO_INFORMATION)
          continue;
        
        unsigned char old_cost = master_array[index];

        if (old_cost == NO_INFORMATION || old_cost < costmap_[index])
          master_grid.setCost(i, j, costmap_[index]); 
      }
    }
  }

  void BallPickerLayer::obstaclesIncomeCallback(const ball_picker::Detections& obstacles)
  {
    ball_picker::PointOfInterest pt;

    for(unsigned int i=0; i < obstacles.ballcenters.size(); i++)
    {
      pt.x = obstacles.ballcenters[i].x;
      pt.y = obstacles.ballcenters[i].y;

      obstacle_buffer.push_back(pt);
    }
  }

} // end namespace

