#include<ball_picker/ball_picker_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::BallPickerLayer, costmap_2d::Layer)


using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace costmap_2d 
{

  BallPickerLayer::BallPickerLayer() {}

  void BallPickerLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_), g_nh;

    nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    nh.param("robot_global_frame", global_frame_, std::string("/map"));

    ros::NodeHandle prefix_nh;
    std::string tf_prefix = tf::getPrefixParam(prefix_nh);

    global_frame_ = tf::resolve(tf_prefix, global_frame_);
    robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);


    while (!tfl.waitForTransform(global_frame_, robot_base_frame_, ros::Time::now(), ros::Duration(1.0)))
    {
      ROS_WARN_THROTTLE(1.0, "Transofrm for looking up robot pose not available.");
      ros::spinOnce();
    }


    obstacle_buffer.clear();
    obstacles_sub = g_nh.subscribe("costmap_custom_obstacles", 1, &BallPickerLayer::obstaclesIncomeCallback, this);

    clear_srv = nh.advertiseService("ball_picker_clear_costmaps", &BallPickerLayer::clearObstacles, this);

    clear_flag = false;
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
    if (!enabled_) 
      return;
    
    if (layered_costmap_->isRolling())
      updateOrigin(origin_x - getSizeInMetersX() / 2, origin_y - getSizeInMetersY() / 2);

    double mark_x, mark_y;
    unsigned int mx, my;
    unsigned int index;

    if (clear_flag)
    {
      *min_x = *min_x - getSizeInMetersX()/2;
      *min_y = *min_y - getSizeInMetersY()/2;
      *max_x = *max_x + getSizeInMetersX()/2;
      *max_y = *max_y + getSizeInMetersY()/2;
    }

    for (std::vector<ball_picker::PointOfInterest>::iterator iter = obstacle_buffer.begin() ; iter != obstacle_buffer.end(); iter++)
    {
      mark_x = iter->x;
      mark_y = iter->y;

      if(worldToMap(mark_x, mark_y, mx, my))
      {
        index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;      
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

        if (clear_flag)
        {
          master_grid.setCost(i, j, FREE_SPACE);
        }
        else
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

     for (int i=0; i < getSizeInCellsX()*getSizeInCellsY(); i++)
       costmap_[i] == NO_INFORMATION;

  }

  void BallPickerLayer::obstaclesIncomeCallback(const ball_picker::Detections& obstacles)
  {
    ball_picker::PointOfInterest pt;

    for(unsigned int i=0; i < obstacles.objectcenters.size(); i++)
    {
      pt.x = obstacles.objectcenters[i].x;
      pt.y = obstacles.objectcenters[i].y;

      obstacle_buffer.push_back(pt);
    }
  }

  bool BallPickerLayer::clearObstacles(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    clear_flag = true;
    tf::Stamped <tf::Pose> pose;
    if (getRobotPose(pose))
      layered_costmap_->updateMap(pose.getOrigin().x(), pose.getOrigin().y(), tf::getYaw(pose.getRotation()));
    else
      layered_costmap_->updateMap(0.0, 0.0, 0.0);
    clear_flag = false;
  }



  bool BallPickerLayer::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
  {
    global_pose.setIdentity();

    tf::Stamped <tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();

    try
    {
      tfl.transformPose(global_frame_, robot_pose, global_pose);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "TF exception looking up robot pose: %s/n", ex.what());
      return false; 
    }

    return true;

  }





} // end namespace

