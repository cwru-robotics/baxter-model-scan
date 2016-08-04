/*
 * planar_pointcloud
 * a ros node to publish a pointcloud based off of a selection of coplanar points
 * 
 * (c) 2015 Luc Bettaieb
 */

#include "model_acquisition/planar_pointcloud.h"

PlanarPublisher::PlanarPublisher(ros::NodeHandle &nh) :
  g_cloud_ptr(new PointCloud<pcl::PointXYZRGB>),
  g_selected_ptr(new PointCloud<pcl::PointXYZ>)
{
  nh_ = nh;

  COPLANAR_TOLERANCE = .85;
  PLANAR_TOLERANCE = 0.01;

  selected_pts_cb_bool = false;
  got_cloud = false;

  utils = new CwruPclUtils(&nh_);

  g_plane_normal << 0.0, 0.0, 0.0;  // ..for now?
  g_plane_distance = -1;

  selected_min_z = 999999999.0;


  cloud_sub_ = nh_.subscribe("/kinect/depth/points", 1, &PlanarPublisher::cloud_cb, this);
  selected_pts_sub_ = nh_.subscribe("/selected_points", 1, &PlanarPublisher::selected_pts_cb, this);
}

PlanarPublisher::~PlanarPublisher()
{
}

void PlanarPublisher::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  got_cloud = true;
  
  pc_frame = cloud->header.frame_id;

  pcl::fromROSMsg(*cloud, *g_cloud_ptr);
}

void PlanarPublisher::selected_pts_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  ROS_INFO("Got Selected pts");
  //std::cout << "Cloud size : " << cloud->data.size() << std::endl;
  //cloud->header.frame_id = pc_frame;

  pcl::fromROSMsg(*cloud, *g_selected_ptr);

  // Find points that are coplanar in g_cloud_ptr
  utils->fit_points_to_plane(g_selected_ptr, g_plane_normal, g_plane_distance);

  for (size_t i = 0; i < g_selected_ptr->points.size(); i++)
  {
    Vector3f xyz = g_selected_ptr->points[i].getVector3fMap();

    if (xyz(2) < selected_min_z)
      selected_min_z = xyz(2);
  }

  std::cout << g_plane_normal << std::endl;
  selected_pts_cb_bool = true;
}

//-------------MEMBER FUNCTIONS

pcl::PointCloud<pcl::PointXYZRGB> PlanarPublisher::find_coplanar_above_points()
{
  pcl::PointCloud<pcl::PointXYZRGB> view_cloud;
  ROS_INFO("Finding coplanar and above points!!");

  //std::cout << "g_cloud_ptr size: " << g_cloud_ptr->points.size() << std::endl;
  //std::cout << "selected_min_z" << selected_min_z << std::endl;

  for (size_t i = 0; i < g_cloud_ptr->points.size(); i++)
  {
    Eigen::Vector3i rgb = g_cloud_ptr->points[i].getRGBVector3i();
    pcl::PointXYZRGB p(rgb(0), rgb(1), rgb(2));

    p.getVector3fMap() = g_cloud_ptr->points[i].getVector3fMap();
    Vector3f xyz = p.getVector3fMap();

    // std::cout << "DOT PRODUCT " << i << ": " << xyz.dot(g_plane_normal) << std::endl;
    //if (xyz(2) <= selected_min_z+0.05)
      //std::cout << xyz.dot(g_plane_normal) << std::endl;
    //xyz.dot(g_plane_normal) <= COPLANAR_TOLERANCE &&
    if (xyz(2) <= selected_min_z+0.07)
    {
      //std::cout << g_cloud_ptr->points[i].getRGBVector3i() << std::endl;

      //p.getRGBVector3i() =g_cloud_ptr->points[i].getRGBVector3i();

      //std::cout << p.getRGBVector3i() << std::endl;

      view_cloud.points.push_back(p);
    }
      
    
      //ROS_INFO("do nothing");
  }
  std::cout << "view cloud size: " << view_cloud.size() << std::endl;
  // Now that the coplanar points are inside the view cloud
  // put the above pts in, too.

  // pcl::PointCloud<pcl::PointXYZRGB> above_cloud;
  // for (size_t i = 0; i < g_cloud_ptr->points.size(); i++)
  // {
  //   pcl::PointXYZRGB p;
  //   p.getVector3fMap() = g_cloud_ptr->points[i].getVector3fMap();

  //   for (size_t j = 0; j < view_cloud.size(); j++)
  //   {
  //     if (pointXYinTolerance(p, view_cloud.points[j]))
  //       above_cloud.push_back(p);
  //   }
  // }

  // for (size_t i = 0; i < above_cloud.size(); i++)
  // {
  //   PointXYZRGB p = above_cloud.points[i];
  //   view_cloud.points.push_back(p);
  // }
  // TODO(lucbettaieb): Do I need to delete above_cloud and other stuff?
  return view_cloud;
}

bool PlanarPublisher::pointXYinTolerance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2)
{
  Vector3f p1_vec = p1.getVector3fMap();
  Vector3f p2_vec = p2.getVector3fMap();
  if (std::fabs(p1_vec(0) - p2_vec(0)) <= PLANAR_TOLERANCE &&
      std::fabs(p1_vec(1) - p2_vec(1)) <= PLANAR_TOLERANCE)
    return true;
  else
    return false;
}

bool PlanarPublisher::ppOK()
{
  return selected_pts_cb_bool && got_cloud;
}
void PlanarPublisher::ppNOK()
{
  selected_pts_cb_bool = false;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planar_pointcloud_publisher");
  ros::NodeHandle nh;

  PlanarPublisher pp(nh);

  pcl::PointCloud<pcl::PointXYZRGB> view_cloud;
  sensor_msgs::PointCloud2 ros_view_cloud;

  ros::Publisher view_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/view_cloud", 1, true);

  while (ros::ok())
  {
    ros::spinOnce();

    // Get the selected points and find their plane_normal
    // Find all points within the g_cloud_ptr which are coplanar with plane_normal
    if (pp.ppOK())
    {
      // probs should make this a pass by reference fcn
      view_cloud = pp.find_coplanar_above_points();
      std::cout << "view cloud size: " << view_cloud.points.size() << std::endl;
      //pp.ppNOK();
    }

    pcl::toROSMsg(view_cloud, ros_view_cloud);
    ros_view_cloud.header.stamp = ros::Time::now();
    ros_view_cloud.header.frame_id = pp.getPCFrame();  // TODO(lucbettaieb): probably should get this from the in cloud


    view_cloud_pub.publish(ros_view_cloud);

    ros::spinOnce();
  }
  return 1;
}
