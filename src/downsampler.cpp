#include <downsampler/downsampler.h>
#include <pluginlib/class_list_macros.h>

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace downsampler
{

void Downsampler::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();

  sub_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("input_cloud", 1, &Downsampler::downsample_cloud_cb, this);
  pub_downsampled_cloud_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("downsampled_points", 1);

  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  private_nh.param("min_range", min_range_, 0.0);
  private_nh.param("max_range", max_range_, 100.0);
  private_nh.param("leaf_size", leaf_size_, 0.03);
  private_nh.param("filter_radius", filter_radius_, 0.03);
  private_nh.param("min_points_threshold", min_points_threshold_, 3);

  double rate;
  private_nh.param("rate", rate, 30.0);

  if (rate == 0)
    interval_ = ros::Duration(0);
  else
    interval_ = ros::Duration(1.0 / rate);
  next_call_time_ = ros::Time::now();

  ROS_INFO_STREAM("Downsampling points using a leaf size of '" << leaf_size_ << "' m, running at " << rate << " Hz.");
}

void Downsampler::downsample_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (ros::Time::now() <= next_call_time_)
    return;
  next_call_time_ = next_call_time_ + interval_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  pcl::fromROSMsg(*cloud_msg, input_cloud);

  if (max_range_ != 0.0)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_range_, max_range_);
    pass.filter(*cut_cloud);
  }
  else
  {
    cut_cloud = input_cloud.makeShared();
  }

  if (leaf_size_ == 0.0)
  {
    sensor_msgs::PointCloud2Ptr downsampled(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cut_cloud, *downsampled);

    pub_downsampled_cloud_.publish(downsampled);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if (leaf_size_ != 0.0)
  {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cut_cloud);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*downsampled_cloud);
  }
  else
  {
    downsampled_cloud = cut_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if(min_points_threshold_ > 0 && filter_radius_ > 0.0)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_removal;
    outlier_removal.setInputCloud(downsampled_cloud);
    outlier_removal.setRadiusSearch(0.03);
    outlier_removal.setMinNeighborsInRadius(3);
    outlier_removal.filter(*filtered_cloud);
  }
  else
  {
    filtered_cloud = downsampled_cloud;
  }

  sensor_msgs::PointCloud2Ptr result(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*filtered_cloud, *result);

  pub_downsampled_cloud_.publish(result);
}

} //end namespace

PLUGINLIB_EXPORT_CLASS(downsampler::Downsampler, nodelet::Nodelet)
