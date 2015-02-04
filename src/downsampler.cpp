#include <downsampler/downsampler.h>
#include <pluginlib/class_list_macros.h>

#include <boost/shared_ptr.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace downsampler
{

void Downsampler::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();

  sub_cloud_ = nh.subscribe < sensor_msgs::PointCloud2 > ("input_cloud", 1, &Downsampler::downsample_cloud_cb, this);
  pub_downsampled_cloud_ = nh.advertise < sensor_msgs::PointCloud2 > ("downsampled_points", 1);

  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  private_nh.param("min_range", min_range_, 0.0);
  private_nh.param("max_range", max_range_, 100.0);
  private_nh.param("leaf_size", leaf_size_, 0.03);

  double rate;
  private_nh.param("rate", rate, 30.0);

  if(rate == 0)
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

  boost::shared_ptr < pcl::PCLPointCloud2 > cloud(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  boost::shared_ptr < pcl::PCLPointCloud2 > filteredCloud(new pcl::PCLPointCloud2);

  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PassThrough < pcl::PCLPointCloud2 > pass;
  pass.setInputCloud(cloudPtr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_range_, max_range_);
  pass.filter(*filteredCloud);
  cloudPtr = filteredCloud;

  if (leaf_size_ != 0)
  {
    pcl::VoxelGrid < pcl::PCLPointCloud2 > sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud);
  }

  sensor_msgs::PointCloud2 downsampled;
  pcl_conversions::moveFromPCL(*cloud, downsampled);
  pub_downsampled_cloud_.publish(downsampled);
}

} //end namespace

PLUGINLIB_EXPORT_CLASS(downsampler::Downsampler, nodelet::Nodelet)
