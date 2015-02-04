#ifndef DOWNSAMPLER_DOWNSAMPLER_H_
#define DOWNSAMPLER_DOWNSAMPLER_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>

namespace downsampler
{

class Downsampler : public nodelet::Nodelet
{
public:
  virtual void onInit();
  virtual void downsample_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

protected:
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_downsampled_cloud_;

  double min_range_;
  double max_range_;
  double leaf_size_;

  ros::Duration interval_;
  ros::Time next_call_time_;

};

} //end namespace

#endif /* DOWNSAMPLER_DOWNSAMPLER_H_ */
