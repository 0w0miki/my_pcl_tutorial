#ifndef MERGE_HPP
#define MERGE_HPP


#include <ros/ros.h>
// PCL specific includes
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "third_party/sophus/so3.hpp"

void setPCA (const sensor_msgs::PointCloud2ConstPtr& input);
void setPCB (const sensor_msgs::PointCloud2ConstPtr& input);
void MergePC(pcl::PointCloud<pcl::PointXYZ> pc_a,pcl::PointCloud<pcl::PointXYZ> pc_b);

class pc_merge
{
public:
	pc_merge();
	~pc_merge();
private:
	pcl::PointCloud<pcl::PointXYZ> pc_a;
	pcl::PointCloud<pcl::PointXYZ> pc_b;
	pcl::PointCloud<pcl::PointXYZ> merged_pc;
};


#endif