#include "my_pcl_tutorial/merge.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ> pc_a;
pcl::PointCloud<pcl::PointXYZ> pc_b;

void setPCA (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Create a container for the data.
	float dist_thresh = 20;

	// Do data processing here...
	// pcl::PointCloud<pcl::PointXYZ> pc_a;
	pcl::fromROSMsg (*input, pc_a);

	ROS_INFO_STREAM("got input PointCloud A");

	MergePC(pc_a,pc_b);
}

void setPCB (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Create a container for the data.
	float dist_thresh = 20;

	// Do data processing here...
	// pcl::PointCloud<pcl::PointXYZ> pc_a;
	pcl::fromROSMsg (*input, pc_b);

	ROS_INFO_STREAM("got input PointCloud B");

	MergePC(pc_a,pc_b);
}

void MergePC(pcl::PointCloud<pcl::PointXYZ> pc_a,pcl::PointCloud<pcl::PointXYZ> pc_b){

	//
	pcl::PointCloud<pcl::PointXYZ> merged_pc;

	Eigen::Matrix4f tfMat = Eigen::Matrix4f::Identity();

	/* Reminder: how transformation matrices work :
           	|-------> This column is the translation
	    | 1 0 0 x |  \
	    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
	    | 0 0 1 z |  /
	    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
	*/
	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	// Here we defined a 45° (PI/4) rotation around the Z axis and a translation on the X axis.
	float theta = M_PI/4; // The angle of rotation in radians
	tfMat (0,0) = cos (theta);
	tfMat (0,1) = -sin(theta);
	tfMat (1,0) = sin (theta);
	tfMat (1,1) = cos (theta);
	//    (row, column)

	// Define a translation of 2.5 meters on the x axis.
	tfMat (0,3) = 2.5;

	pcl::transformPointCloud (pc_b, merged_pc, tfMat);
	/*
	void pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in, 
	                                pcl::PointCloud< PointT > &  cloud_out,  
	                                const Eigen::Matrix4f &  transform  ) 
	*/
	merged_pc += pc_a;
	ROS_INFO_STREAM("Merged");

	// 转换为ROS 下的数据格式发布出去
	sensor_msgs::PointCloud2 output;   //声明的输出的点云的格式
	pcl::toROSMsg(merged_pc,output);
	output.header.frame_id = "frame";
	pub.publish(output);
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber pc_sub = nh.subscribe ("pc_a", 10, setPCA);
	ros::Subscriber pose_sub = nh.subscribe ("pc_b", 10, setPCB);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("merged_pc", 10);

	// Spin
	ros::spin ();
}