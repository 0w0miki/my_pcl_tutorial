#include "my_pcl_tutorial/example.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
Eigen::Vector3f pos=Eigen::Vector3f(0,0,0);
Eigen::Quaternionf rotq=Eigen::Quaternionf(0,0,0,1);
float linear = 0.5;
float angular = 0;
float dist_thresh = 20;                 // threshold of distance, out of the range points won't be considered

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // get input point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  ROS_INFO_STREAM("got input PointCloud");

  const int width = cloud.width;
  const int height = cloud.height;
  float dist = 0;
  float dist_sum = 0;
  float theta = 0;
  float dir = 0;
  Eigen::Vector3f z_norm=Eigen::Vector3f(1,0,0);
  Eigen::Vector3f att=rotq.toRotationMatrix()*z_norm;
  Eigen::Vector3f right_n = Eigen::Vector3f(0,1,0);
  Eigen::Vector3f pt=Eigen::Vector3f(0,0,0);
  Eigen::Vector3f nvec=Eigen::Vector3f(0,0,0);

  ROS_INFO_STREAM("norm of vector:" << pt.cross(att));
  for (int i = 0; i < width; ++i)
  {
    for (int j = 0; j < height; ++j)
    {
      pt = Eigen::Vector3f(cloud.points[i+j].x,cloud.points[i+j].y,cloud.points[i+j].z)-pos;
      dist = pt.norm();//get length
      if (dist<dist_thresh)
      {
        // right or left
        if(pt.transpose()*right_n>0){
            //  right
            nvec = att.cross(pt);//get cross product  
         }else{
            // left
            nvec = pt.cross(att);//get cross product  
         }
         theta = nvec.transpose() * att;
         theta = theta / att.norm() / nvec.norm();
         theta = acos(theta);
         if (nvec.transpose()*right_n>0){
            // right
            dir += theta / dist ;
         }else{
            // left
            dir += (2*M_PI-theta) / dist ;
         }
        dist_sum += 1/dist;
      }else{
        continue;
      }
    }
  }
  if (dist_sum != 0)
  {
    dir /= dist_sum;
  }
  Eigen::Matrix3f rot_dir_Mat = Eigen::Matrix3f::Identity();
  rot_dir_Mat (1,1) = cos(-dir);
  rot_dir_Mat (1,2) = -sin(-dir);
  rot_dir_Mat (2,1) = sin(-dir);
  rot_dir_Mat (2,2) = cos(-dir);
  Eigen::Matrix3f head2x_Mat = Eigen::Matrix3f::Identity();
  head2x_Mat (0,0) = cos(M_PI/2);
  head2x_Mat (0,2) = -sin(M_PI/2);
  head2x_Mat (2,0) = sin(M_PI/2);
  head2x_Mat (2,2) = cos(M_PI/2);
  ROS_INFO_STREAM("dir:"<<dir);
  Eigen::Quaternionf quat_dir(rot_dir_Mat * head2x_Mat);
  linear = 0.5 * (cos(dir) + 1) + 0.5;
  angular = (dir-M_PI)>0 ? dir-M_PI : -dir;

  // twist_pub_.publish(twist);

  // Publish the data.
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "/frame";
  pose.header.stamp = ros::Time::now();
  
  pose.pose.position.x = pos[0];
  pose.pose.position.y = pos[1];
  pose.pose.position.z = pos[2];

  float quat_dir_length = quat_dir.norm();
  pose.pose.orientation.x = quat_dir.x()/quat_dir_length;
  pose.pose.orientation.y = quat_dir.y()/quat_dir_length;
  pose.pose.orientation.z = quat_dir.z()/quat_dir_length;
  pose.pose.orientation.w = quat_dir.w()/quat_dir_length;
  pub.publish (pose);
}

void 
set_pose (const geometry_msgs::PoseStamped& poseMsg)
{
  pos[0] = poseMsg.pose.position.x;
  pos[1] = poseMsg.pose.position.y;
  pos[2] = poseMsg.pose.position.z;
  rotq = Eigen::Quaternion<float>(poseMsg.pose.orientation.w,poseMsg.pose.orientation.x,poseMsg.pose.orientation.y,poseMsg.pose.orientation.z);
}

int
main (int argc, char** argv)
{
  float scale_linear = 0.5;
  float scale_angular = 0.2;
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  
  ros::Subscriber pc_sub = nh.subscribe ("input", 100, cloud_cb);
  ros::Subscriber pose_sub = nh.subscribe ("pose", 10, set_pose);

  // Create a ROS publisher for the cmd_vel
  ros::Publisher twist_pub_;

  pub = nh.advertise<geometry_msgs::PoseStamped> ("dir_pose", 5);
  twist_pub_ = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 100);
  ros::param::get("~scale_linear",scale_linear);
  ros::param::get("~scale_angular",scale_angular);
  ros::param::get("~dist_thresh",dist_thresh);
  ROS_INFO_STREAM("scale_linear:"<<scale_linear);
  ROS_INFO_STREAM("scale_angular:"<<scale_angular);
  ros::Rate loop_rate(10); //循环频率控制对象，控制while循环一秒循环10次
  while (ros::ok()) //ros::ok()只有当用户按下Crtl+C按键时，才会返回false，循环终止
  {
    geometry_msgs::Twist twist;
    twist.linear.x = linear * scale_linear;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular * scale_angular;
    twist_pub_.publish(twist);
    ros::spinOnce(); // 执行这条函数的意义在于，让主线程暂停，处理来自订阅者的请求
    loop_rate.sleep(); // 执行线程休眠
  }
}