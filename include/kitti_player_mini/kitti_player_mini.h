#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Geometry>
//#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class KittiPlayerMini{
  public:
    KittiPlayerMini(ros::NodeHandle& nh) : nh_(nh) {
      left_color_image_publisher_ = nh_.advertise<sensor_msgs::Image>("left_color_image", 1);
      depth_image_publisher_ = nh_.advertise<sensor_msgs::Image>("depth_image", 1);
      velodyne_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("velodyne", 1);
      left_camera_pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("left_camera_pose", 1);
    }

    void process_scan() {
      int scan_num = 100;
      
      std::string velodyne_dir = "/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/sequences/05/velodyne/";
      std::string left_color_image_dir = "/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/sequences/05/image_2/";
      std::string depth_image_dir = "/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/sequences/05/depth_img/";
      for (int scan_id = 0; scan_id <= scan_num; ++scan_id) {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        std::string velodyne_name = velodyne_dir + std::string(scan_id_c) + ".bin";
        velodyne_ = read_velodyne(velodyne_name);

        std::string left_color_image_name = left_color_image_dir + std::string(scan_id_c) + ".png";
        std::string depth_image_name = depth_image_dir + std::string(scan_id_c) + ".png";

        Eigen::Matrix4d left_camera_pose = left_camera_poses_[scan_id];
        Eigen::Matrix4d init_trans_to_ground;
        init_trans_to_ground <<  1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0,-1, 0, 1,
                                 0, 0, 0, 1;
        left_camera_pose = init_trans_to_ground * left_camera_pose;

        ros::Time t_cur = ros::Time::now();
        
        // publish tf
        tf::Transform transform;
        Eigen::Affine3d eigen_affine_pose(left_camera_pose);
        tf::transformEigenToTF(eigen_affine_pose, transform);
        br_.sendTransform(tf::StampedTransform(transform, t_cur, "map", "robot"));
        Eigen::Matrix4d Tr;
        Tr << -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
              -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
               0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
       	       0                ,  0                ,  0                ,  1.000000000000000;
        Eigen::Affine3d eigen_affine_Tr(Tr);
        tf::transformEigenToTF(eigen_affine_Tr, transform);
        br_.sendTransform(tf::StampedTransform(transform, t_cur, "robot", "sensor"));


        // publish pose

        // publish velodyne
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*velodyne_, cloud_msg);
        cloud_msg.header.frame_id = "sensor";
        cloud_msg.header.stamp = t_cur;
        velodyne_publisher_.publish(cloud_msg);
        publish_posecov(eigen_affine_pose, t_cur);

        // publish images
        cv::Mat depth_image = cv::imread(depth_image_name, CV_LOAD_IMAGE_ANYDEPTH);
        cv_bridge::CvImage depth_msg;
        depth_msg.header.stamp = t_cur;
        depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        depth_msg.image = depth_image;

        cv::Mat left_color_image = cv::imread(left_color_image_name, CV_LOAD_IMAGE_UNCHANGED);
        cv_bridge::CvImage left_color_msg;
        left_color_msg.header.stamp = t_cur;
        left_color_msg.encoding = sensor_msgs::image_encodings::RGB8;
        left_color_msg.image = left_color_image;
        left_color_image_publisher_.publish(left_color_msg);
        depth_image_publisher_.publish(depth_msg);
        
        ros::Duration(3.5).sleep();
      
      }
    }

    void publish_posecov(Eigen::Affine3d pose, ros::Time t)
{
//    geometry_msgs::Pose pose_msg;
//    tf::poseEigenToMsg(pose, pose_msg);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "robot";
    pose_msg.header.stamp = t;

    // set pose
    tf::poseEigenToMsg(pose, pose_msg.pose.pose);

    // set cov
    pose_msg.pose.covariance[6*0+0] = 0.0;
    pose_msg.pose.covariance[6*0+1] = 0.0;
    pose_msg.pose.covariance[6*0+2] = 0.0;
    pose_msg.pose.covariance[6*0+3] = 0.0;
    pose_msg.pose.covariance[6*0+4] = 0.0;
    pose_msg.pose.covariance[6*0+5] = 0.0;
    pose_msg.pose.covariance[6*1+0] = 0.0;
    pose_msg.pose.covariance[6*1+1] = 0.0;
    pose_msg.pose.covariance[6*1+2] = 0.0;
    pose_msg.pose.covariance[6*1+3] = 0.0;
    pose_msg.pose.covariance[6*1+4] = 0.0;
    pose_msg.pose.covariance[6*1+5] = 0.0;
    pose_msg.pose.covariance[6*2+0] = 0.0;
    pose_msg.pose.covariance[6*2+1] = 0.0;
    pose_msg.pose.covariance[6*2+2] = 0.0;
    pose_msg.pose.covariance[6*2+3] = 0.0;
    pose_msg.pose.covariance[6*2+4] = 0.0;
    pose_msg.pose.covariance[6*2+5] = 0.0;
    pose_msg.pose.covariance[6*3+0] = 0.0;
    pose_msg.pose.covariance[6*3+1] = 0.0;
    pose_msg.pose.covariance[6*3+2] = 0.0;
    pose_msg.pose.covariance[6*3+3] = 0.0;
    pose_msg.pose.covariance[6*3+4] = 0.0;
    pose_msg.pose.covariance[6*3+5] = 0.0;
    pose_msg.pose.covariance[6*4+0] = 0.0;
    pose_msg.pose.covariance[6*4+1] = 0.0;
    pose_msg.pose.covariance[6*4+2] = 0.0;
    pose_msg.pose.covariance[6*4+3] = 0.0;
    pose_msg.pose.covariance[6*4+4] = 0.0;
    pose_msg.pose.covariance[6*4+5] = 0.0;
    pose_msg.pose.covariance[6*5+0] = 0.0;
    pose_msg.pose.covariance[6*5+1] = 0.0;
    pose_msg.pose.covariance[6*5+2] = 0.0;
    pose_msg.pose.covariance[6*5+3] = 0.0;
    pose_msg.pose.covariance[6*5+4] = 0.0;
    pose_msg.pose.covariance[6*5+5] = 0.0;

    // publish
    left_camera_pose_publisher_.publish(pose_msg);
}
        
    bool read_left_camera_poses(const std::string pose_name) {
      if (std::ifstream(pose_name)) {
        std::ifstream fPoses;
        fPoses.open(pose_name.c_str());
        while (!fPoses.eof()) {
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 3; ++i)
              for (int j = 0; j < 4; ++j)
                ss >> t_matrix(i, j);
            left_camera_poses_.push_back(t_matrix);
          }
        }
        fPoses.close();
        return true;
        } else {
         ROS_ERROR_STREAM("Cannot open pose file " << pose_name);
         return false;
      }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr read_velodyne(std::string fn) {
      FILE* fp = std::fopen(fn.c_str(), "r");
      if (!fp) {
        std::perror("File opening failed");
      }
      std::fseek(fp, 0L, SEEK_END);
      size_t sz = std::ftell(fp);
      std::rewind(fp);
      int n_hits = sz / (sizeof(float) * 4);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
      for (int i = 0; i < n_hits; i++) {
        pcl::PointXYZI point;
        float intensity;
        if (fread(&point.x, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.y, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.z, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.intensity, sizeof(float), 1, fp) == 0) break;
        pc->push_back(point);
      }
      std::fclose(fp);
      return pc;
    }

  private:
    cv::Mat left_color_image_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_;
    std::vector<Eigen::Matrix4d> left_camera_poses_;

    ros::NodeHandle nh_;
    ros::Publisher left_color_image_publisher_;
    ros::Publisher depth_image_publisher_;
    ros::Publisher velodyne_publisher_;
    ros::Publisher left_camera_pose_publisher_;
    tf::TransformBroadcaster br_;
};
