#include "kitti_player_mini.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "kitti_player_mini_node");
  ros::NodeHandle nh("~");

  KittiPlayerMini kitti_player_mini(nh);
  kitti_player_mini.read_left_camera_poses("/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/poses/05.txt");
  kitti_player_mini.process_scan();
  ros::spin();

  return 0;
}
