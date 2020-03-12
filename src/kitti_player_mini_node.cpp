#include "kitti_player_mini.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "kitti_player_mini_node");
  ros::NodeHandle nh("~");

  int scan_num = 1100;
  std::string sequence_dir = "/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/sequences/07/";

  KittiPlayerMini kitti_player_mini(nh);
  kitti_player_mini.read_left_camera_poses("/media/ganlu/PERL-SSD/Datasets/KITTI/dataset/poses/07.txt");
  kitti_player_mini.process_scans(sequence_dir, scan_num);
  ros::spin();

  return 0;
}
