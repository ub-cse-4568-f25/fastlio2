// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <math.h>
#include <unistd.h>

#include <deque>

#include <Eigen/Core>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "ikd_Tree.h"
#include "imu_processing.hpp"
#include "preprocess.h"

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
#include <livox_ros_driver/CustomMsg.h>
#endif

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN],
    s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true,
     path_en                  = true;
bool enable_gravity_alignment = false;
bool is_gravity_aligned       = false;
/**************************/

float res_last[100000]    = {0.0};
float DET_RANGE           = 300.0f;
const float MOV_THRESHOLD = 1.5f;

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

std::string root_dir = ROOT_DIR;
std::string sequence_name, save_dir;
std::string map_file_path, map_frame, lidar_frame, base_frame, imu_frame, visualization_frame;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_map_smaller = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0,
       first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0,
    pcd_save_interval = -1, pcd_index = 0;
int point_filter_num         = 4;  // empirically, 4 showed the best performance
int feats_down_size_neighbor = numeric_limits<int>::max();
// Empirically, the acceleration difference in mobile robots is usually over 0.35.
double acc_diff_thr              = 0.2;
int num_moving_frames_thr        = 10;
int num_gravity_measurements_thr = 10;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_lidar_pub_en = false, scan_body_pub_en = false,
     scan_base_pub_en = false;

std::deque<V3D> local_gravity_directions;
std::deque<V3D> global_gravity_directions;

std::vector<vector<int>> pointSearchInd_surf;
std::vector<BoxPointType> cub_needrm;
std::vector<PointVector> Nearest_Points;
std::vector<double> g_base_vec{0.0, 0.0, -1.0};
std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
std::deque<double> time_buffer;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;

std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr cloud_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D g_base(Zero3d);
V3D mean_acc_stopped(Zero3d);
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
M3D R_gravity_aligned(Eye3d);
/*** Only used for integration with the Hydra system ***/
V3D Lidar_T_wrt_Base(Zero3d);
M3D Lidar_R_wrt_Base(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

std::string output_pose_file = "/home/shapelim/fastlio_original_poses.txt";

void SigHandle(int sig) {
  flg_exit = true;
  ROS_DEBUG("catch sig %d", sig);
  sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(std::ofstream &out) {
  V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
  out << Measures.lidar_beg_time - first_lidar_time << " ";
  out << rot_ang(0) << " " << rot_ang(1) << " " << rot_ang(2) << " ";                   // Angle
  out << state_point.pos(0) << " " << state_point.pos(1) << " " << state_point.pos(2);  // Pos
  out << 0.0 << " " << 0.0 << " " << 0.0 << " ";
  out << state_point.vel(0) << " " << state_point.vel(1) << " " << state_point.vel(2)
      << " ";  // Vel
  out << 0.0 << " " << 0.0 << " " << 0.0 << " ";
  out << state_point.bg(0) << " " << state_point.bg(1) << " " << state_point.bg(2)
      << " ";  // Bias_g
  out << state_point.ba(0) << " " << state_point.ba(1) << " " << state_point.ba(2)
      << " ";                                                                              // Bias_a
  out << state_point.grav[0] << " " << state_point.grav[1] << " " << state_point.grav[2];  // Bias_a
  out << std::endl;
}

// Outputs rotation matrix that aligns a to b, i.e., R such that R * g_a = g_b
M3D computeRelativeRotation(const Eigen::Vector3d &g_a, const Eigen::Vector3d &g_b) {
  Eigen::Vector3d g_a_norm = g_a.normalized();
  Eigen::Vector3d g_b_norm = g_b.normalized();

  Eigen::Vector3d axis = g_a_norm.cross(g_b_norm);
  double cos_theta     = g_a_norm.dot(g_b_norm);

  if (std::fabs(1.0 - cos_theta) < 1e-3) {
    return Eigen::Matrix3d::Identity();
  }

  // Degenerate condition a = -b
  // Compute cross product with any arbitrary nonparallel vector,
  // i.e., Eigen::Vector3d(1, 2, 3)
  if (std::fabs(1.0 + cos_theta) < 1e-3) {
    Eigen::Vector3d perturbed = g_a_norm + Eigen::Vector3d(1, 2, 3);
    axis                      = g_a_norm.cross(perturbed);

    if (axis.norm() < 1e-6) {
      perturbed = g_a_norm + Eigen::Vector3d(3, 2, 1);
      axis      = g_a_norm.cross(perturbed);
    }

    axis.normalize();
    return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
  } else {
    axis.normalize();
    double theta = std::acos(cos_theta);

    Eigen::Quaterniond q(Eigen::AngleAxisd(theta, axis));

    return q.toRotationMatrix();
  }
}

void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

  po->x         = p_global(0);
  po->y         = p_global(1);
  po->z         = p_global(2);
  po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
               state_point.pos);

  po->x         = p_global(0);
  po->y         = p_global(1);
  po->z         = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
               state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void pclPointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
               state_point.pos);

  po->x         = p_global(0);
  po->y         = p_global(1);
  po->z         = p_global(2);
  po->intensity = pi->intensity;
}

void pclPointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

  po->x         = p_body_imu(0);
  po->y         = p_body_imu(1);
  po->z         = p_body_imu(2);
  po->intensity = pi->intensity;
}

void pclPointBodyLidarToBase(PointType const *const pi, PointType *const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_base(Lidar_R_wrt_Base * p_body_lidar + Lidar_T_wrt_Base);

  po->x         = p_body_base(0);
  po->y         = p_body_base(1);
  po->z         = p_body_base(2);
  po->intensity = pi->intensity;
}

void pclPointIMUToLiDAR(PointType const *const pi, PointType *const po) {
  V3D p_body_imu(pi->x, pi->y, pi->z);
  V3D p_body_lidar(state_point.offset_R_L_I.inverse() * (p_body_imu - state_point.offset_T_L_I));

  po->x         = p_body_lidar(0);
  po->y         = p_body_lidar(1);
  po->z         = p_body_lidar(2);
  po->intensity = pi->intensity;
}

void pclPointIMUToBase(PointType const *const pi, PointType *const po) {
  static const auto &offset_R_B_I = state_point.offset_R_L_I * Lidar_R_wrt_Base.inverse();
  static const auto &offset_T_B_I = -1 * offset_R_B_I * Lidar_T_wrt_Base + state_point.offset_T_L_I;

  V3D p_body_imu(pi->x, pi->y, pi->z);
  V3D p_body_base(offset_R_B_I.inverse() * (p_body_imu - offset_T_B_I));

  po->x         = p_body_base(0);
  po->y         = p_body_base(1);
  po->z         = p_body_base(2);
  po->intensity = pi->intensity;
}

void points_cache_collect() {
  PointVector points_history;
  ikdtree.acquire_removed_points(points_history);
  // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment() {
  cub_needrm.clear();
  kdtree_delete_counter = 0;
  kdtree_delete_time    = 0.0;
  pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
  V3D pos_LiD = pos_lid;
  if (!Localmap_Initialized) {
    for (int i = 0; i < 3; i++) {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }
  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      need_move = true;
  }
  if (!need_move) return;
  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  float mov_dist      = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                       static_cast<double>(DET_RANGE * (MOV_THRESHOLD - 1)));
  for (int i = 0; i < 3; i++) {
    tmp_boxpoints = LocalMap_Points;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  LocalMap_Points = New_LocalMap_Points;

  points_cache_collect();
  double delete_begin = omp_get_wtime();
  if (cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
  kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  mtx_buffer.lock();
  scan_count++;
  double preprocess_start_time = omp_get_wtime();
  if (msg->header.stamp.toSec() < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar = msg->header.stamp.toSec();
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg         = false;

#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  mtx_buffer.lock();
  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  if (msg->header.stamp.toSec() < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  last_timestamp_lidar = msg->header.stamp.toSec();

  if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 &&
      !imu_buffer.empty() && !lidar_buffer.empty()) {
    ROS_WARN_STREAM("IMU and LiDAR not Synced, IMU time: "
                    << last_timestamp_imu << ", lidar header time: " << last_timestamp_lidar);
  }

  if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
      !imu_buffer.empty()) {
    timediff_set_flg       = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    ROS_INFO("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(last_timestamp_lidar);

  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}
#endif

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  publish_count++;
  // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
    msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
  }

  double timestamp = msg->header.stamp.toSec();

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num               = 0;
bool sync_packages(MeasureGroup &meas, bool verbose) {
  if (verbose) {
    ROS_INFO_STREAM(lidar_buffer.size() << " vs " << imu_buffer.size());
  }

  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed) {
    meas.lidar                = lidar_buffer.front();
    meas.lidar_beg_time       = time_buffer.front();
    static double denominator = 1000;
    // time too little
    if (meas.lidar->points.size() <= 1) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      ROS_WARN("Too few input point cloud!\n");
    } else if (meas.lidar->points.back().curvature / denominator < 0.5 * lidar_mean_scantime) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      if (meas.lidar->points.back().curvature < 80 || meas.lidar->points.back().curvature > 120) {
        ROS_WARN_STREAM("meas.lidar->points.back().curvature ("
                        << meas.lidar->points.back().curvature
                        << ") should be close to 100) Please check the `timestamp_unit`"
                        << " or values of `time` (or `t`) field of the point cloud input"
                        << " from your sensor");
      }

      lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / denominator;
      lidar_mean_scantime +=
          (meas.lidar->points.back().curvature / denominator - lidar_mean_scantime) / scan_num;
    }

    meas.lidar_end_time = lidar_end_time;

    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    if (verbose) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(9) << last_timestamp_imu << " vs " << lidar_end_time
         << ". Timestamp is not matched. Discarded";
      ROS_INFO_STREAM(ss.str());
    }
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer.front()->header.stamp.toSec();
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    imu_time = imu_buffer.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time) break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

int process_increments = 0;
void map_incremental() {
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);
  for (int i = 0; i < feats_down_size; i++) {
    /* transform to world frame */
    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    /* decide if need add to map */
    if (!Nearest_Points[i].empty() && flg_EKF_inited) {
      const PointVector &points_near = Nearest_Points[i];
      bool need_add                  = true;
      PointType downsample_result, mid_point;
      mid_point.x =
          floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
          0.5 * filter_size_map_min;
      mid_point.y =
          floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
          0.5 * filter_size_map_min;
      mid_point.z =
          floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
          0.5 * filter_size_map_min;
      float dist = calc_dist(feats_down_world->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
    } else {
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  double st_time = omp_get_wtime();
  add_point_size = ikdtree.Add_Points(PointToAdd, true);
  ikdtree.Add_Points(PointNoNeedDownsample, false);
  add_point_size          = PointToAdd.size() + PointNoNeedDownsample.size();
  kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher &pubLaserCloudFull) {
  if (scan_pub_en) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? cloud_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    PointCloudXYZI::Ptr laserCloudTmp(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++) {
      if (visualization_frame == "imu") {
        pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
      } else if (visualization_frame == "lidar") {
        pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudTmp->points[i]);
        pclPointIMUToLiDAR(&laserCloudTmp->points[i], &laserCloudWorld->points[i]);
      } else if (visualization_frame == "base") {
        pclPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudTmp->points[i]);
        pclPointIMUToBase(&laserCloudTmp->points[i], &laserCloudWorld->points[i]);
      } else {
        throw invalid_argument("Invalid visualization frame has been given");
      }
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = map_frame;
    pubLaserCloudFull.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
   * 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en) {
    int size = cloud_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      pclPointBodyToWorld(&cloud_undistort->points[i], &laserCloudWorld->points[i]);
    }
    *pcl_wait_save += *laserCloudWorld;

    static int scan_wait_num = 0;
    scan_wait_num++;
    if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) +
                            string(".pcd"));
      pcl::PCDWriter pcd_writer;
      cout << "current scan saved to /PCD/" << all_points_dir << endl;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
      pcl_wait_save->clear();
      scan_wait_num = 0;
    }
  }
}

void publish_frame(const ros::Publisher &pubLaserCloudFull, const std::string viz_frame = "imu") {
  int size = cloud_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudTransformed(new PointCloudXYZI(size, 1));
  sensor_msgs::PointCloud2 laserCloudmsg;
  if (viz_frame == "lidar") {
    for (int i = 0; i < size; i++) {
      laserCloudTransformed->points[i] = cloud_undistort->points[i];
    }
    pcl::toROSMsg(*laserCloudTransformed, laserCloudmsg);
    laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = lidar_frame;
  } else if (viz_frame == "imu") {
    for (int i = 0; i < size; i++) {
      pclPointBodyLidarToIMU(&cloud_undistort->points[i], &laserCloudTransformed->points[i]);
    }
    pcl::toROSMsg(*laserCloudTransformed, laserCloudmsg);
    laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = imu_frame;
  } else if (viz_frame == "base") {
    for (int i = 0; i < size; i++) {
      pclPointBodyLidarToBase(&cloud_undistort->points[i], &laserCloudTransformed->points[i]);
    }
    pcl::toROSMsg(*laserCloudTransformed, laserCloudmsg);
    laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = base_frame;
  } else {
    throw std::invalid_argument("Invalid frame has been given");
  }

  pubLaserCloudFull.publish(laserCloudmsg);
  publish_count -= PUBFRAME_PERIOD;
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> transform_pose_wrt_lidar_frame() {
  // offset_A_B: transformation matrix of A w.r.t. B
  Eigen::Vector3d lidar_position =
      state_point.offset_R_L_I.inverse() *
      (state_point.rot * state_point.offset_T_L_I + state_point.pos - state_point.offset_T_L_I);
  Eigen::Quaterniond lidar_orientation =
      state_point.offset_R_L_I.inverse() * state_point.rot * state_point.offset_R_L_I;

  return std::make_tuple(lidar_position, lidar_orientation);
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> transform_pose_wrt_base_frame() {
  // offset_A_B: transformation matrix of A w.r.t. B
  static const Eigen::Matrix3d offset_R_B_I = state_point.offset_R_L_I * Lidar_R_wrt_Base.inverse();
  static const Eigen::Vector3d offset_T_B_I =
      -offset_R_B_I * Lidar_T_wrt_Base + state_point.offset_T_L_I;

  Eigen::Vector3d base_position =
      offset_R_B_I.inverse() * (state_point.rot * offset_T_B_I + state_point.pos - offset_T_B_I);
  Eigen::Quaterniond base_orientation =
      Eigen::Quaterniond(offset_R_B_I.inverse() * state_point.rot * offset_R_B_I);

  return std::make_tuple(base_position, base_orientation);
}

template <typename T>
void set_posestamp(T &out, const ::string viz_frame = "imu") {
  if (viz_frame == "imu") {
    out.pose.position.x    = state_point.pos(0);
    out.pose.position.y    = state_point.pos(1);
    out.pose.position.z    = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
  } else if (viz_frame == "lidar") {
    const auto &[position, orientation] = transform_pose_wrt_lidar_frame();
    out.pose.position.x                 = position(0);
    out.pose.position.y                 = position(1);
    out.pose.position.z                 = position(2);
    out.pose.orientation.x              = orientation.x();
    out.pose.orientation.y              = orientation.y();
    out.pose.orientation.z              = orientation.z();
    out.pose.orientation.w              = orientation.w();
  } else if (viz_frame == "base") {
    const auto &[position, orientation] = transform_pose_wrt_base_frame();
    out.pose.position.x                 = position(0);
    out.pose.position.y                 = position(1);
    out.pose.position.z                 = position(2);
    out.pose.orientation.x              = orientation.x();
    out.pose.orientation.y              = orientation.y();
    out.pose.orientation.z              = orientation.z();
    out.pose.orientation.w              = orientation.w();
  } else {
    throw invalid_argument("Invalid visualization frame has been given");
  }
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped) {
  odomAftMapped.header.frame_id = map_frame;
  odomAftMapped.header.stamp    = ros::Time().fromSec(lidar_end_time);
  set_posestamp(odomAftMapped.pose, visualization_frame);
  if (visualization_frame == "lidar") {
    odomAftMapped.child_frame_id = lidar_frame;
  } else if (visualization_frame == "base") {
    odomAftMapped.child_frame_id = base_frame;
  } else if (visualization_frame == "imu") {
    odomAftMapped.child_frame_id = imu_frame;
  } else {
    throw invalid_argument("Invalid visualization frame has been given");
  }
  pubOdomAftMapped.publish(odomAftMapped);
  auto P = kf.get_P();
  for (int i = 0; i < 6; i++) {
    int k                                    = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
  }

  /******* Save poses *******/
  const auto &[lidar_position, lidar_orientation] = transform_pose_wrt_lidar_frame();

  if (!output_pose_file.empty()) {
    std::ofstream pose_file;
    pose_file.open(output_pose_file, std::ios::app);
    if (pose_file.is_open()) {
      pose_file << std::fixed << std::setprecision(8) << lidar_end_time << " " << lidar_position(0)
                << " " << lidar_position(1) << " " << lidar_position(2) << " "
                << lidar_orientation.x() << " " << lidar_orientation.y() << " "
                << lidar_orientation.z() << " " << lidar_orientation.w() << std::endl;
      pose_file.close();
    }
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                  odomAftMapped.pose.pose.position.y,
                                  odomAftMapped.pose.pose.position.z));
  q.setW(odomAftMapped.pose.pose.orientation.w);
  q.setX(odomAftMapped.pose.pose.orientation.x);
  q.setY(odomAftMapped.pose.pose.orientation.y);
  q.setZ(odomAftMapped.pose.pose.orientation.z);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(
      transform, odomAftMapped.header.stamp, map_frame, odomAftMapped.child_frame_id));
}

void publish_path(const ros::Publisher pubPath) {
  set_posestamp(msg_body_pose, visualization_frame);
  msg_body_pose.header.stamp    = ros::Time().fromSec(lidar_end_time);
  msg_body_pose.header.frame_id = map_frame;

  /*** if path is too large, the rvis will crash ***/
  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0) {
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
  }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  double match_start = omp_get_wtime();
  laserCloudOri->clear();
  corr_normvect->clear();
  total_residual = 0.0;

  /** closest surface search and residual computation **/
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (int i = 0; i < feats_down_size; i++) {
    PointType &point_body  = feats_down_body->points[i];
    PointType &point_world = feats_down_world->points[i];

    /* transform to world frame */
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    point_world.x         = p_global(0);
    point_world.y         = p_global(1);
    point_world.z         = p_global(2);
    point_world.intensity = point_body.intensity;

    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto &points_near = Nearest_Points[i];

    if (ekfom_data.converge) {
      /** Find the closest surfaces in the map **/
      ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
      point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                               : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                            : true;
    }

    if (!point_selected_surf[i]) continue;

    VF(4) pabcd;
    point_selected_surf[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f)) {
      float pd2 =
          pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s > 0.9) {
        point_selected_surf[i]       = true;
        normvec->points[i].x         = pabcd(0);
        normvec->points[i].y         = pabcd(1);
        normvec->points[i].z         = pabcd(2);
        normvec->points[i].intensity = pd2;
        res_last[i]                  = abs(pd2);
      }
    }
  }

  effct_feat_num = 0;

  for (int i = 0; i < feats_down_size; i++) {
    if (point_selected_surf[i]) {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[i];
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    ROS_WARN("No Effective Points! \n");
    return;
  }

  res_mean_last = total_residual / effct_feat_num;
  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);  // 23
  ekfom_data.h.resize(effct_feat_num);

  for (int i = 0; i < effct_feat_num; i++) {
    const PointType &laser_p = laserCloudOri->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType &norm_p = corr_normvect->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (extrinsic_est_en) {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);  // s.rot.conjugate()*norm_vec);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
          VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0;
    }

    /*** Measuremnt: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -norm_p.intensity;
  }
  solve_time += omp_get_wtime() - solve_start_;
}

bool lookup_base_extrinsics(std::vector<double> &extrinT_Lidar_wrt_Base,
                            std::vector<double> &extrinR_Lidar_wrt_Base) {
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    ROS_INFO_STREAM("Looking up " << base_frame << " ->  " << lidar_frame);
    listener.waitForTransform(base_frame, lidar_frame, ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform(base_frame, lidar_frame, ros::Time(0), transform);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  // Translation
  extrinT_Lidar_wrt_Base[0] = transform.getOrigin().x();
  extrinT_Lidar_wrt_Base[1] = transform.getOrigin().y();
  extrinT_Lidar_wrt_Base[2] = transform.getOrigin().z();

  ROS_INFO("Translation: [%f, %f, %f]",
           extrinT_Lidar_wrt_Base[0],
           extrinT_Lidar_wrt_Base[1],
           extrinT_Lidar_wrt_Base[2]);

  // Rotation (Matrix)
  tf::Matrix3x3 m(transform.getRotation());
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      extrinR_Lidar_wrt_Base[i * 3 + j] = m[i][j];
    }
  }

  const auto q = transform.getRotation();
  ROS_INFO("Rotation (Quaternion): [%f, %f, %f, %f]", q.x(), q.y(), q.z(), q.w());

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("Rotation (RPY in radians): [%f, %f, %f]", roll, pitch, yaw);
  ROS_INFO("Rotation (RPY in degrees): [%f, %f, %f]",
           roll * 180.0 / M_PI,
           pitch * 180.0 / M_PI,
           yaw * 180.0 / M_PI);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_mapping");

  bool pcl_verbose = true;

  ros::NodeHandle nh("~");
  nh.param<bool>("publish/path_en", path_en, true);
  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
  nh.param<bool>("publish/scan_lidarframe_pub_en", scan_lidar_pub_en, true);
  nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
  nh.param<bool>("publish/scan_baseframe_pub_en", scan_base_pub_en, true);
  nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  nh.param<string>("map_file_path", map_file_path, "");
  nh.param<string>("common/save_dir", save_dir, "");
  nh.param<string>("common/sequence_name", sequence_name, "");
  nh.param<string>("common/map_frame", map_frame, "map");
  nh.param<string>("common/lidar_frame", lidar_frame, "lidar");
  nh.param<string>("common/base_frame", base_frame, "");
  nh.param<string>("common/imu_frame", imu_frame, "base");
  nh.param<string>("common/visualization_frame", visualization_frame, "imu");
  nh.param<bool>("common/time_sync_en", time_sync_en, false);
  nh.param<bool>("common/pcl_verbose", pcl_verbose, true);
  nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
  nh.param<double>("cube_side_length", cube_len, 200);
  nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
  nh.param<double>("mapping/fov_degree", fov_deg, 180);
  nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh.param<bool>("gravity_alignment/enable_gravity_alignment", enable_gravity_alignment, true);
  nh.param<double>("gravity_alignment/acc_diff_thr", acc_diff_thr, 0.2);
  nh.param<int>("gravity_alignment/num_moving_frames_thr", num_moving_frames_thr, 20);
  nh.param<int>("gravity_alignment/num_gravity_measurements_thr", num_gravity_measurements_thr, 20);
  nh.param<vector<double>>("gravity_alignment/g_base", g_base_vec, vector<double>{0.0, 0.0, -1.0});
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/blind", p_pre->blind_for_human_pilots, 1.5);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
  nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
  nh.param<int>("point_filter_num_for_preprocessing", p_pre->point_filter_num, 1);
  nh.param<int>("point_filter_num", point_filter_num, 4);
  nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
  nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
  nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, false);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
  ROS_DEBUG_STREAM("p_pre->lidar_type " << p_pre->lidar_type);

  if (!pcl_verbose) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  if (visualization_frame == "base" && base_frame.empty()) {
    ROS_FATAL("base_frame must not be empty to use base visualization");
    return 1;
  }

  path.header.stamp    = ros::Time::now();
  path.header.frame_id = map_frame;

  {  // temporary scope
    ROS_INFO_STREAM("extrinT: x=" << extrinT[0] << " y=" << extrinT[1] << " z=" << extrinT[2]);
    Eigen::Matrix3d lidar_R_imu;
    lidar_R_imu << MAT_FROM_ARRAY(extrinR);
    Eigen::Quaterniond q(lidar_R_imu);
    ROS_INFO_STREAM("extrinR: x=" << q.x() << " y=" << q.y() << " z=" << q.z() << " w=" << q.w());
  }

  // Flush
  std::string prefix = save_dir;
  if (!sequence_name.empty()) {
    prefix += prefix.empty() ? sequence_name : "/" + sequence_name;
  }

  if (prefix.empty()) {
    ROS_INFO("No save dir specified! not logging poses!");
  } else {
    ROS_INFO_STREAM("Save dir: " << prefix);
  }

  if (!prefix.empty()) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << filter_size_map_min;
    std::string voxel_size_str = oss.str();
    std::replace(voxel_size_str.begin(), voxel_size_str.end(), '.', '_');
    output_pose_file = prefix + "/FastLIO2_" + voxel_size_str + ".txt";
  }

  if (!output_pose_file.empty()) {
    std::ofstream pose_file;
    pose_file.open(output_pose_file);
    pose_file << "#timestamp x y z qx qy qz qw\n";
    pose_file.close();
  }

  vector<double> extrinT_Lidar_wrt_Base(3, 0.0);
  vector<double> extrinR_Lidar_wrt_Base(9, 0.0);
  extrinR_Lidar_wrt_Base[0] = 1.0;
  extrinR_Lidar_wrt_Base[4] = 1.0;
  extrinR_Lidar_wrt_Base[8] = 1.0;

  // load optional transform from IMU to robot base
  if (!base_frame.empty()) {
    if (!lookup_base_extrinsics(extrinT_Lidar_wrt_Base, extrinR_Lidar_wrt_Base)) {
      return 1;
    }
  }

  /*** variables definition ***/
  int frame_num          = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
         aver_time_solve = 0, aver_time_const_H_time = 0;

  FOV_DEG      = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
  HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

  _featsArray.reset(new PointCloudXYZI());

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));
  downSizeFilterSurf.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));

  g_base << g_base_vec[0], g_base_vec[1], g_base_vec[2];
  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  Lidar_T_wrt_Base << VEC_FROM_ARRAY(extrinT_Lidar_wrt_Base);
  Lidar_R_wrt_Base << MAT_FROM_ARRAY(extrinR_Lidar_wrt_Base);
  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

  double epsi[23] = {0.001};
  fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

  /*** debug record ***/
  string pos_log_dir = root_dir + "/Log/pos_log.txt";
  std::ofstream fp(pos_log_dir);

  ofstream fout_pre, fout_out;
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
  if (fout_pre && fout_out) {
    ROS_INFO_STREAM("~~~~" << ROOT_DIR << " file opened");
  } else {
    ROS_WARN_STREAM("~~~~" << ROOT_DIR << " doesn't exist");
  }

  /*** ROS subscribe initialization ***/
  ros::Subscriber sub_pcl;
  if (p_pre->lidar_type != AVIA) {
    sub_pcl = nh.subscribe("lidar", 200000, standard_pcl_cbk);
  } else {
#if defined(LIVOX_ROS_DRIVER_FOUND) && LIVOX_ROS_DRIVER_FOUND
    sub_pcl = nh.subscribe("lidar", 200000, livox_pcl_cbk);
#else
    ROS_FATAL("Not built with livox_ros_driver! Unable to suscribe to AVIA lidar");
    return 1;
#endif
  }

  ros::Subscriber sub_imu = nh.subscribe("imu", 200000, imu_cbk);
  ROS_INFO_STREAM("LiDAR topic: " << sub_pcl.getTopic());
  ROS_INFO_STREAM("IMU topic: " << sub_imu.getTopic());

  ros::Publisher pubLaserCloudFull =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_registered", 100000);
  ros::Publisher pubLaserCloudFull_lidar =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_registered_lidar", 100000);
  ros::Publisher pubLaserCloudFull_body =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_registered_body", 100000);
  ros::Publisher pubLaserCloudFull_base =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_registered_base", 100000);
  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("odometry", 100000);
  ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>("path", 100000);
  //------------------------------------------------------------------------------------------------------

  signal(SIGINT, SigHandle);
  ros::WallRate rate(5000);
  while (ros::ok()) {
    if (flg_exit) break;
    ros::spinOnce();
    if (sync_packages(Measures, false)) {
      if (flg_first_scan) {
        first_lidar_time        = Measures.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan          = false;
        continue;
      }

      double t0, t1, t3, t5;

      match_time         = 0;
      kdtree_search_time = 0.0;
      solve_time         = 0;
      solve_const_H_time = 0;
      t0                 = omp_get_wtime();

      // NOTE(hlim): Place resampling outside `Process` function to get full cloud point,
      // i.e., `cloud_undistort`: raw undistorted cloud points
      // & `feats_undistort`: Undistorted cloud points for pose estimation module
      cloud_undistort->clear();
      feats_undistort->clear();
      p_imu->Process(Measures, kf, cloud_undistort);
      feats_undistort->reserve(cloud_undistort->size() / point_filter_num);
      for (size_t i = 0; i < cloud_undistort->points.size(); ++i) {
        const auto &pt = cloud_undistort->points[i];
        if (i % point_filter_num != 0) continue;
        feats_undistort->points.emplace_back(pt);
      }

      state_point = kf.get_x();
      pos_lid     = state_point.pos + state_point.rot * state_point.offset_T_L_I;

      if (feats_undistort->empty() || (feats_undistort == NULL)) {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      static int num_consecutive_moving_frames = 0;
      if (enable_gravity_alignment && !is_gravity_aligned && !base_frame.empty()) {
        if (!flg_EKF_inited) {
          // Assume that it is stationary at the beginning.
          mean_acc_stopped = Measures.getMeanAcc();
        } else {
          const auto &mean_acc = Measures.getMeanAcc();
          const auto acc_diff  = (mean_acc_stopped - mean_acc).norm();
          if (acc_diff > acc_diff_thr) {
            num_consecutive_moving_frames = min(num_consecutive_moving_frames + 1, 100000);
          } else {
            ROS_WARN_STREAM(
                "Waiting for motion to perform gravity alignment...now a robot has been stopped");
            num_consecutive_moving_frames = 0;
          }
        }
      }

      flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
      /*** Segment the map in lidar FOV ***/
      lasermap_fov_segment();

      /*** downsample the feature points in a scan ***/
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      t1              = omp_get_wtime();
      feats_down_size = feats_down_body->points.size();

      /*****************************/

      /*** initialize the map kdtree ***/
      if (ikdtree.Root_Node == nullptr) {
        if (feats_down_size > 5) {
          ikdtree.set_downsample_param(filter_size_map_min);
          feats_down_world->resize(feats_down_size);
          for (int i = 0; i < feats_down_size; i++) {
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
          }
          ikdtree.Build(feats_down_world->points);
        }
        continue;
      }
      kdtree_size_st = ikdtree.size();

      //      cout << "effect num:" << effct_feat_num << endl;

      /*** ICP and iterated Kalman filter update ***/
      if (feats_down_size < 5) {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
      fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
               << euler_cur.transpose() << " " << state_point.pos.transpose() << " "
               << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " "
               << state_point.vel.transpose() << " " << state_point.bg.transpose() << " "
               << state_point.ba.transpose() << " " << state_point.grav << endl;

      // If you need to see map point, change to "if(1)"
      if (0) {
        PointVector().swap(ikdtree.PCL_Storage);
        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
        featsFromMap->clear();
        featsFromMap->points = ikdtree.PCL_Storage;
      }

      pointSearchInd_surf.resize(feats_down_size);
      Nearest_Points.resize(feats_down_size);

      /*** iterated state estimation ***/
      double t_update_start = omp_get_wtime();
      double solve_H_time   = 0;
      kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);

      /***** Perform gravity alignment *****/
      // NOTE(hlim): Introduce a delay using `num_moving_frames_thr` to make sure
      // the gravity vectors are sufficiently updated.
      if (enable_gravity_alignment && !is_gravity_aligned && !base_frame.empty() &&
          (num_consecutive_moving_frames > num_moving_frames_thr)) {
        static const auto &offset_R_I_B = Lidar_R_wrt_Base * state_point.offset_R_L_I.inverse();

        // NOTE(hlim): Here, we don't need to normalize the scale of vectors
        V3D gravity_direction = kf.get_x().grav;
        if (global_gravity_directions.size() < static_cast<size_t>(num_gravity_measurements_thr)) {
          ROS_INFO_STREAM("Waiting for motion: " << global_gravity_directions.size() << " / "
                                                 << num_gravity_measurements_thr);
          global_gravity_directions.push_back(offset_R_I_B * gravity_direction);
        } else {
          V3D avg_global_gravity_vec = Eigen::Vector3d::Zero();
          for (const auto &gravity_vec : global_gravity_directions) {
            avg_global_gravity_vec += gravity_vec;
          }
          avg_global_gravity_vec /= global_gravity_directions.size();

          R_gravity_aligned = computeRelativeRotation(avg_global_gravity_vec, g_base);
          ROS_INFO_STREAM("Gravity alignment complete! `R_gravity_aligned`: " << R_gravity_aligned);
          is_gravity_aligned = true;
          local_gravity_directions.clear();
        }
      }
      /*********************************/

      state_point = kf.get_x();
      // Update corrected rotation here
      state_point.pos = R_gravity_aligned * state_point.pos;
      state_point.rot = R_gravity_aligned * state_point.rot;
      euler_cur       = SO3ToEuler(state_point.rot);
      pos_lid         = state_point.pos + state_point.rot * state_point.offset_T_L_I;
      geoQuat.x       = state_point.rot.coeffs()[0];
      geoQuat.y       = state_point.rot.coeffs()[1];
      geoQuat.z       = state_point.rot.coeffs()[2];
      geoQuat.w       = state_point.rot.coeffs()[3];

      double t_update_end = omp_get_wtime();

      if (enable_gravity_alignment && !is_gravity_aligned && !base_frame.empty()) {
        ROS_WARN_STREAM(
            "Gravity alignment is enabled but not yet completed. Waiting for alignment...");
        continue;
      }
      /******* Publish odometry *******/
      publish_odometry(pubOdomAftMapped);

      /*** add the feature points to map kdtree ***/
      t3 = omp_get_wtime();
      map_incremental();
      t5 = omp_get_wtime();

      /******* Publish points *******/
      if (path_en) publish_path(pubPath);
      if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFull);
      if (scan_pub_en && scan_lidar_pub_en) publish_frame(pubLaserCloudFull_lidar, "lidar");
      if (scan_pub_en && scan_body_pub_en) publish_frame(pubLaserCloudFull_body, "imu");
      if (scan_pub_en && scan_base_pub_en) publish_frame(pubLaserCloudFull_base, "base");

      /*** Debug variables ***/
      if (runtime_pos_log) {
        frame_num++;
        kdtree_size_end = ikdtree.size();
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        aver_time_icp   = aver_time_icp * (frame_num - 1) / frame_num +
                        (t_update_end - t_update_start) / frame_num;
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_incre =
            aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
        aver_time_solve =
            aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
        aver_time_const_H_time =
            aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
        T1[time_log_counter]       = Measures.lidar_beg_time;
        s_plot[time_log_counter]   = t5 - t0;
        s_plot2[time_log_counter]  = feats_undistort->points.size();
        s_plot3[time_log_counter]  = kdtree_incremental_time;
        s_plot4[time_log_counter]  = kdtree_search_time;
        s_plot5[time_log_counter]  = kdtree_delete_counter;
        s_plot6[time_log_counter]  = kdtree_delete_time;
        s_plot7[time_log_counter]  = kdtree_size_st;
        s_plot8[time_log_counter]  = kdtree_size_end;
        s_plot9[time_log_counter]  = aver_time_consu;
        s_plot10[time_log_counter] = add_point_size;
        time_log_counter++;
        ROS_INFO(
            "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
            "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: "
            "%0.6f \n",
            t1 - t0,
            aver_time_match,
            aver_time_solve,
            t3 - t1,
            t5 - t3,
            aver_time_consu,
            aver_time_icp,
            aver_time_const_H_time);
        ext_euler = SO3ToEuler(state_point.offset_R_L_I);
        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                 << euler_cur.transpose() << " " << state_point.pos.transpose() << " "
                 << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " "
                 << state_point.vel.transpose() << " " << state_point.bg.transpose() << " "
                 << state_point.ba.transpose() << " " << state_point.grav << " "
                 << feats_undistort->points.size() << endl;
        dump_lio_state_to_log(fp);
      }
    }

    rate.sleep();
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
   * 2. pcd save will largely influence the real-time performences **/
  if (pcl_wait_save->size() > 0 && pcd_save_en) {
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    cout << "current scan saved to /PCD/" << file_name << endl;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }

  fout_out.close();
  fout_pre.close();

  if (runtime_pos_log) {
    vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
    string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    std::ofstream fp2(log_dir);
    fp2 << "time_stamp, total time, scan point size, incremental time, search time, delete size, "
           "delete time, tree size st, tree size end, add point size, preprocess time"
        << std::endl;
    for (int i = 0; i < time_log_counter; i++) {
      fp2 << std::setprecision(8) << T1[i] << "," << s_plot[i] << ","
          << static_cast<int>(s_plot2[i]) << "," << s_plot3[i] << "," << s_plot4[i] << ","
          << static_cast<int>(s_plot5[i]) << "," << s_plot6[i] << ","
          << static_cast<int>(s_plot7[i]) << "," << static_cast<int>(s_plot8[i]) << ","
          << static_cast<int>(s_plot10[i]) << "," << s_plot11[i] << std::endl;
      t.push_back(T1[i]);
      s_vec.push_back(s_plot9[i]);
      s_vec2.push_back(s_plot3[i] + s_plot6[i]);
      s_vec3.push_back(s_plot4[i]);
      s_vec5.push_back(s_plot[i]);
    }
  }

  return 0;
}
