#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <string>
#include <vector>
#include "filter_extract_lib.hpp"
#include "common.h"
#include "tic_toc.h"

using std::atan2;
using std::cos;
using std::sin;


constexpr bool b_viz_curv = false;
constexpr bool b_voxel_filter = false;
constexpr bool b_normalize_curv = true;

const double scanPeriod = 0.1;

const int systemDelay = 0;
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;

float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000]; // not picked 0, picked 1(不能作为特征点)
// cloudLabel:
// normal 0, curvature < kThresholdFlat -1, too far or too near 99, 
// edgePointsLessSharp 1, edgePointsSharp 2
int cloudLabel[400000];  

bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubEdgePointsSharp;
ros::Publisher pubEdgePointsLessSharp;
ros::Publisher pubPlanarPointsFlat;
ros::Publisher pubPlanarPointsLessFlat;
ros::Publisher pubColorEdgePlanarPoints;
ros::Publisher pubColorLessEdgePlanarPoints;

std::vector<ros::Publisher> pubEachScan;


bool b_PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1;
double THRESHOLD_FLAT = 0.01;
double THRESHOLD_SHARP = 0.01;

int kNumCurvSize = 5;
constexpr int kNumRegion = 50;       // 6
constexpr int kNumEdge = 2;          // 2 最大edge points数目
constexpr int kNumFlat = 4;          // 4 最大 planar points数目
constexpr int kNumEdgeNeighbor = 5;  // 5;
constexpr int kNumFlatNeighbor = 5;  // 5;
float kThresholdSharp = 50;          // 0.1;
float kThresholdFlat = 30;           // 0.1;
constexpr float kThresholdLessflat = 0.1;

constexpr float kDistanceFaraway = 25;


void initialize_and_filter(const int &cloudSize, const pcl::PointCloud<PointType>::Ptr &laserCloud)
{
  // 1. 计算曲率 2. 初始化cloudSortInd, cloudNeighborPicked, cloudLabel 3. unreliable points(太近或太远)
  for (int i = 5; i < cloudSize - 5; i++) {
    float dis = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);
    if (dis > kDistanceFaraway) {
      kNumCurvSize = 2;
    }
    float diffX = 0, diffY = 0, diffZ = 0;
    for (int j = 1; j <= kNumCurvSize; ++j) {
      diffX += laserCloud->points[i - j].x + laserCloud->points[i + j].x;
      diffY += laserCloud->points[i - j].y + laserCloud->points[i + j].y;
      diffZ += laserCloud->points[i - j].z + laserCloud->points[i + j].z;
    }
    diffX -= 2 * kNumCurvSize * laserCloud->points[i].x;
    diffY -= 2 * kNumCurvSize * laserCloud->points[i].y;
    diffZ -= 2 * kNumCurvSize * laserCloud->points[i].z;
   

    float tmp2 = diffX * diffX + diffY * diffY + diffZ * diffZ;
    float tmp = sqrt(tmp2);
    // 在一个邻域内计算扫描方向点云的Laplacian算子，作为点云在该点的曲率
    cloudCurvature[i] = tmp2;
    if (b_normalize_curv) {
      /// use normalized curvature(论文LOAM中提出)
      cloudCurvature[i] = tmp / (2 * kNumCurvSize * dis + 1e-3);
    }
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    /// Mark un-reliable points
    constexpr float kMaxFeatureDis = 1e4;
    if (fabs(dis) > kMaxFeatureDis || fabs(dis) < 1e-4 || !std::isfinite(dis)) {
      cloudLabel[i] = 99;
      cloudNeighborPicked[i] = 1; // 标签为1不能作为特征点
    }
  }
}

void filter_sharp_points(const int &cloudSize, const pcl::PointCloud<PointType>::Ptr &laserCloud)
{
  // 一个点距前一个点和距后一个点的距离都太远就舍弃(尖锐的点)
  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
    float dis = laserCloud->points[i].x * laserCloud->points[i].x +
                laserCloud->points[i].y * laserCloud->points[i].y +
                laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.00015 * dis && diff2 > 0.00015 * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }
}

void extract_edge_points(pcl::PointCloud<PointType> &edgePointsSharp, pcl::PointCloud<PointType> &edgePointsLessSharp, const pcl::PointCloud<PointType>::Ptr &laserCloud, 
const int &sp, const int &ep)
{
  // ////////////////////////////////////////////////////////////////////////
  // 提取edge point
  // ////////////////////////////////////////////////////////////////////////
  int largestPickedNum = 0;
  for (int k = ep; k >= sp; k--) {
    int ind = cloudSortInd[k];

    if (cloudNeighborPicked[ind] != 0) continue;
    // 提取edge points(曲率很大的点)
    if (cloudCurvature[ind] > kThresholdSharp) {
      largestPickedNum++;
      // kNumEdge，edge points个数有上界，2个以内Sharp，20个以内lessSharp
      if (largestPickedNum <= kNumEdge) {
        cloudLabel[ind] = 2;
        edgePointsSharp.push_back(laserCloud->points[ind]);
        edgePointsLessSharp.push_back(laserCloud->points[ind]);
      } else if (largestPickedNum <= 20) {
        cloudLabel[ind] = 1;
        edgePointsLessSharp.push_back(laserCloud->points[ind]);
      } else {
        break;
      }

      cloudNeighborPicked[ind] = 1;

      // 如果在周围5个点之内的点的距离小于0.02m，就认为周围点被picked了
      for (int l = 1; l <= kNumEdgeNeighbor; l++) {
        float diffX = laserCloud->points[ind + l].x -
                      laserCloud->points[ind + l - 1].x;
        float diffY = laserCloud->points[ind + l].y -
                      laserCloud->points[ind + l - 1].y;
        float diffZ = laserCloud->points[ind + l].z -
                      laserCloud->points[ind + l - 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
          break;
        }

        cloudNeighborPicked[ind + l] = 1;
      }
      for (int l = -1; l >= -kNumEdgeNeighbor; l--) {
        float diffX = laserCloud->points[ind + l].x -
                      laserCloud->points[ind + l + 1].x;
        float diffY = laserCloud->points[ind + l].y -
                      laserCloud->points[ind + l + 1].y;
        float diffZ = laserCloud->points[ind + l].z -
                      laserCloud->points[ind + l + 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
          break;
        }

        cloudNeighborPicked[ind + l] = 1;
      }
    }
  }
}

void extract_planar_points(pcl::PointCloud<PointType> &planarPointsFlat, pcl::PointCloud<PointType>::Ptr planarPointsLessFlatScan, 
const pcl::PointCloud<PointType>::Ptr &laserCloud, const int &sp, const int &ep)
{
  int smallestPickedNum = 0;
  for (int k = sp; k <= ep; k++) 
  {
    int ind = cloudSortInd[k];

    if (cloudNeighborPicked[ind] != 0) continue;

    if (cloudCurvature[ind] < kThresholdFlat) {
      cloudLabel[ind] = -1;
      planarPointsFlat.push_back(laserCloud->points[ind]);
      cloudNeighborPicked[ind] = 1;

      smallestPickedNum++;
      if (smallestPickedNum >= kNumFlat) {
        // 4 points
        break;
      }
      // 如果在周围5个点之内的点的距离小于0.02m，就认为周围点被picked了
      for (int l = 1; l <= kNumFlatNeighbor; l++) {
        float diffX = laserCloud->points[ind + l].x -
                      laserCloud->points[ind + l - 1].x;
        float diffY = laserCloud->points[ind + l].y -
                      laserCloud->points[ind + l - 1].y;
        float diffZ = laserCloud->points[ind + l].z -
                      laserCloud->points[ind + l - 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
          break;
        }

        cloudNeighborPicked[ind + l] = 1;
      }
      for (int l = -1; l >= -kNumFlatNeighbor; l--) {
        float diffX = laserCloud->points[ind + l].x -
                      laserCloud->points[ind + l + 1].x;
        float diffY = laserCloud->points[ind + l].y -
                      laserCloud->points[ind + l + 1].y;
        float diffZ = laserCloud->points[ind + l].z -
                      laserCloud->points[ind + l + 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
          break;
        }

        cloudNeighborPicked[ind + l] = 1;
      }
    }
  }
  // 标签小于等于0，激光点云的点曲率小于阈值0.1 lessflat，planar points多多益善
  for (int k = sp; k <= ep; k++) {
    if (cloudLabel[k] <= 0 && cloudCurvature[k] < kThresholdLessflat) {
      planarPointsLessFlatScan->push_back(laserCloud->points[k]);
    }
  }
}

void pub_color( const pcl::PointCloud<PointType> &edgePointsSharp, 
                  const pcl::PointCloud<PointType> &edgePointsLessSharp,
                  const pcl::PointCloud<PointType> &planarPointsFlat,
                  const pcl::PointCloud<PointType> &planarPointsLessFlat, const ros::Time &stamp)
{
  
  pcl::PointXYZRGB point;
  pcl::PointCloud<pcl::PointXYZRGB> sharp_flat_points;
  pcl::PointCloud<pcl::PointXYZRGB> less_sharp_flat_points;
  for(int i=0; i < edgePointsSharp.size(); ++i)
  {
    point.x = edgePointsSharp.points[i].x;
    point.y = edgePointsSharp.points[i].y;
    point.z = edgePointsSharp.points[i].z;
    point.r = 255;
    point.g = 0;
    point.b = 0;
    sharp_flat_points.push_back(point);
  }
  for(int i=0; i < planarPointsFlat.size(); ++i)
  {
    point.x = planarPointsFlat.points[i].x;
    point.y = planarPointsFlat.points[i].y;
    point.z = planarPointsFlat.points[i].z;
    point.r = 0;
    point.g = 0;
    point.b = 255;
    sharp_flat_points.push_back(point);
  }
  // /////////////////////////////////////////////
  for(int i=0; i < edgePointsLessSharp.size(); ++i)
  {
    point.x = edgePointsLessSharp.points[i].x;
    point.y = edgePointsLessSharp.points[i].y;
    point.z = edgePointsLessSharp.points[i].z;
    point.r = 255;
    point.g = 0;
    point.b = 0;
    less_sharp_flat_points.push_back(point);
  }
  for(int i=0; i < planarPointsLessFlat.size(); ++i)
  {
    point.x = planarPointsLessFlat.points[i].x;
    point.y = planarPointsLessFlat.points[i].y;
    point.z = planarPointsLessFlat.points[i].z;
    point.r = 0;
    point.g = 0;
    point.b = 255;
    less_sharp_flat_points.push_back(point);
  }
  sensor_msgs::PointCloud2 sharp_flat_points2;
  pcl::toROSMsg(sharp_flat_points, sharp_flat_points2);
  sharp_flat_points2.header.stamp = stamp;
  sharp_flat_points2.header.frame_id = "/aft_extract";
  pubColorEdgePlanarPoints.publish(sharp_flat_points2);

  sensor_msgs::PointCloud2 less_sharp_flat_points2;
  pcl::toROSMsg(less_sharp_flat_points, less_sharp_flat_points2);
  less_sharp_flat_points2.header.stamp = stamp;
  less_sharp_flat_points2.header.frame_id = "/aft_extract";
  pubColorLessEdgePlanarPoints.publish(less_sharp_flat_points2);

};



void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    } else
      return;
  }

  TicToc t_whole;
  TicToc t_prepare;
  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  pcl::PointCloud<PointType> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;
  // 1. filtering
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);
  int cloudSize = laserCloudIn.points.size();

  // 2. order according to scan num and merge
  PointType point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS); // 6个SCAN
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn.points[i].x;
    point.y = laserCloudIn.points[i].y;
    point.z = laserCloudIn.points[i].z;
    point.intensity = laserCloudIn.points[i].intensity; // intensity 整数是scan的数目，小数是时间戳
    point.curvature = laserCloudIn.points[i].curvature; // curvature 是0.1的反射率
    int scanID = 0;
    // main函数中改成了6
    if (N_SCANS == 6) {
      scanID = (int)point.intensity;
    }
    laserCloudScans[scanID].push_back(point);
  }

  // printf("points size %d \n", cloudSize);
  // PointType -> PointXYZINormal(merge)
  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  for (int i = 0; i < N_SCANS; i++) {
    scanStartInd[i] = laserCloud->size() + 5;
    *laserCloud += laserCloudScans[i];
    scanEndInd[i] = laserCloud->size() - 6;
  } 
  // printf("prepare time %f \n", t_prepare.toc());

  
  // 3. 三个内容: 1. 计算曲率 2. 初始化cloudSortInd, cloudNeighborPicked, cloudLabel 3. unreliable points(太近或太远)
  initialize_and_filter(cloudSize, laserCloud);

  // 4. 一个点距前一个点和距后一个点的距离都太远就舍弃(尖锐的点，或是某种空间上的噪点)
  filter_sharp_points(cloudSize, laserCloud);

  TicToc t_pts;

  pcl::PointCloud<PointType> edgePointsSharp;
  pcl::PointCloud<PointType> edgePointsLessSharp;
  pcl::PointCloud<PointType> planarPointsFlat;
  pcl::PointCloud<PointType> planarPointsLessFlat;

  if (b_normalize_curv) {
    kThresholdFlat = THRESHOLD_FLAT;
    kThresholdSharp = THRESHOLD_SHARP;
  }

  float t_q_sort = 0;
  // kNumCurvSize计算曲率的邻居数6； kNumRegion：把一个扫描线再分成若干区域(50)
  for (int i = 0; i < N_SCANS; i++) {
    if (scanEndInd[i] - scanStartInd[i] < kNumCurvSize) continue;
    pcl::PointCloud<PointType>::Ptr planarPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < kNumRegion; j++) {
      // start point
      int sp =
          scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / kNumRegion;
      int ep = scanStartInd[i] +
               (scanEndInd[i] - scanStartInd[i]) * (j + 1) / kNumRegion - 1;
      
      TicToc t_tmp;
      // sort the curvatures from small to large： 按照曲率由小到大排序，不改变点，改变id
      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) 
          {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      float SumCurRegion = 0.0;
      float MaxCurRegion = cloudCurvature[cloudSortInd[ep]];  //the largest curvature in sp ~ ep
      for (int k = ep - 1; k >= sp; k--) {
        SumCurRegion += cloudCurvature[cloudSortInd[k]];
      }

      if (MaxCurRegion > 3 * SumCurRegion)
        cloudNeighborPicked[cloudSortInd[ep]] = 1;  // 如果edge_points曲率大于三倍的曲率的和，则该点picked,是噪点，不能作为edge point

      t_q_sort += t_tmp.toc();
      // ////////////////////////////////////////////////////////////////////////
      // 提取edge point
      // ////////////////////////////////////////////////////////////////////////
      extract_edge_points(edgePointsSharp, edgePointsLessSharp, laserCloud, sp, ep);
      

      // ////////////////////////////////////////////////////////////////////////
      // 提取planar point
      // ////////////////////////////////////////////////////////////////////////
      extract_planar_points(planarPointsFlat, planarPointsLessFlatScan, laserCloud, sp, ep);
      
    }
    // /////////////////////////////////////////////////////////
    // 对surfPointsLessFlat体素滤波
    // /////////////////////////////////////////////////////////
    planarPointsLessFlat += planarPointsFlat;
    edgePointsLessSharp += edgePointsSharp;
    /// Whether downsample less-flat points
    if (b_voxel_filter) {
      pcl::PointCloud<PointType> planarPointsLessFlatScanDS;
      // 体素滤波
      pcl::VoxelGrid<PointType> downSizeFilter;
      downSizeFilter.setInputCloud(planarPointsLessFlatScan);
      downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
      downSizeFilter.filter(planarPointsLessFlatScanDS);
      planarPointsLessFlat += planarPointsLessFlatScanDS;
    } else {
      planarPointsLessFlat += *planarPointsLessFlatScan;
    }
  }
  // printf("sort q time %f \n", t_q_sort);
  // printf("seperate points time %f \n", t_pts.toc());

  /// Visualize curvature
  if (b_viz_curv) {
    std_msgs::Header ros_hdr = laserCloudMsg->header;
    ros_hdr.frame_id = "/aft_extract";
    VisualizeCurvature(cloudCurvature, cloudLabel, *laserCloud, ros_hdr);
  }
  const auto& stamp = laserCloudMsg->header.stamp;
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = stamp;
  laserCloudOutMsg.header.frame_id = "/aft_extract";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 edgePointsSharpMsg;
  pcl::toROSMsg(edgePointsSharp, edgePointsSharpMsg);
  edgePointsSharpMsg.header.stamp = stamp;
  edgePointsSharpMsg.header.frame_id = "/aft_extract";
  pubEdgePointsSharp.publish(edgePointsSharpMsg);

  sensor_msgs::PointCloud2 edgePointsLessSharpMsg;
  pcl::toROSMsg(edgePointsLessSharp, edgePointsLessSharpMsg);
  edgePointsLessSharpMsg.header.stamp = stamp;
  edgePointsLessSharpMsg.header.frame_id = "/aft_extract";
  pubEdgePointsLessSharp.publish(edgePointsLessSharpMsg);

  sensor_msgs::PointCloud2 planarPointsFlat2;
  pcl::toROSMsg(planarPointsFlat, planarPointsFlat2);
  planarPointsFlat2.header.stamp = stamp;
  planarPointsFlat2.header.frame_id = "/aft_extract";
  pubPlanarPointsFlat.publish(planarPointsFlat2);

  sensor_msgs::PointCloud2 planarPointsLessFlat2;
  pcl::toROSMsg(planarPointsLessFlat, planarPointsLessFlat2);
  planarPointsLessFlat2.header.stamp = stamp;
  planarPointsLessFlat2.header.frame_id = "/aft_extract";
  pubPlanarPointsLessFlat.publish(planarPointsLessFlat2);
  // ////////////////////////////////////////////////////////
  // publish color
  // ////////////////////////////////////////////////////////
  pub_color(edgePointsSharp, edgePointsLessSharp, planarPointsFlat, planarPointsLessFlat, stamp);

  // pub each scam
  if (b_PUB_EACH_LINE) {
    for (int i = 0; i < N_SCANS; i++) {
      sensor_msgs::PointCloud2 scanMsg;
      pcl::toROSMsg(laserCloudScans[i], scanMsg);
      scanMsg.header.stamp = stamp;
      scanMsg.header.frame_id = "/aft_extract";
      pubEachScan[i].publish(scanMsg);
    }
  }

  printf("scan registration time %f ms *************\n", t_whole.toc());
  if (t_whole.toc() > 100) ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_feature");
  ros::NodeHandle nh;

  nh.param<int>("scan_line", N_SCANS, 6); // Horizon has 6 scan lines
  nh.param<double>("threshold_flat", THRESHOLD_FLAT, 0.01);
  nh.param<double>("threshold_sharp", THRESHOLD_SHARP, 0.01);

  printf("scan line number %d \n", N_SCANS);

  if (N_SCANS != 6) {
    printf("only support livox horizon lidar!");
    return 0;
  }

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/livox_repub", 100, laserCloudHandler);
  
  // 滤波后的全部点
  pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_filter", 100);

  pubEdgePointsSharp =
      nh.advertise<sensor_msgs::PointCloud2>("/feature_sharp", 100);

  pubEdgePointsLessSharp =
      nh.advertise<sensor_msgs::PointCloud2>("/feature_less_sharp", 100);

  pubPlanarPointsFlat =
      nh.advertise<sensor_msgs::PointCloud2>("/feature_flat", 100);

  pubPlanarPointsLessFlat =
      nh.advertise<sensor_msgs::PointCloud2>("/feature_less_flat", 100);

  pubColorEdgePlanarPoints = 
      nh.advertise<sensor_msgs::PointCloud2>("/feature_color_sharp_flat", 100);
  
  pubColorLessEdgePlanarPoints = 
      nh.advertise<sensor_msgs::PointCloud2>("/feature_less_color_sharp_flat", 100);

  
  if (b_PUB_EACH_LINE) {
    for (int i = 0; i < N_SCANS; i++) {
      ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>(
          "/laser_scanid_" + std::to_string(i), 100);
      pubEachScan.push_back(tmp);
    }
  }
  ros::spin();

  return 0;
}

