#pragma once
#include <pcl/point_types.h>


constexpr bool dbg_show_id = false;

// 原位操作,避免push_back耗时操作
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }
  size_t j = 0;
  float thresh_hold = thres * thres;
  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    if (cloud_in.points[i].x * cloud_in.points[i].x +
            cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z <
        thresh_hold)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}

template <typename PointT>
void VisualizeCurvature(float *v_curv, int *v_label,
                        const pcl::PointCloud<PointT> &pcl_in,
                        const std_msgs::Header &hdr) {
  ROS_ASSERT(pcl_in.size() < 400000);

  /// Same marker attributes
  visualization_msgs::Marker txt_mk;
  txt_mk.header = hdr;
  txt_mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  txt_mk.ns = "default";
  txt_mk.id = 0;
  txt_mk.action = visualization_msgs::Marker::ADD;
  txt_mk.pose.orientation.x = 0;
  txt_mk.pose.orientation.y = 0;
  txt_mk.pose.orientation.z = 0;
  txt_mk.pose.orientation.w = 1;
  txt_mk.scale.z = 0.05;
  txt_mk.color.a = 1;
  txt_mk.color.r = 0;
  txt_mk.color.g = 1;
  txt_mk.color.b = 0;

  static visualization_msgs::MarkerArray curv_txt_msg;
 

  /// Marger array message
  static size_t pre_pt_num = 0;
  size_t pt_num = pcl_in.size();

  if (pre_pt_num == 0) {
    curv_txt_msg.markers.reserve(400000);
  }
  if (pre_pt_num > pt_num) {
    curv_txt_msg.markers.resize(pre_pt_num);
  } else {
    curv_txt_msg.markers.resize(pt_num);
  }

  int edge_num = 0, edgeless_num = 0, flat_num = 0, flatless_num = 0, nn = 0;

  /// Add marker and namespace
  for (size_t i = 0; i < pcl_in.size(); ++i) {
    auto curv = v_curv[i];
    auto label = v_label[i];  /// -1: flat, 0: less-flat, 1:less-edge, 2:edge
    const auto &pt = pcl_in[i];

    switch (label) {
      case 2: {
        /// edge
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "edge";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 1;
        mk_i.color.r = 1;
        mk_i.color.g = 0;
        mk_i.color.b = 0;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        edge_num++;
        break;
      }
      case 1: {
        /// less edge
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "edgeless";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 0.5;
        mk_i.color.r = 0.5;
        mk_i.color.g = 0;
        mk_i.color.b = 0.8;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        edgeless_num++;
        break;
      }
      case 0: {
        /// less flat
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "flatless";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 0.5;
        mk_i.color.r = 0;
        mk_i.color.g = 0.5;
        mk_i.color.b = 0.8;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        flatless_num++;
        break;
      }
      case -1: {
        /// flat
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "flat";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 1;
        mk_i.color.r = 0;
        mk_i.color.g = 1;
        mk_i.color.b = 0;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        flat_num++;
        break;
      }
      default: {
        /// Un-reliable
        /// Do nothing for label=99
        // ROS_ASSERT_MSG(false, "%d", label);
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "unreliable";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 0;
        mk_i.color.r = 0;
        mk_i.color.g = 0;
        mk_i.color.b = 0;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);

        nn++;
        break;
      }
    }
  }
  ROS_INFO("edge/edgeless/flatless/flat/nn num: [%d / %d / %d / %d / %d] - %lu",
           edge_num, edgeless_num, flatless_num, flat_num, nn, pt_num);

  /// Delete old points
  if (pre_pt_num > pt_num) {
    ROS_WARN("%lu > %lu", pre_pt_num, pt_num);
    // curv_txt_msg.markers.resize(pre_pt_num);
    for (size_t i = pt_num; i < pre_pt_num; ++i) {
      auto &mk_i = curv_txt_msg.markers[i];
      mk_i.action = visualization_msgs::Marker::DELETE;
      mk_i.color.a = 0;
      mk_i.color.r = 0;
      mk_i.color.g = 0;
      mk_i.color.b = 0;
      mk_i.ns = "old";
      mk_i.text = "";
    }
  }
  pre_pt_num = pt_num;

  // pub_curvature.publish(curv_txt_msg);
}
