#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <stdio.h>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "occupancy_grid_map/occupancygrid.hpp"

namespace PerceptionNS
{
  GroundSegmentor::GroundSegmentor(const rclcpp::NodeOptions &node_options)
      : rclcpp::Node("occupancy_grid_map_node", node_options),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_),
        pointcloud_sub1_(this, "input/pcl_1", rclcpp::SensorDataQoS().get_rmw_qos_profile()),
        pointcloud_sub2_(this, "input/pcl_2", rclcpp::SensorDataQoS().get_rmw_qos_profile())
  {
    // Define placeholders for callback arguments
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Define Quality of Service (QoS) settings for video streaming
    rclcpp::QoS m_video_qos(2);
    m_video_qos.keep_last(1);          // Keep only the last message
    m_video_qos.best_effort();         // Attempt to deliver the message but do not retry if it fails
    m_video_qos.durability_volatile(); // Messages are not saved to disk

    // Set parameters from the parameter server or use default values
    target_frame_ = declare_parameter("target_frame", "base_link");
    length_ = this->declare_parameter<std::vector<double>>("length");
    cell_size_ = this->declare_parameter<float>("cell_size", 0.5);
    obstacle_separation_threshold_ = this->declare_parameter<float>("obstacle_separation_threshold", 1.0);
    projection_dz_threshold_ = this->declare_parameter<double>("projection_dz_threshold", 1.0);
    origin_z_ = this->declare_parameter<double>("origin_z", -0.533);
    robot_z_ = this->declare_parameter<double>("robot_z", -1.8);
    bins_numb_ = this->declare_parameter<double>("bins_numb", 0.1);

    // Set up information for the occupancy grid message
    grid_msg_.header.frame_id = target_frame_;
    grid_msg_.info.resolution = cell_size_;
    grid_msg_.info.origin.position.x = -length_[0] / 2.;
    grid_msg_.info.origin.position.y = -length_[1] / 2.;
    grid_msg_.info.origin.position.z = 0;
    grid_msg_.info.origin.orientation.x = 0;
    grid_msg_.info.origin.orientation.y = 0;
    grid_msg_.info.origin.orientation.z = 0;
    grid_msg_.info.origin.orientation.w = 1;

    // Calculate grid dimensions and size
    cell_num_x_ = lround(length_[0] / cell_size_);
    cell_num_y_ = lround(length_[1] / cell_size_);
    grid_msg_.info.width = cell_num_x_;
    grid_msg_.info.height = cell_num_y_;
    grid_size_ = cell_num_x_ * cell_num_y_;
    cell_size_recp_ = 1 / cell_size_;
    bottomright_x_ = -length_[0] / 2.;
    bottomright_y_ = -length_[1] / 2.;
    origin_index_x_ = fabs(0 - bottomright_x_) * cell_size_recp_;
    origin_index_y_ = fabs(0 - bottomright_y_) * cell_size_recp_;
    origin_idx_ = origin_index_y_ * cell_num_x_ + origin_index_x_;

    // Set angle parameters for lidar
    min_angle_ = this->deg2rad(-180.0);
    max_angle_ = this->deg2rad(180.0);
    angle_increment_ = this->deg2rad(bins_numb_);

    // Create a Synchronizer for two point cloud subscribers
    sync_ = std::make_shared<Sync>(SyncPolicy(10), pointcloud_sub1_, pointcloud_sub2_);

    // Register callback function for synchronized point cloud messages
    sync_->registerCallback(std::bind(&GroundSegmentor::synchro_callback, this, _1, _2));

    // Create publisher for the occupancy grid map
    grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_topic_", 5);
  }

  inline int sign(int x)
  {
    return x > 0 ? 1.0 : -1.0;
  }

  /**
   * @brief Perform Bresenham's algorithm for line drawing in a 2D grid.
   *
   * This function marks cells along a line in a 2D grid (represented by an occupancy grid map)
   * as either occupied, unknown, or free based on the provided state. Bresenham's algorithm
   * efficiently traces a line between two points in a grid.
   *
   * @param map Reference to the occupancy grid map to perform line drawing on.
   * @param abs_da Absolute value of the delta along the dominant dimension.
   * @param abs_db Absolute value of the delta along the non-dominant dimension.
   * @param error_b The initial error value along the non-dominant dimension.
   * @param offset_a Offset along the dominant dimension.
   * @param offset_b Offset along the non-dominant dimension.
   * @param offset The starting offset within the grid map.
   * @param state The state to set the cells along the line to (occupied, unknown, or free).
   * @param max_length Maximum length of the line.
   */
  inline void GroundSegmentor::bresenham2D(
      nav_msgs::msg::OccupancyGrid &map, unsigned int abs_da, unsigned int abs_db, int error_b,
      int offset_a, int offset_b, unsigned int offset, CellState state, unsigned int max_length)
  {
    // Determine the end point based on the maximum length
    unsigned int end = std::min(max_length, abs_da);

    // Loop through each cell along the line
    for (unsigned int i = 0; i < end; ++i)
    {
      // Set the cell state based on the provided state
      if (state == CellState::OCCUPIED)
      {
        map.data[offset] = 97; // ASCII code for 'a', indicating occupied cell
      }
      else if (state == CellState::UNKNOWN)
      {
        map.data[offset] = 50; // ASCII code for '2', indicating unknown cell
      }
      else
      {
        map.data[offset] = 3; // Arbitrary value (here ASCII code for ETX) indicating free cell
      }

      // Move to the next cell along the dominant dimension
      offset += offset_a;

      // Update the error value along the non-dominant dimension
      error_b += abs_db;

      // Check if a step along the non-dominant dimension should be taken
      if ((unsigned int)error_b >= abs_da)
      {
        offset += offset_b;
        error_b -= abs_da;
      }
    }

    // Set the state of the last cell along the line
    if (state == CellState::OCCUPIED)
    {
      map.data[offset] = 97;
    }
    else if (state == CellState::UNKNOWN)
    {
      map.data[offset] = 50;
    }
    else
    {
      map.data[offset] = 3;
    }
  }

  /**
   * @brief Perform ray tracing to mark cells as occupied along a line in the occupancy grid map.
   *
   * This function traces a line from (x0, y0) to (x1, y1) in the occupancy grid map and marks
   * the cells along the line as occupied. The line is scaled based on the maximum length specified.
   *
   * @param map Reference to the occupancy grid map to perform ray tracing on.
   * @param x0 Starting X coordinate of the line.
   * @param y0 Starting Y coordinate of the line.
   * @param x1 Ending X coordinate of the line.
   * @param y1 Ending Y coordinate of the line.
   * @param size_x Width of the occupancy grid map.
   * @param state The state to set the cells along the line to (occupied or free).
   * @param max_length Maximum length of the line.
   */
  inline void GroundSegmentor::raytraceLine(
      nav_msgs::msg::OccupancyGrid &map, unsigned int x0, unsigned int y0, unsigned int x1,
      unsigned int y1, int size_x, CellState state, unsigned int max_length)
  {
    int dx = x1 - x0;
    int dy = y1 - y0;
    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x;

    unsigned int offset = y0 * size_x + x0;

    // Calculate the distance between the start and end points of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));

    // Scale the line based on the maximum length
    double scale = std::min(1.0, max_length / dist);

    // Determine if x or y is the dominant dimension
    if (abs_dx >= abs_dy) // if x is dominant
    {
      int error_y = abs_dx / 2;
      // Perform Bresenham's algorithm for the dominant dimension
      bresenham2D(
          map, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, state, (unsigned int)(scale * abs_dx));
      return;
    }

    // Otherwise y is dominant
    int error_x = abs_dy / 2;
    // Perform Bresenham's algorithm for the dominant dimension
    bresenham2D(
        map, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, state, (unsigned int)(scale * abs_dy));
  }

  /**
   * @brief Converts world coordinates to map coordinates.
   *
   * This function takes world coordinates (wx, wy) and computes their corresponding
   * map coordinates (mx, my) based on the resolution of the grid map.
   *
   * @param wx World X coordinate.
   * @param wy World Y coordinate.
   * @param mx Reference to store the map X coordinate.
   * @param my Reference to store the map Y coordinate.
   * @return True if conversion is successful and the resulting map coordinates are within the map boundaries, false otherwise.
   */
  bool GroundSegmentor::worldToMap(
      double wx, double wy, unsigned int &mx, unsigned int &my) const
  {
    // Check if the world coordinates are within the map boundary
    if (wx < bottomright_x_ || wy < bottomright_y_)
    {
      return false; // World coordinates are outside the map
    }

    // Calculate map coordinates based on world coordinates and grid resolution
    mx = static_cast<int>(std::floor(fabs((wx - (bottomright_x_)) / grid_msg_.info.resolution)));
    my = static_cast<int>(std::floor(fabs((wy - (bottomright_y_)) / grid_msg_.info.resolution)));

    // Check if the resulting map coordinates are within the map boundary
    if (mx < cell_num_x_ && my < cell_num_y_)
    {
      return true; // Conversion successful and map coordinates are within map boundaries
    }
    return false; // Map coordinates are outside the map
  }

  /**
   * @brief Perform ray tracing from a source point to a target point in the occupancy grid map.
   *
   * This function traces a ray from a source point (source_x, source_y) to a target point (target_x, target_y)
   * in the occupancy grid map and marks the cells along the ray as occupied, unknown, or free based on the provided state.
   *
   * @param map Reference to the occupancy grid map to perform ray tracing on.
   * @param source_x X coordinate of the source point.
   * @param source_y Y coordinate of the source point.
   * @param target_x X coordinate of the target point.
   * @param target_y Y coordinate of the target point.
   * @param size_x Width of the occupancy grid map.
   * @param state The state to set the cells along the ray to (occupied, unknown, or free).
   */
  void GroundSegmentor::raytrace(
      nav_msgs::msg::OccupancyGrid &map, double source_x, double source_y, double target_x, double target_y,
      int size_x, CellState state)
  {
    // Convert source point coordinates to map coordinates
    unsigned int x0{};
    unsigned int y0{};
    const double ox{source_x};
    const double oy{source_y};

    // Check if the source point is within the map boundary
    if (!worldToMap(ox, oy, x0, y0))
    {
      return;
    }

    // Pre-compute endpoints of the map outside of the inner loop for optimization
    const double origin_x = (bottomright_x_), origin_y = (bottomright_y_);
    const double map_end_x = -origin_x;
    const double map_end_y = -origin_y;

    // Adjust target point coordinates if they are outside the map boundary
    double wx = target_x;
    double wy = target_y;
    const double a = wx - ox;
    const double b = wy - oy;

    // Ensure that the target point is within the map boundary and scale if necessary
    if (wx < origin_x)
    {
      const double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      const double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }
    if (wx > map_end_x)
    {
      const double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      const double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // Convert target point coordinates to map coordinates
    unsigned int x1{};
    unsigned int y1{};
    if (!worldToMap(wx, wy, x1, y1))
    {
      return;
    }

    // Perform ray tracing from the source point to the target point
    raytraceLine(map, x0, y0, x1, y1, size_x, state);
  }

  /**
   * @brief Lambda function to determine if a point is visible beyond an obstacle.
   *
   * This lambda function takes two BinInfo3D objects representing an obstacle and a raw point,
   * and determines if the raw point is visible beyond the obstacle.
   *
   * @param obstacle Information about the obstacle.
   * @param raw Information about the raw point.
   * @return True if the raw point is visible beyond the obstacle, false otherwise.
   */
  auto is_visible_beyond_obstacle = [](const BinInfo3D &obstacle, const BinInfo3D &raw) -> bool
  {
    // Check if the raw point is closer to the sensor than the obstacle
    if (raw.range < obstacle.range)
    {
      return false;
    }

    // Check if the obstacle has zero projection length (invalid)
    if (obstacle.projection_length == 0)
    {
      return false;
    }

    // Calculate the coefficients of the line equation y = ax + b, representing the obstacle's surface
    const double a = -(origin_z_ - robot_z_) / (obstacle.range + obstacle.projection_length);
    const double b = origin_z_;

    // Check if the raw point's vertical position (wz) is above the obstacle's surface
    return raw.wz > (a * raw.range + b);
  };

  void GroundSegmentor::synchro_callback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_pcl_msg_1,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_pcl_msg_2)
  {
    // Declare PointCloud objects to store raw and obstacle point cloud data
    pcl::PointCloud<pcl::PointXYZI> raw_pcl;
    pcl::PointCloud<pcl::PointXYZI> obs_pcl;

    // Convert ROS PointCloud2 messages to PointCloud objects
    pcl::fromROSMsg(*input_pcl_msg_1, raw_pcl);
    pcl::fromROSMsg(*input_pcl_msg_2, obs_pcl);

    // Declare vectors to store angle bins for raw and obstacle point clouds
    std::vector<std::vector<BinInfo3D>> raw_pointcloud_angle_bins_;
    std::vector<std::vector<BinInfo3D>> obstacle_pointcloud_angle_bins_;

    // Declare a PointCloud pointer to store non-NaN points
    pcl::PointCloud<pcl::PointXYZI>::Ptr non_nan_pcl(new pcl::PointCloud<pcl::PointXYZI>);

    // Set is_dense flag of raw point cloud to false to handle non-dense point clouds
    raw_pcl.is_dense = false;

    // Declare a vector to store indices of non-NaN points
    std::vector<int> ind;

    // Remove NaN points from the raw point cloud and store the result in non_nan_pcl
    pcl::removeNaNFromPointCloud(raw_pcl, *non_nan_pcl, ind);

    // Initialize a vector with default values of 50 representing grid points
    std::vector<signed char> grid_points_(grid_size_, 50);

    // Calculate the number of angle bins based on the specified angle range and increment
    const size_t angle_bin_size = ((max_angle_ - min_angle_) / angle_increment_) + size_t(1 /*margin*/);

    // Resize obstacle_pointcloud_angle_bins_ and raw_pointcloud_angle_bins_ vectors to accommodate angle bins
    obstacle_pointcloud_angle_bins_.resize(angle_bin_size);
    raw_pointcloud_angle_bins_.resize(angle_bin_size);

    // Divide the non-NaN raw points into bins based on their angle
    for (auto &point : non_nan_pcl->points)
    {
      // Calculate the angle and range of the point
      const double angle = atan2(point.y, point.x);
      const double range = std::hypot(point.y, point.x);

      // Determine the index of the angle bin for the point
      const int angle_bin_index = (angle - min_angle_) / angle_increment_;

      // Store the point in the corresponding angle bin
      raw_pointcloud_angle_bins_.at(angle_bin_index).emplace_back(range, point.x, point.y, point.z);
    }

    // Divide the obstacle point cloud into bins
    for (auto &point : obs_pcl.points)
    {
      // Calculate the angle and range of the point
      const double angle = atan2(point.y, point.x);
      const double range = std::hypot(point.y, point.x);

      // Determine the index of the angle bin for the point
      const int angle_bin_index = (angle - min_angle_) / angle_increment_;

      // Calculate the vertical distance from the lidar scan to the obstacle point
      const double scan_z = origin_z_ - robot_z_;
      const double obstacle_z = point.z - robot_z_;
      const double dz = scan_z - obstacle_z;

      // Check if the vertical distance meets the threshold for projection
      if (dz > projection_dz_threshold_)
      {
        // Calculate the ratio of obstacle height to vertical distance
        const double ratio = (obstacle_z / dz);

        // Calculate the length of the projection of the obstacle onto the scan plane
        const double projection_length = range * ratio;

        // Project the obstacle point onto the scan plane
        const double projected_wx = (point.x) + ((point.x)) * ratio;
        const double projected_wy = (point.y) + ((point.y)) * ratio;

        // Store the obstacle point with projection information in the corresponding angle bin
        obstacle_pointcloud_angle_bins_.at(angle_bin_index).emplace_back(range, point.x, point.y, point.z, projection_length, projected_wx, projected_wy);
      }
      else
      {
        // If the vertical distance is below the threshold, assume no projection and project to a far point
        const double projected_wx = (point.x) + ((point.x)) * 100000000000000000; // Multiplied by big number to insure that it reaches the boarders of the map
        const double projected_wy = (point.y) + ((point.y)) * 100000000000000000; // Multiplied by big number to insure that it reaches the boarders of the map

        // Store the obstacle point with zero projection length in the corresponding angle bin
        obstacle_pointcloud_angle_bins_.at(angle_bin_index).emplace_back(range, point.x, point.y, point.z, 0, projected_wx, projected_wy);
      }
    }

    // Sort raw points within each bin
    for (auto &raw_pointcloud_angle_bin : raw_pointcloud_angle_bins_)
    {
      std::sort(raw_pointcloud_angle_bin.begin(), raw_pointcloud_angle_bin.end(), [](auto a, auto b)
                { return a.range < b.range; });
    }

    // Sort obs points within each bin
    for (auto &obstacle_pointcloud_angle_bin : obstacle_pointcloud_angle_bins_)
    {
      std::sort(obstacle_pointcloud_angle_bin.begin(), obstacle_pointcloud_angle_bin.end(), [](auto a, auto b)
                { return a.range < b.range; });
    }
    grid_msg_.data = grid_points_;

    // Apply raytracing
    for (size_t j = 0; j < angle_bin_size - 1; j++)
    {
      if (raw_pointcloud_angle_bins_[j].size() < 1)
        continue;

      const auto &source = raw_pointcloud_angle_bins_[j].back();
      raytrace(grid_msg_, 0, 0, source.wx, source.wy, cell_num_x_, CellState::FREE);
    }

    // Second step: Add unknown cell
    for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins_.size(); ++bin_index)
    {
      // Retrieve obstacle and raw point cloud angle bins for the current angle bin index
      const auto &obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins_.at(bin_index);
      const auto &raw_pointcloud_angle_bin = raw_pointcloud_angle_bins_.at(bin_index);

      // Initialize iterator for raw point cloud angle bin
      auto raw_distance_iter = raw_pointcloud_angle_bin.begin();

      // Iterate through the obstacle point cloud angle bin
      for (size_t dist_index = 0; dist_index < obstacle_pointcloud_angle_bin.size(); ++dist_index)
      {
        // Skip the last point in the obstacle bin
        if (dist_index + 1 == obstacle_pointcloud_angle_bin.size())
          continue;

        // Retrieve the current obstacle point
        const auto &obstacle_bin = obstacle_pointcloud_angle_bin.at(dist_index);

        // Find the next visible raw point beyond the obstacle
        while (raw_distance_iter != raw_pointcloud_angle_bin.end())
        {
          if (!is_visible_beyond_obstacle(obstacle_bin, *raw_distance_iter))
            raw_distance_iter++;
          else
            break;
        }

        // Check if there is no visible point beyond the obstacle
        const bool no_visible_point_beyond = (raw_distance_iter == raw_pointcloud_angle_bin.end());

        // If no visible point beyond, perform ray tracing to mark unknown cells
        if (no_visible_point_beyond)
        {
          const auto &source = obstacle_pointcloud_angle_bin.at(dist_index);
          raytrace(grid_msg_, source.wx, source.wy, source.projected_wx, source.projected_wy, cell_num_x_, CellState::UNKNOWN);
          break;
        }

        // If the current obstacle point is the last point, perform ray tracing to mark unknown cells
        if (dist_index + 1 == obstacle_pointcloud_angle_bin.size())
        {
          const auto &source = obstacle_pointcloud_angle_bin.at(dist_index);
          if (!no_visible_point_beyond)
          {
            raytrace(grid_msg_, source.wx, source.wy, source.projected_wx, source.projected_wy, cell_num_x_, CellState::UNKNOWN);
          }
          continue;
        }

        // Calculate the distance between the next two obstacle points
        auto next_obstacle_point_distance = std::abs(
            obstacle_pointcloud_angle_bin.at(dist_index + 1).range -
            obstacle_pointcloud_angle_bin.at(dist_index).range);

        // If the distance between the next two obstacle points is less than the separation threshold, continue
        if (next_obstacle_point_distance <= obstacle_separation_threshold_)
        {
          continue;
        }
        // If no visible point beyond and not the last point, perform ray tracing between the current and next obstacle points
        else if (no_visible_point_beyond)
        {
          const auto &source = obstacle_pointcloud_angle_bin.at(dist_index);
          const auto &target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
          raytrace(grid_msg_, source.wx, source.wy, target.wx, target.wy, cell_num_x_, CellState::UNKNOWN);
          continue;
        }
        // If there is a visible point beyond, perform ray tracing between the current obstacle point and the visible raw point
        auto next_raw_distance =
            std::abs(obstacle_pointcloud_angle_bin.at(dist_index).range - raw_distance_iter->range);
        if (next_raw_distance < next_obstacle_point_distance)
        {
          const auto &source = obstacle_pointcloud_angle_bin.at(dist_index);
          const auto &target = *raw_distance_iter;
          raytrace(grid_msg_, source.wx, source.wy, target.wx, target.wy, cell_num_x_, CellState::UNKNOWN);
          continue;
        }
        // Otherwise, perform ray tracing between the current and next obstacle points
        else
        {
          const auto &source = obstacle_pointcloud_angle_bin.at(dist_index);
          const auto &target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
          raytrace(grid_msg_, source.wx, source.wy, target.wx, target.wy, cell_num_x_, CellState::UNKNOWN);
          continue;
        }
      }
    }

    // Third step: Overwrite occupied cell
    for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins_.size(); ++bin_index)
    {
      // Retrieve obstacle point cloud angle bin for the current angle bin index
      auto &obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins_.at(bin_index);

      // Iterate through the obstacle point cloud angle bin
      for (size_t dist_index = 0; dist_index < obstacle_pointcloud_angle_bin.size(); ++dist_index)
      {
        // Skip the last point in the angle bin
        if (dist_index + 1 == obstacle_pointcloud_angle_bin.size())
        {
          continue;
        }

        // Calculate the distance between the next two obstacle points
        auto next_obstacle_point_distance = std::abs(
            obstacle_pointcloud_angle_bin.at(dist_index + 1).range -
            obstacle_pointcloud_angle_bin.at(dist_index).range);

        // If the distance between the next two obstacle points is less than the separation threshold,
        // overwrite cells between them as occupied
        if (next_obstacle_point_distance < obstacle_separation_threshold_)
        {
          const auto &source = obstacle_pointcloud_angle_bin.at(dist_index);
          const auto &target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
          raytrace(grid_msg_, source.wx, source.wy, target.wx, target.wy, cell_num_x_, CellState::OCCUPIED);
          continue;
        }
      }
    }

    grid_msg_.header.set__stamp(input_pcl_msg_1->header.stamp);
    grid_msg_.header.frame_id = input_pcl_msg_1->header.frame_id;

    grid_publisher_->publish(grid_msg_);
  }

} // namespace PerceptionNS

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto occupancy_grid_map_node =
      std::make_shared<PerceptionNS::GroundSegmentor>(rclcpp::NodeOptions());

  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(occupancy_grid_map_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}