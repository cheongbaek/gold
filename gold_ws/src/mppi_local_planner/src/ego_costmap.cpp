#include "mppi_local_planner/ego_costmap.hpp"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace mppi_local_planner
{

EgoCostmap::EgoCostmap(const CostmapParams & params)
: params_(params)
{
  cells_x_ = static_cast<int>(std::round(params_.size_x / params_.resolution));
  cells_y_ = static_cast<int>(std::round(params_.size_y / params_.resolution));
  occupancy_.assign(static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_), 0.0f);
  cost_.assign(static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_), 0.0f);
}

int EgoCostmap::worldToIndex(double val, double size, double resolution) const
{
  return static_cast<int>(std::floor((val + size / 2.0) / resolution));
}

bool EgoCostmap::hasData() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_data_;
}

CostmapSnapshot EgoCostmap::snapshot() const
{
  CostmapSnapshot snap;
  std::lock_guard<std::mutex> lock(mutex_);
  snap.cost = cost_;
  snap.cells_x = cells_x_;
  snap.cells_y = cells_y_;
  snap.size_x = params_.size_x;
  snap.size_y = params_.size_y;
  snap.resolution = params_.resolution;
  snap.valid = has_data_;
  return snap;
}

void EgoCostmap::updateFromPointCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<float> occ_new(
    static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_), 0.0f);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  const double cos_off = std::cos(params_.sensor_yaw_offset);
  const double sin_off = std::sin(params_.sensor_yaw_offset);

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float xs = *iter_x;
    const float ys = *iter_y;
    const float zs = *iter_z;

    if (!std::isfinite(xs) || !std::isfinite(ys) || !std::isfinite(zs)) {
      continue;
    }
    // Ground / overhang height filter, done in the raw sensor frame.
    if (zs < params_.ground_z_min || zs > params_.ground_z_max) {
      continue;
    }

    // Apply static sensor -> base/ego frame extrinsics (translation + yaw only).
    const double x = params_.sensor_offset_x + xs * cos_off - ys * sin_off;
    const double y = params_.sensor_offset_y + xs * sin_off + ys * cos_off;

    if ((x * x + y * y) < params_.min_range * params_.min_range) {
      continue;
    }

    if (x < -params_.size_x / 2.0 || x >= params_.size_x / 2.0 ||
        y < -params_.size_y / 2.0 || y >= params_.size_y / 2.0)
    {
      continue;
    }

    const int ix = worldToIndex(x, params_.size_x, params_.resolution);
    const int iy = worldToIndex(y, params_.size_y, params_.resolution);
    if (ix < 0 || ix >= cells_x_ || iy < 0 || iy >= cells_y_) {
      continue;
    }
    occ_new[static_cast<size_t>(iy) * static_cast<size_t>(cells_x_) + static_cast<size_t>(ix)] = 1.0f;
  }

  // Drop self-hits only (tight circle + short rect). Large clear used to paint
  // a white free square over nearby 라바콘 and freeze the planner.
  clearEgoOccupancy(occ_new);

  // Temporal fusion with previous occupancy (reduces flicker / false clears).
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const float decay = static_cast<float>(std::clamp(params_.occupancy_decay, 0.0, 0.95));
    if (has_data_ && decay > 0.0f) {
      for (size_t i = 0; i < occupancy_.size(); ++i) {
        // Keep hit cells solid; free cells retain a decayed residue of past hits.
        occupancy_[i] = std::max(occ_new[i], occupancy_[i] * decay);
      }
    } else {
      occupancy_ = std::move(occ_new);
    }

    // Wipe residual self-hits only (not near cones just outside the body).
    clearEgoOccupancy(occupancy_);
    has_data_ = true;
  }
  inflate();
}

void EgoCostmap::clearCircle(
  std::vector<float> & grid, double radius) const
{
  if (grid.empty() || radius <= 0.0) {
    return;
  }
  const double res = params_.resolution;
  const double r2 = radius * radius;
  const int reach = static_cast<int>(std::ceil(radius / res));
  const int cx = worldToIndex(0.0, params_.size_x, res);
  const int cy = worldToIndex(0.0, params_.size_y, res);
  for (int dy = -reach; dy <= reach; ++dy) {
    const int iy = cy + dy;
    if (iy < 0 || iy >= cells_y_) continue;
    for (int dx = -reach; dx <= reach; ++dx) {
      const int ix = cx + dx;
      if (ix < 0 || ix >= cells_x_) continue;
      const double wx = -params_.size_x / 2.0 + (ix + 0.5) * res;
      const double wy = -params_.size_y / 2.0 + (iy + 0.5) * res;
      if (wx * wx + wy * wy <= r2) {
        grid[static_cast<size_t>(iy) * static_cast<size_t>(cells_x_) +
             static_cast<size_t>(ix)] = 0.0f;
      }
    }
  }
}

void EgoCostmap::clearEgoOccupancy(std::vector<float> & grid) const
{
  if (grid.empty()) {
    return;
  }
  // 1) Mount / near-body circle
  clearCircle(grid, params_.ego_clear_radius);

  // 2) Short rectangular body pad (rear / sides / slight forward). x_max must
  // stay small (see ego_clear_front_max) so close cones remain in the map.
  const bool use_rect =
    params_.ego_clear_x_max > params_.ego_clear_x_min && params_.ego_clear_y_half > 0.0;
  if (!use_rect) {
    return;
  }
  const double res = params_.resolution;
  const int ix0 = std::max(0, worldToIndex(params_.ego_clear_x_min, params_.size_x, res));
  const int ix1 = std::min(cells_x_ - 1, worldToIndex(params_.ego_clear_x_max, params_.size_x, res));
  const int iy0 = std::max(0, worldToIndex(-params_.ego_clear_y_half, params_.size_y, res));
  const int iy1 = std::min(cells_y_ - 1, worldToIndex(params_.ego_clear_y_half, params_.size_y, res));
  for (int iy = iy0; iy <= iy1; ++iy) {
    for (int ix = ix0; ix <= ix1; ++ix) {
      const double wx = -params_.size_x / 2.0 + (ix + 0.5) * res;
      const double wy = -params_.size_y / 2.0 + (iy + 0.5) * res;
      if (wx >= params_.ego_clear_x_min && wx <= params_.ego_clear_x_max &&
          std::abs(wy) <= params_.ego_clear_y_half)
      {
        grid[static_cast<size_t>(iy) * static_cast<size_t>(cells_x_) +
             static_cast<size_t>(ix)] = 0.0f;
      }
    }
  }
}

void EgoCostmap::inflate()
{
  // Brute-force inflation: lethal disk of radius robot_half_width, plus a
  // linearly decaying ring out to inflation_radius.
  std::vector<float> occ_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    occ_copy = occupancy_;
  }

  std::vector<float> cost(static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_), 0.0f);
  const double res = params_.resolution;
  const double lethal_r = params_.robot_half_width;
  const double total_r = params_.robot_half_width + params_.inflation_radius;
  const int reach = static_cast<int>(std::ceil(total_r / res));

  for (int oy = 0; oy < cells_y_; ++oy) {
    for (int ox = 0; ox < cells_x_; ++ox) {
      if (occ_copy[static_cast<size_t>(oy) * static_cast<size_t>(cells_x_) + static_cast<size_t>(ox)] < 0.5f) {
        continue;
      }
      for (int dy = -reach; dy <= reach; ++dy) {
        const int ny = oy + dy;
        if (ny < 0 || ny >= cells_y_) continue;
        for (int dx = -reach; dx <= reach; ++dx) {
          const int nx = ox + dx;
          if (nx < 0 || nx >= cells_x_) continue;

          const double d = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * res;
          float c = 0.0f;
          if (d <= lethal_r) {
            c = kLethalCost;
          } else if (d <= total_r) {
            const double ratio = 1.0 - (d - lethal_r) / params_.inflation_radius;
            c = static_cast<float>(50.0 * ratio);
          }
          auto & cell = cost[static_cast<size_t>(ny) * static_cast<size_t>(cells_x_) + static_cast<size_t>(nx)];
          cell = std::max(cell, c);
        }
      }
    }
  }

  // Cost layer: ONLY a small circle for inflation bleed into the origin.
  // Re-applying the full body rect here was wiping cost of nearby 라바콘
  // (white free square in RViz) while point cloud still showed the cones.
  const double cost_r = (params_.ego_cost_clear_radius > 0.0) ?
    params_.ego_cost_clear_radius : params_.ego_clear_radius;
  clearCircle(cost, cost_r);

  std::lock_guard<std::mutex> lock(mutex_);
  cost_ = std::move(cost);
}

nav_msgs::msg::OccupancyGrid EgoCostmap::toOccupancyGrid() const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = static_cast<float>(params_.resolution);
  grid.info.width = static_cast<uint32_t>(cells_x_);
  grid.info.height = static_cast<uint32_t>(cells_y_);
  grid.info.origin.position.x = -params_.size_x / 2.0;
  grid.info.origin.position.y = -params_.size_y / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  std::lock_guard<std::mutex> lock(mutex_);
  grid.data.assign(static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_), -1);
  if (has_data_) {
    constexpr float kLethalViz = 999.0f;
    constexpr float kMaxGradient = 50.0f;
    for (size_t i = 0; i < cost_.size(); ++i) {
      const float c = cost_[i];
      int8_t v = 0;
      if (c >= kLethalViz) {
        v = 100;
      } else if (c > 0.0f) {
        v = static_cast<int8_t>(std::clamp(c / kMaxGradient * 99.0f, 1.0f, 99.0f));
      }
      grid.data[i] = v;
    }
  }
  return grid;
}

double EgoCostmap::getCost(double x, double y) const
{
  return snapshot().getCost(x, y);
}

}  // namespace mppi_local_planner
