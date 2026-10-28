#include "mppi_local_planner/ego_costmap.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace mppi_local_planner
{

namespace
{

struct EgoPt
{
  float x;
  float y;
  float z;
};

// z = a + b x + c y. ok == false → caller keeps the fixed z slab.
struct GroundPlane
{
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  bool ok = false;

  double zAt(double x, double y) const
  {
    return a + b * x + c * y;
  }
};

struct GroundBin
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

bool solve3(double m[3][4], double & a, double & b, double & c)
{
  for (int col = 0; col < 3; ++col) {
    int piv = col;
    for (int r = col + 1; r < 3; ++r) {
      if (std::abs(m[r][col]) > std::abs(m[piv][col])) {
        piv = r;
      }
    }
    if (std::abs(m[piv][col]) < 1e-8) {
      return false;
    }
    if (piv != col) {
      for (int k = col; k < 4; ++k) {
        std::swap(m[col][k], m[piv][k]);
      }
    }
    const double div = m[col][col];
    for (int k = col; k < 4; ++k) {
      m[col][k] /= div;
    }
    for (int r = 0; r < 3; ++r) {
      if (r == col) {
        continue;
      }
      const double f = m[r][col];
      for (int k = col; k < 4; ++k) {
        m[r][k] -= f * m[col][k];
      }
    }
  }
  a = m[0][3];
  b = m[1][3];
  c = m[2][3];
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

bool fitThree(const GroundBin & p, const GroundBin & q, const GroundBin & r,
              double & a, double & b, double & c)
{
  double m[3][4] = {
    {1.0, p.x, p.y, p.z},
    {1.0, q.x, q.y, q.z},
    {1.0, r.x, r.y, r.z}};
  return solve3(m, a, b, c);
}

bool fitLeastSquares(const std::vector<GroundBin> & bins, const std::vector<int> & id,
                     double & a, double & b, double & c)
{
  if (id.size() < 6) {
    return false;
  }
  double n = 0.0;
  double sx = 0.0, sy = 0.0, sz = 0.0;
  double sxx = 0.0, syy = 0.0, sxy = 0.0;
  double sxz = 0.0, syz = 0.0;
  for (int i : id) {
    const GroundBin & p = bins[static_cast<size_t>(i)];
    n += 1.0;
    sx += p.x;
    sy += p.y;
    sz += p.z;
    sxx += static_cast<double>(p.x) * p.x;
    syy += static_cast<double>(p.y) * p.y;
    sxy += static_cast<double>(p.x) * p.y;
    sxz += static_cast<double>(p.x) * p.z;
    syz += static_cast<double>(p.y) * p.z;
  }
  double m[3][4] = {
    {n, sx, sy, sz},
    {sx, sxx, sxy, sxz},
    {sy, sxy, syy, syz}};
  return solve3(m, a, b, c);
}

// Lowest return in each 1 m cell, then a plane under those lows.
// A cone-only cloud (no surface near -sensor_height) is rejected so the
// fixed slab still marks the cones.
GroundPlane fitGroundPlane(const std::vector<EgoPt> & pts, const CostmapParams & params)
{
  GroundPlane none;
  constexpr double kCell = 1.0;
  constexpr float kInlier = 0.12f;
  struct Acc
  {
    float z = 1.0e9f;
    float x = 0.0f;
    float y = 0.0f;
    int n = 0;
  };
  std::unordered_map<uint64_t, Acc> grid;
  grid.reserve(pts.size() / 8 + 1);
  for (const EgoPt & p : pts) {
    const int ix = static_cast<int>(std::floor(p.x / kCell));
    const int iy = static_cast<int>(std::floor(p.y / kCell));
    const uint64_t key =
      (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
      static_cast<uint32_t>(iy);
    Acc & a = grid[key];
    ++a.n;
    if (p.z < a.z) {
      a.z = p.z;
      a.x = p.x;
      a.y = p.y;
    }
  }
  std::vector<GroundBin> bins;
  bins.reserve(grid.size());
  for (const auto & kv : grid) {
    if (kv.second.n < 4) {
      continue;
    }
    bins.push_back(GroundBin{kv.second.x, kv.second.y, kv.second.z});
  }
  const int n = static_cast<int>(bins.size());
  if (n < 10) {
    return none;
  }

  auto countInliers = [&](double a, double b, double c, float tol) {
    int hit = 0;
    for (const GroundBin & p : bins) {
      if (std::abs(p.z - (a + b * p.x + c * p.y)) <= tol) {
        ++hit;
      }
    }
    return hit;
  };

  double best_a = 0.0, best_b = 0.0, best_c = 0.0;
  int best_hit = 0;
  const int trials = std::min(48, n);
  for (int t = 0; t < trials; ++t) {
    const int i = (t * 17) % n;
    const int j = (i + n / 3) % n;
    const int k = (i + (2 * n) / 3) % n;
    if (i == j || j == k || i == k) {
      continue;
    }
    double a = 0.0, b = 0.0, c = 0.0;
    if (!fitThree(bins[static_cast<size_t>(i)], bins[static_cast<size_t>(j)],
                  bins[static_cast<size_t>(k)], a, b, c))
    {
      continue;
    }
    if (std::abs(b) > 0.45 || std::abs(c) > 0.45) {
      continue;
    }
    const int hit = countInliers(a, b, c, kInlier);
    if (hit > best_hit) {
      best_hit = hit;
      best_a = a;
      best_b = b;
      best_c = c;
    }
  }
  if (best_hit < 8) {
    return none;
  }

  std::vector<int> inliers;
  inliers.reserve(static_cast<size_t>(best_hit));
  for (int i = 0; i < n; ++i) {
    const GroundBin & p = bins[static_cast<size_t>(i)];
    if (std::abs(p.z - (best_a + best_b * p.x + best_c * p.y)) <= kInlier) {
      inliers.push_back(i);
    }
  }
  double a = best_a, b = best_b, c = best_c;
  if (!fitLeastSquares(bins, inliers, a, b, c)) {
    return none;
  }
  if (std::abs(b) > 0.36 || std::abs(c) > 0.36) {
    return none;
  }
  // Mount height may be off by about half a metre. A cloud of only cones
  // (sim obstacles at z ≈ -0.5, no ground) sits well above -sensor_height
  // and must fall back to the slab, or the plane eats the cones.
  const double nominal = -std::max(0.3, params.sensor_height_m);
  if (std::abs(a - nominal) > 0.50) {
    return none;
  }
  float xmin = 1.0e9f, xmax = -1.0e9f;
  int kept = 0;
  for (int i : inliers) {
    const GroundBin & p = bins[static_cast<size_t>(i)];
    if (std::abs(p.z - (a + b * p.x + c * p.y)) > 0.15f) {
      continue;
    }
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ++kept;
  }
  if (kept < 8 || (xmax - xmin) < 3.0f) {
    return none;
  }
  GroundPlane gp;
  gp.a = a;
  gp.b = b;
  gp.c = c;
  gp.ok = true;
  return gp;
}

}  // namespace

EgoCostmap::EgoCostmap(const CostmapParams & params)
: params_(params)
{
  cells_x_ = static_cast<int>(std::round(params_.size_x / params_.resolution));
  cells_y_ = static_cast<int>(std::round(params_.size_y / params_.resolution));
  occupancy_.assign(static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_), 0.0f);
  cost_.assign(static_cast<size_t>(cells_x_) * static_cast<size_t>(cells_y_), 0.0f);
}

void EgoCostmap::setFilterParams(
  double ground_z_min, double ground_z_max, double inflation_radius,
  double sensor_height_m, double roi_agl_min, double roi_agl_max,
  double pitch_angle_deg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  params_.ground_z_min = ground_z_min;
  params_.ground_z_max = ground_z_max;
  params_.sensor_height_m = std::max(0.3, sensor_height_m);
  params_.roi_agl_min = std::max(0.05, roi_agl_min);
  params_.roi_agl_max = std::max(params_.roi_agl_min + 0.2, roi_agl_max);
  params_.pitch_angle_deg = pitch_angle_deg;
  if (inflation_radius > 0.05) {
    params_.inflation_radius = inflation_radius;
  }
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
  snap.occ = occ_bin_;
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
  const double pitch_rad = params_.pitch_angle_deg * M_PI / 180.0;
  const double cos_pitch = std::cos(pitch_rad);
  const double sin_pitch = std::sin(pitch_rad);
  const bool has_pitch = std::abs(params_.pitch_angle_deg) > 1e-4;

  // Keep ground returns in this pass. The fixed slab drops them only when the
  // plane fit fails. Sky and deep underground never enter the fit.
  std::vector<EgoPt> pts;
  pts.reserve(static_cast<size_t>(cloud.width) * static_cast<size_t>(std::max(1u, cloud.height)));
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const float xs = *iter_x;
    const float ys = *iter_y;
    const float zs = *iter_z;
    if (!std::isfinite(xs) || !std::isfinite(ys) || !std::isfinite(zs)) {
      continue;
    }
    if (zs < -3.5f || zs > 2.5f) {
      continue;
    }
    double x_rot = xs * cos_off - ys * sin_off;
    double y_rot = xs * sin_off + ys * cos_off;
    double z_rot = static_cast<double>(zs);

    // ★ 전방 기울어짐(Pitch) 보정 ★
    // 센서가 전방으로 숙여진 각도(+)만큼 좌표계를 수평으로 회전 복원
    if (has_pitch) {
      const double x_corr =  x_rot * cos_pitch + z_rot * sin_pitch;
      const double z_corr = -x_rot * sin_pitch + z_rot * cos_pitch;
      x_rot = x_corr;
      z_rot = z_corr;
    }

    const double x = params_.sensor_offset_x + x_rot;
    const double y = params_.sensor_offset_y + y_rot;
    const float z = static_cast<float>(z_rot);
    if ((x * x + y * y) < params_.min_range * params_.min_range) {
      continue;
    }
    if (x < -params_.size_x / 2.0 || x >= params_.size_x / 2.0 ||
        y < -params_.size_y / 2.0 || y >= params_.size_y / 2.0)
    {
      continue;
    }
    pts.push_back(EgoPt{static_cast<float>(x), static_cast<float>(y), z});
  }

  const GroundPlane gp = fitGroundPlane(pts, params_);
  for (const EgoPt & p : pts) {
    bool keep = false;
    if (gp.ok) {
      const double agl = static_cast<double>(p.z) - gp.zAt(p.x, p.y);
      keep = agl >= params_.roi_agl_min && agl <= params_.roi_agl_max;
    } else if (p.z >= params_.ground_z_min && p.z <= params_.ground_z_max) {
      keep = true;
    }
    if (!keep) {
      continue;
    }
    const int ix = worldToIndex(p.x, params_.size_x, params_.resolution);
    const int iy = worldToIndex(p.y, params_.size_y, params_.resolution);
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
  //  ★훑는 범위는 둘 중 넓은 쪽★ 안 그러면 passed 쪽 넓은 반폭이 잘려 나간다.
  const double y_scan = std::max(params_.ego_clear_y_half,
                                 params_.ego_clear_y_half_passed);
  const int iy0 = std::max(0, worldToIndex(-y_scan, params_.size_y, res));
  const int iy1 = std::min(cells_y_ - 1, worldToIndex(y_scan, params_.size_y, res));
  for (int iy = iy0; iy <= iy1; ++iy) {
    for (int ix = ix0; ix <= ix1; ++ix) {
      const double wx = -params_.size_x / 2.0 + (ix + 0.5) * res;
      const double wy = -params_.size_y / 2.0 + (iy + 0.5) * res;
      //  ★pass_x 뒤에서만 더 넓게 지운다★ (헤더 주석의 실측 근거)
      //  지나친 콘은 조향으로 피할 대상이 아닌데, 남아 있으면 ★복귀하는 쪽★
      //  롤아웃만 비용을 먹어 차가 영영 돌아오지 못한다. pass_x 가 앞차축보다
      //  앞이면 아직 피해야 할 콘까지 지운다.
      const bool passed = (params_.ego_clear_pass_x > 0.0 &&
                           params_.ego_clear_y_half_passed > params_.ego_clear_y_half &&
                           wx <= params_.ego_clear_pass_x);
      const double y_half = passed ? params_.ego_clear_y_half_passed
                                   : params_.ego_clear_y_half;
      if (wx >= params_.ego_clear_x_min && wx <= params_.ego_clear_x_max &&
          std::abs(wy) <= y_half)
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
  std::vector<uint8_t> occ_bin(occ_copy.size(), 0);
  const double res = params_.resolution;
  const double lethal_r = params_.robot_half_width;
  const double total_r = params_.robot_half_width + params_.inflation_radius;
  const int reach = static_cast<int>(std::ceil(total_r / res));

  for (int oy = 0; oy < cells_y_; ++oy) {
    for (int ox = 0; ox < cells_x_; ++ox) {
      const size_t oi = static_cast<size_t>(oy) * static_cast<size_t>(cells_x_) + static_cast<size_t>(ox);
      if (occ_copy[oi] < 0.5f) {
        continue;
      }
      occ_bin[oi] = 1;
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
  occ_bin_ = std::move(occ_bin);   // ★cost_ 와 같은 스캔★ — 군집이 치사층과 어긋나지 않게
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
