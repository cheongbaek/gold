#include "mppi_local_planner/cone_detector.hpp"

#include <algorithm>
#include <cstdint>

namespace mppi_local_planner
{

std::vector<ObstacleCluster> clusterOccupancy(
  const CostmapSnapshot & snap,
  const ClusterParams & params,
  ConeStats * stats)
{
  std::vector<ObstacleCluster> out;
  if (stats) {
    *stats = ConeStats{};
  }
  if (!snap.valid || snap.cells_x <= 0 || snap.cells_y <= 0) {
    return out;
  }
  const size_t n =
    static_cast<size_t>(snap.cells_x) * static_cast<size_t>(snap.cells_y);
  if (stats && snap.cost.size() == n) {
    const float thr = EgoCostmap::kLethalCost * 0.99f;
    for (const float c : snap.cost) {
      if (c >= thr) {
        ++stats->lethal_cells;
      }
    }
  }
  if (snap.occ.size() != n) {
    return out;
  }
  const double res = snap.resolution;
  auto cellX = [&](int ix) { return -snap.size_x / 2.0 + (ix + 0.5) * res; };
  auto cellY = [&](int iy) { return -snap.size_y / 2.0 + (iy + 0.5) * res; };
  const int ix0 = std::max(0, static_cast<int>(std::floor((params.x_min + snap.size_x / 2.0) / res)));
  const int ix1 = std::min(snap.cells_x - 1,
                           static_cast<int>(std::floor((params.x_max + snap.size_x / 2.0) / res)));
  const int iy0 = std::max(0, static_cast<int>(std::floor((-params.y_abs_max + snap.size_y / 2.0) / res)));
  const int iy1 = std::min(snap.cells_y - 1,
                           static_cast<int>(std::floor((params.y_abs_max + snap.size_y / 2.0) / res)));
  if (ix1 < ix0 || iy1 < iy0) {
    return out;
  }
  // ROI 안의 점유 셀만 라벨을 붙인다 (-1 = 점유·미방문, -2 = 비어 있음).
  const int w = ix1 - ix0 + 1;
  const int h = iy1 - iy0 + 1;
  std::vector<int> label(static_cast<size_t>(w) * static_cast<size_t>(h), -2);
  for (int iy = iy0; iy <= iy1; ++iy) {
    for (int ix = ix0; ix <= ix1; ++ix) {
      if (snap.occupied(ix, iy)) {
        label[static_cast<size_t>(iy - iy0) * static_cast<size_t>(w) +
              static_cast<size_t>(ix - ix0)] = -1;
      }
    }
  }
  const int k = std::max(1, params.link_cells);
  std::vector<int> stack;
  int next_id = 0;
  for (int ly = 0; ly < h; ++ly) {
    for (int lx = 0; lx < w; ++lx) {
      const size_t i0 = static_cast<size_t>(ly) * static_cast<size_t>(w) + static_cast<size_t>(lx);
      if (label[i0] != -1) {
        continue;
      }
      const int id = next_id++;
      ObstacleCluster c;
      c.x0 = c.y0 = 1e9;
      c.x1 = c.y1 = -1e9;
      double sx = 0.0;
      double sy = 0.0;
      stack.clear();
      stack.push_back(static_cast<int>(i0));
      label[i0] = id;
      while (!stack.empty()) {
        const int cur = stack.back();
        stack.pop_back();
        const int cx = cur % w;
        const int cy = cur / w;
        const double wx = cellX(cx + ix0);
        const double wy = cellY(cy + iy0);
        sx += wx;
        sy += wy;
        c.x0 = std::min(c.x0, wx - 0.5 * res);
        c.x1 = std::max(c.x1, wx + 0.5 * res);
        c.y0 = std::min(c.y0, wy - 0.5 * res);
        c.y1 = std::max(c.y1, wy + 0.5 * res);
        ++c.cells;
        for (int dy = -k; dy <= k; ++dy) {
          const int ny = cy + dy;
          if (ny < 0 || ny >= h) {
            continue;
          }
          for (int dx = -k; dx <= k; ++dx) {
            const int nx = cx + dx;
            if (nx < 0 || nx >= w) {
              continue;
            }
            const size_t ni = static_cast<size_t>(ny) * static_cast<size_t>(w) + static_cast<size_t>(nx);
            if (label[ni] != -1) {
              continue;
            }
            label[ni] = id;
            stack.push_back(static_cast<int>(ni));
          }
        }
      }
      c.cx = sx / static_cast<double>(c.cells);
      c.cy = sy / static_cast<double>(c.cells);
      if (stats) {
        ++stats->clusters_all;
      }
      if (c.cells < std::max(1, params.min_cells) || (c.y1 - c.y0) > params.max_width_m) {
        if (stats) {
          ++stats->clusters_rejected;
        }
        continue;
      }
      out.push_back(c);
    }
  }
  std::sort(out.begin(), out.end(),
            [](const ObstacleCluster & a, const ObstacleCluster & b) { return a.x0 < b.x0; });
  return out;
}

}  // namespace mppi_local_planner
