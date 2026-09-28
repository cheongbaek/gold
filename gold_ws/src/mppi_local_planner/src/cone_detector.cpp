#include "mppi_local_planner/cone_detector.hpp"

#include <algorithm>
#include <cstdint>

namespace mppi_local_planner
{

std::vector<ConeObs> detectCones(
  const CostmapSnapshot & snap,
  const ConeDetectParams & params,
  ConeStats * stats)
{
  std::vector<ConeObs> out;
  if (stats) {
    *stats = ConeStats{};
  }
  if (!snap.valid || snap.cells_x <= 0 || snap.cells_y <= 0) {
    return out;
  }
  const double res = snap.resolution;
  const double thr = EgoCostmap::kLethalCost * 0.99;
  const size_t n =
    static_cast<size_t>(snap.cells_x) * static_cast<size_t>(snap.cells_y);
  std::vector<uint8_t> seen(n, 0);
  std::vector<int> stack;
  auto idx = [&](int ix, int iy) {
    return static_cast<size_t>(iy) * static_cast<size_t>(snap.cells_x) +
           static_cast<size_t>(ix);
  };
  for (int iy = 0; iy < snap.cells_y; ++iy) {
    for (int ix = 0; ix < snap.cells_x; ++ix) {
      const size_t i0 = idx(ix, iy);
      if (seen[i0] || snap.cost[i0] < thr) {
        continue;
      }
      double sx = 0.0;
      double sy = 0.0;
      double minx = 1e9;
      double maxx = -1e9;
      double miny = 1e9;
      double maxy = -1e9;
      int cnt = 0;
      stack.clear();
      stack.push_back(static_cast<int>(i0));
      seen[i0] = 1;
      while (!stack.empty()) {
        const int cur = stack.back();
        stack.pop_back();
        const int cx = cur % snap.cells_x;
        const int cy = cur / snap.cells_x;
        const double wx = -snap.size_x / 2.0 + (cx + 0.5) * res;
        const double wy = -snap.size_y / 2.0 + (cy + 0.5) * res;
        sx += wx;
        sy += wy;
        minx = std::min(minx, wx);
        maxx = std::max(maxx, wx);
        miny = std::min(miny, wy);
        maxy = std::max(maxy, wy);
        ++cnt;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || nx >= snap.cells_x || ny < 0 || ny >= snap.cells_y) {
              continue;
            }
            const size_t ni = idx(nx, ny);
            if (seen[ni] || snap.cost[ni] < thr) {
              continue;
            }
            seen[ni] = 1;
            stack.push_back(static_cast<int>(ni));
          }
        }
      }
      if (stats) {
        stats->lethal_cells += cnt;
        ++stats->clusters_all;
      }
      if (cnt >= params.min_cells) {
        const double span = std::max(maxx - minx, maxy - miny);
        if (span > params.max_span_m) {
          if (stats) {
            ++stats->clusters_rejected;
          }
        } else {
          out.push_back({sx / cnt, sy / cnt, cnt});
        }
      } else if (stats) {
        ++stats->clusters_rejected;
      }
    }
  }
  std::sort(out.begin(), out.end(),
            [](const ConeObs & a, const ConeObs & b) { return a.x < b.x; });
  return out;
}

}  // namespace mppi_local_planner
