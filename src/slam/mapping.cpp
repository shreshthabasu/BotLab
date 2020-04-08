#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>



Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (! initialized_) {
        prev_pose_ = pose;
    }

    MovingLaserScan moving_scan(scan, prev_pose_, pose);
     for (auto& ray : moving_scan) {
         scoreEndPoint(ray, map);
     }
     for (auto& ray : moving_scan) {
         scoreRay(ray, map);
     }

     initialized_ = true;
     prev_pose_ = pose;
}

void Mapping::scoreEndPoint(const adjusted_ray_t& ray, OccupancyGrid& map) {
    if (ray.range <= kMaxLaserDistance_) {
        Point<float> ray_start = global_position_to_grid_position(ray.origin, map);
        Point<int> ray_cell;
        ray_cell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + ray_start.x);
        ray_cell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + ray_start.y);
        if (map.isCellInGrid(ray_cell.x, ray_cell.y)) {
            increaseCellOdds(ray_cell.x, ray_cell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map) {
    if (ray.range <= kMaxLaserDistance_) {
        Point<float> ray_start = global_position_to_grid_position(ray.origin, map);
        Point<int> ray_end;
        ray_end.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + ray_start.x);
        ray_end.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + ray_start.y);
        raster_line(ray_start.x, ray_start.y, ray_end.x, ray_end.y, map);
    }
}

void Mapping::increaseCellOdds(int x, int y , OccupancyGrid& map) {
    if (! initialized_) {

    } else if (std::numeric_limits<CellOdds>::max() - map(x,y) > kHitOdds_) {
        map(x,y) += kHitOdds_;
    } else {
         map(x,y) = std::numeric_limits<CellOdds>::max();
    }
}

void Mapping::decreaseCellOdds(int x, int y , OccupancyGrid& map) {
    if (! initialized_) {
    } else if (map(x,y) - kMissOdds_ > std::numeric_limits<CellOdds>::min()) {
        map(x,y) -= kMissOdds_;
    } else {
         map(x,y) = std::numeric_limits<CellOdds>::min();
    }
}


void Mapping::raster_line(int x0, int y0, int x1, int y1, OccupancyGrid& map) {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;
    while (x != x1 || y != y1) {
        if (map.isCellInGrid(x, y)) {
            decreaseCellOdds(x, y, map);
        }
        float e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
}