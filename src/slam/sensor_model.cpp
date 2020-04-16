#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    double scanScore = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    for (const auto& ray : movingScan) {
        double rayScore = scoreRay(ray, map);
        scanScore += rayScore;
    }
    return scanScore;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map) {
   
    Point<float> ray_start = global_position_to_grid_position(ray.origin, map);
    Point<int> ray_cell;
    float fraction = 1.0 / 3;
    ray_cell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + ray_start.x);
    ray_cell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + ray_start.y);
    int8_t cellOdds = map.logOdds(ray_cell.x, ray_cell.y);
    if (cellOdds > 0.0) {
        return (cellOdds + 127);
    } else {
        Point<int> cell_down = raster_line(ray_cell.x, ray_cell.y, ray_start.x, ray_start.y);
        cellOdds = map.logOdds(cell_down.x, cell_down.y);
        if (cellOdds > 0.0) {
            return (cellOdds + 127) * fraction;
        }
        Point<int> cell_up = raster_line(ray_cell.x, ray_cell.y, ray_cell.x + ray_start.x, ray_cell.x + ray_start.y);
        cellOdds = map.logOdds(cell_up.x, cell_up.y);
        if (cellOdds > 0.0) {
            return (cellOdds + 127) * fraction;
        }
    }
    return 0.0;
}

Point<int> SensorModel::raster_line(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;
    float e2 = 2 * err;
    if (e2 >= -dy) {
        err -= dy;
        x += sx;
    }
    if (e2 <= dx) {
        err += dx;
        y += sy;
    }
    Point<int> cell_coord;
    cell_coord.x = x;
    cell_coord.y = y;
    return cell_coord;
}