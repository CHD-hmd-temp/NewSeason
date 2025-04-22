use crate::prelude::*;
use super::point_filter_2d::LaserFrame2D;
use bresenham::Bresenham;

const L_FREE:  f32 = -0.4;   // 自由区增量（减小 log‑odds）
const L_OCC:   f32 =  0.85;  // 占据区增量（增大 log‑odds）
const L_MIN:   f32 = -4.0;   // 截断下限
const L_MAX:   f32 =  4.0;   // 截断上限

pub struct OccupancyGrid {
    pub log_odds: Vec<f32>, // log odds of occupancy
    pub res: f32, // the size of each cell in meters
    pub width: usize,   // number of cells in the x direction
    pub height: usize,  // number of cells in the y direction
    pub origin: Point2f,    // the origin of the grid in world coordinates
}

impl OccupancyGrid {
    /// Map the global index to the local index
    fn world_to_grid(&self, wx: f32, wy: f32) -> Option<(usize, usize)> {
        let gx = ((wx - self.origin.x) / self.res).floor() as isize;
        let gy = ((wy - self.origin.y) / self.res).floor() as isize;
        if gx >= 0 && gx < self.width as isize && gy >= 0 && gy < self.height as isize {
            Some((gx as usize, gy as usize))
        } else {
            None
        }
    }

    /// Upgrade global map with a LaserFrame2D
    pub fn integrate_scan(
        &mut self,
        frame: &LaserFrame2D,
        yaw: f32, // degrees
    ) {
        let sensor_xy = Point2f::new(frame.sensor_pos.x, frame.sensor_pos.y);

        if let Some(sensor_idx) = self.world_to_grid(sensor_xy.x, sensor_xy.y) {
            // Iterate through each point in the laser frame
            for pt in &frame.points {
                // Transform the local point to global coordinates
                let global_pt = local_to_global(pt, &frame.sensor_pos, yaw);
                if let Some(end_idx) = self.world_to_grid(global_pt.x, global_pt.y) {
                    // Bresenham's line algorithm to get the points between the sensor and the point
                    for (x, y) in Bresenham::new(
                            (sensor_idx.0 as isize, sensor_idx.1 as isize),
                            (end_idx.0 as isize, end_idx.1 as isize)
                    ) {
                        let idx = (y * self.width as isize + x) as usize;
                        let delta = if (x, y) == (end_idx.0 as isize, end_idx.1 as isize) {
                            L_OCC
                        } else {
                            L_FREE
                        };
                        let lo = (self.log_odds[idx] + delta).clamp(L_MIN, L_MAX);
                        self.log_odds[idx] = lo;
                    }
                }
            }
        }
    }

    /// Get the occupancy grid as a vector of 0 and 100
    /// 0: free, 100: occupied
    #[allow(unused)]
    fn as_occupancy(&self, thresh: f32) -> Vec<u8> {
        self.log_odds
            .iter()
            .map(|&lo| if lo > thresh { 100 } else { 0 })
            .collect()
    }
}

fn local_to_global(
    pt: &Point2f,
    sensor_pos: &Point3f,
    yaw: f32, // degrees
) -> Point2f {
    let cos_y = yaw.to_radians().cos();
    let sin_y = yaw.to_radians().sin();

    let x = pt.x * cos_y - pt.y * sin_y + sensor_pos.x;
    let y = pt.x * sin_y + pt.y * cos_y + sensor_pos.y;

    Point2f::new(x, y)
}