#pragma once

#include "math.hpp"
#include "grid.hpp"
#include "clustering.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>

//----------------------------------------------
// Navigation Structures
//----------------------------------------------
struct NavigationWaypoint {
    Vec3 position;
    Vec3 velocity;
    float yaw;
    std::chrono::steady_clock::time_point timestamp;
    
    NavigationWaypoint() : yaw(0.0f) {}
    NavigationWaypoint(const Vec3& pos, const Vec3& vel, float y) 
        : position(pos), velocity(vel), yaw(y) {}
};

struct NavigationCommand {
    enum Type {
        POSITION_SETPOINT,
        VELOCITY_SETPOINT,
        LAND,
        HOLD,
        EMERGENCY_STOP
    };
    
    Type type;
    Vec3 position;
    Vec3 velocity;
    float yaw;
    float yaw_rate;
    std::chrono::steady_clock::time_point timestamp;
    
    NavigationCommand() : type(POSITION_SETPOINT), yaw(0.0f), yaw_rate(0.0f) {}
};

//----------------------------------------------
// Costmap Generation
//----------------------------------------------
class NavigationCostmap {
private:
    int width, height;
    float resolution;
    Vec3 origin;
    std::vector<float> costs;
    std::vector<bool> occupied;

    friend class VoxelCostmapGenerator;

public:
    NavigationCostmap(int w, int h, float res, const Vec3& orig)
        : width(w), height(h), resolution(res), origin(orig) {
        costs.resize(width * height, 0.0f);
        occupied.resize(width * height, false);
    }
    
    // Get cost at grid position
    float get_cost(int x, int y) const {
        if(x < 0 || x >= width || y < 0 || y >= height) return 1.0f; // High cost for out of bounds
        return costs[y * width + x];
    }
    
    // Set cost at grid position
    void set_cost(int x, int y, float cost) {
        if(x < 0 || x >= width || y < 0 || y >= height) return;
        costs[y * width + x] = std::max(0.0f, std::min(1.0f, cost));
    }
    
    // Check if position is occupied
    bool is_occupied(int x, int y) const {
        if(x < 0 || x >= width || y < 0 || y >= height) return true;
        return occupied[y * width + x];
    }
    
    // Set occupied status
    void set_occupied(int x, int y, bool occ) {
        if(x < 0 || x >= width || y < 0 || y >= height) return;
        occupied[y * width + x] = occ;
    }
    
    // World to grid coordinates
    void world_to_grid(const Vec3& world_pos, int& grid_x, int& grid_y) const {
        grid_x = static_cast<int>((world_pos.x - origin.x) / resolution);
        grid_y = static_cast<int>((world_pos.y - origin.y) / resolution);
    }
    
    // Grid to world coordinates
    Vec3 grid_to_world(int grid_x, int grid_y) const {
        return origin + Vec3(grid_x * resolution, grid_y * resolution, 0.0f);
    }
    
    // Get dimensions
    int get_width() const { return width; }
    int get_height() const { return height; }
    float get_resolution() const { return resolution; }
    const Vec3& get_origin() const { return origin; }
    
    // Clear all costs
    void clear() {
        std::fill(costs.begin(), costs.end(), 0.0f);
        std::fill(occupied.begin(), occupied.end(), false);
    }
};

//----------------------------------------------
// Costmap from Voxel Grid
//----------------------------------------------
class VoxelCostmapGenerator {
private:
    float static_weight;
    float dynamic_weight;
    float cluster_weight;
    float safety_margin;
    
public:
    VoxelCostmapGenerator(float static_w = 0.3f, float dynamic_w = 0.7f, 
                          float cluster_w = 1.0f, float safety = 2.0f)
        : static_weight(static_w), dynamic_weight(dynamic_w), 
          cluster_weight(cluster_w), safety_margin(safety) {}
    
    // Generate 2D costmap from 3D voxel grid
    NavigationCostmap generate_costmap(const DualVoxelGrid& voxel_grid, 
                                      const std::vector<VoxelCluster>& clusters,
                                      const Vec3& drone_position,
                                      float altitude_band_width = 5.0f) {
        const GridConfig& grid_config = voxel_grid.get_config();
        
        // Create costmap centered on drone
        int costmap_size = static_cast<int>(grid_config.horizon_distance * 2 / 0.5f); // 0.5m resolution
        float costmap_resolution = 0.5f;
        Vec3 costmap_origin = drone_position - Vec3(grid_config.horizon_distance, grid_config.horizon_distance, 0);
        
        NavigationCostmap costmap(costmap_size, costmap_size, costmap_resolution, costmap_origin);
        
        // Project voxel grid to 2D costmap
        project_voxels_to_costmap(voxel_grid, costmap, drone_position, altitude_band_width);
        
        // Add cluster costs
        add_cluster_costs(clusters, costmap, drone_position);
        
        // Add safety margins
        add_safety_margins(costmap);
        
        return costmap;
    }
    
private:
    // Project 3D voxels to 2D costmap
    void project_voxels_to_costmap(const DualVoxelGrid& voxel_grid, 
                                  NavigationCostmap& costmap,
                                  const Vec3& drone_position,
                                  float altitude_band_width) {
        const GridConfig& grid_config = voxel_grid.get_config();
        Vec3 grid_min, grid_max;
        voxel_grid.get_bounds(grid_min, grid_max);
        
        // Define altitude band around drone
        float z_min = drone_position.z - altitude_band_width * 0.5f;
        float z_max = drone_position.z + altitude_band_width * 0.5f;
        
        // Iterate through voxel grid
        for(int ix = 0; ix < grid_config.N; ix++) {
            for(int iy = 0; iy < grid_config.N; iy++) {
                for(int iz = 0; iz < grid_config.N; iz++) {
                    // Check if voxel is in altitude band
                    float voxel_z = grid_min.z + iz * grid_config.voxel_size;
                    if(voxel_z < z_min || voxel_z > z_max) continue;
                    
                    // Get voxel values
                    int voxel_idx = voxel_grid.voxel_to_index(ix, iy, iz);
                    float static_val = voxel_grid.get_static_voxel(voxel_idx);
                    float dynamic_val = voxel_grid.get_dynamic_voxel(voxel_idx);
                    
                    // Convert voxel position to world coordinates
                    Vec3 voxel_world = grid_min + Vec3(ix * grid_config.voxel_size, 
                                                      iy * grid_config.voxel_size, 
                                                      iz * grid_config.voxel_size);
                    
                    // Project to costmap
                    int costmap_x, costmap_y;
                    costmap.world_to_grid(voxel_world, costmap_x, costmap_y);
                    
                    if(costmap_x >= 0 && costmap_x < costmap.get_width() && 
                       costmap_y >= 0 && costmap_y < costmap.get_height()) {
                        
                        // Compute combined cost
                        float cost = static_weight * static_val + dynamic_weight * dynamic_val;
                        float current_cost = costmap.get_cost(costmap_x, costmap_y);
                        costmap.set_cost(costmap_x, costmap_y, std::max(current_cost, cost));
                        
                        // Mark as occupied if cost is high
                        if(cost > 0.5f) {
                            costmap.set_occupied(costmap_x, costmap_y, true);
                        }
                    }
                }
            }
        }
    }
    
    // Add costs from detected clusters
    void add_cluster_costs(const std::vector<VoxelCluster>& clusters,
                          NavigationCostmap& costmap,
                          const Vec3& drone_position) {
        for(const auto& cluster : clusters) {
            // Convert cluster centroid to costmap coordinates
            int costmap_x, costmap_y;
            costmap.world_to_grid(cluster.centroid, costmap_x, costmap_y);
            
            if(costmap_x >= 0 && costmap_x < costmap.get_width() && 
               costmap_y >= 0 && costmap_y < costmap.get_height()) {
                
                // Add cluster cost based on intensity and size
                float cluster_cost = cluster_weight * cluster.total_intensity /
                                   std::max(1.0f, static_cast<float>(cluster.voxel_count));
                
                float current_cost = costmap.get_cost(costmap_x, costmap_y);
                costmap.set_cost(costmap_x, costmap_y, std::max(current_cost, cluster_cost));
                
                // Mark cluster area as occupied
                mark_cluster_area_occupied(cluster, costmap);
            }
        }
    }
    
    // Mark cluster area as occupied in costmap
    void mark_cluster_area_occupied(const VoxelCluster& cluster, NavigationCostmap& costmap) {
        Vec3 cluster_size = cluster.get_size();
        float max_dim = std::max({cluster_size.x, cluster_size.y, cluster_size.z});
        
        // Mark area around cluster centroid
        int radius = static_cast<int>(max_dim / costmap.get_resolution() + safety_margin);
        
        int center_x, center_y;
        costmap.world_to_grid(cluster.centroid, center_x, center_y);
        
        for(int dx = -radius; dx <= radius; dx++) {
            for(int dy = -radius; dy <= radius; dy++) {
                int x = center_x + dx;
                int y = center_y + dy;
                
                if(x >= 0 && x < costmap.get_width() && y >= 0 && y < costmap.get_height()) {
                    float distance = std::sqrt(dx*dx + dy*dy);
                    if(distance <= radius) {
                        costmap.set_occupied(x, y, true);
                    }
                }
            }
        }
    }
    
    // Add safety margins around obstacles
    void add_safety_margins(NavigationCostmap& costmap) {
        std::vector<bool> new_occupied = costmap.occupied;
        
        for(int y = 0; y < costmap.get_height(); y++) {
            for(int x = 0; x < costmap.get_width(); x++) {
                if(costmap.is_occupied(x, y)) {
                    // Add safety margin
                    int margin = static_cast<int>(safety_margin / costmap.get_resolution());
                    
                    for(int dy = -margin; dy <= margin; dy++) {
                        for(int dx = -margin; dx <= margin; dx++) {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            if(nx >= 0 && nx < costmap.get_width() && 
                               ny >= 0 && ny < costmap.get_height()) {
                                
                                float distance = std::sqrt(dx*dx + dy*dy);
                                if(distance <= margin) {
                                    new_occupied[ny * costmap.get_width() + nx] = true;
                                    
                                    // Increase cost in safety margin
                                    float margin_cost = 0.5f * (1.0f - distance / margin);
                                    float current_cost = costmap.get_cost(nx, ny);
                                    costmap.set_cost(nx, ny, std::max(current_cost, margin_cost));
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Update occupied status
        costmap.occupied = new_occupied;
    }
};

//----------------------------------------------
// Path Planning
//----------------------------------------------
class NavigationPlanner {
private:
    float max_velocity;
    float max_acceleration;
    float planning_horizon;
    float replan_rate;
    
public:
    NavigationPlanner(float max_vel = 5.0f, float max_acc = 2.0f, 
                      float horizon = 10.0f, float replan = 5.0f)
        : max_velocity(max_vel), max_acceleration(max_acc), 
          planning_horizon(horizon), replan_rate(replan) {}
    
    // Plan path to goal using A* over the costmap. Falls back to a straight-line
    // march if A* finds no route (e.g. goal enclosed). Returns waypoints with
    // velocity profiles. The deterministic failsafe (Part 7) and the CfC
    // imitation dataset (Part 4) both consume this.
    std::vector<NavigationWaypoint> plan_path(const NavigationCostmap& costmap,
                                             const Vec3& start_pos,
                                             const Vec3& goal_pos,
                                             const Vec3& current_velocity = Vec3(0,0,0)) {
        std::vector<Vec3> path = astar_pathfinding(costmap, start_pos, goal_pos);
        if(path.empty()) {
            path = greedy_straight_line_planner(costmap, start_pos, goal_pos);
        }
        if(path.empty()) {
            return std::vector<NavigationWaypoint>();
        }
        return generate_waypoints(path, current_velocity);
    }
    
    // Generate velocity setpoints for current position
    NavigationCommand generate_velocity_command(const Vec3& current_pos,
                                              const Vec3& goal_pos,
                                              const Vec3& current_velocity,
                                              const NavigationCostmap& costmap) {
        NavigationCommand cmd;
        cmd.type = NavigationCommand::VELOCITY_SETPOINT;
        cmd.timestamp = std::chrono::steady_clock::now();
        
        // Simple proportional control
        Vec3 position_error = goal_pos - current_pos;
        float distance = length(position_error);
        
        if(distance < 0.5f) {
            // Close to goal, slow down
            cmd.velocity = Vec3(0, 0, 0);
        } else {
            // Move toward goal
            Vec3 desired_velocity = normalize(position_error) * max_velocity;
            
            // Limit velocity based on distance and obstacles
            desired_velocity = limit_velocity_for_obstacles(desired_velocity, current_pos, costmap);
            
            cmd.velocity = desired_velocity;
        }
        
        // Set yaw to face goal direction
        if(distance > 0.1f) {
            cmd.yaw = std::atan2(position_error.y, position_error.x);
        }
        
        return cmd;
    }
    
private:
    // A* over the 2D occupancy costmap. 8-connected, octile heuristic,
    // step cost = resolution * (1 + cell cost); occupied cells are
    // impassable. Returns an empty path if no route exists.
    std::vector<Vec3> astar_pathfinding(const NavigationCostmap& costmap,
                                       const Vec3& start_pos,
                                       const Vec3& goal_pos) {
        int W = costmap.get_width();
        int H = costmap.get_height();
        if(W <= 0 || H <= 0) return {};

        int sx, sy, gx, gy;
        costmap.world_to_grid(start_pos, sx, sy);
        costmap.world_to_grid(goal_pos, gx, gy);

        // Clamp start/goal into the costmap (they can be marginally out of
        // bounds because world_to_grid does no clamping).
        sx = std::max(0, std::min(sx, W - 1));
        sy = std::max(0, std::min(sy, H - 1));
        gx = std::max(0, std::min(gx, W - 1));
        gy = std::max(0, std::min(gy, H - 1));

        if(costmap.is_occupied(sx, sy) || costmap.is_occupied(gx, gy)) {
            return {};  // start or goal inside an obstacle -> no A* route
        }

        const int N = W * H;
        struct Node {
            float g = std::numeric_limits<float>::infinity();
            float f = std::numeric_limits<float>::infinity();
            int came_from = -1;
        };
        std::vector<Node> nodes(N);

        auto idx = [&](int x, int y){ return y * W + x; };

        // Priority queue ordered by f = g + h
        auto cmp = [&](int a, int b){ return nodes[a].f > nodes[b].f; };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

        const int start_idx = idx(sx, sy);
        nodes[start_idx].g = 0.0f;
        nodes[start_idx].f = octile_heuristic(sx, sy, gx, gy, costmap.get_resolution());
        open.push(start_idx);

        const int dx8[8] = {1, -1, 0,  0, 1, 1, -1, -1};
        const int dy8[8] = {0,  0, 1, -1, 1, -1, 1, -1};

        std::vector<bool> closed(N, false);
        bool found = false;

        while(!open.empty()) {
            int cur = open.top(); open.pop();
            if(closed[cur]) continue;
            closed[cur] = true;

            int cx = cur % W;
            int cy = cur / W;
            if(cx == gx && cy == gy) { found = true; break; }

            for(int k = 0; k < 8; ++k) {
                int nx = cx + dx8[k];
                int ny = cy + dy8[k];
                if(nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                if(costmap.is_occupied(nx, ny)) continue;

                // Prevent corner-cutting through diagonally adjacent obstacles
                if(k >= 4) {
                    if(costmap.is_occupied(cx + dx8[k], cy) ||
                       costmap.is_occupied(cx, cy + dy8[k])) continue;
                }

                int nidx = idx(nx, ny);
                if(closed[nidx]) continue;

                float step = costmap.get_resolution();
                if(dx8[k] != 0 && dy8[k] != 0) step *= 1.41421356f;  // sqrt(2)
                float cost = step * (1.0f + costmap.get_cost(nx, ny));
                float tentative_g = nodes[cur].g + cost;

                if(tentative_g < nodes[nidx].g) {
                    nodes[nidx].g = tentative_g;
                    nodes[nidx].f = tentative_g + octile_heuristic(nx, ny, gx, gy, costmap.get_resolution());
                    nodes[nidx].came_from = cur;
                    open.push(nidx);
                }
            }
        }

        if(!found) return {};

        // Reconstruct from goal -> start, then reverse
        std::vector<Vec3> path;
        int cur = idx(gx, gy);
        while(cur != -1) {
            int x = cur % W;
            int y = cur / W;
            path.push_back(costmap.grid_to_world(x, y));
            cur = nodes[cur].came_from;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Octile heuristic for an 8-connected uniform-cost 2D grid
    static float octile_heuristic(int ax, int ay, int bx, int by, float res) {
        int dx = std::abs(ax - bx);
        int dy = std::abs(ay - by);
        float D = res;
        float D2 = res * 1.41421356f;
        return D * (dx + dy) + (D2 - 2.0f * D) * static_cast<float>(std::min(dx, dy));
    }

    // Deterministic greedy straight-line march. Used as a fallback when A*
    // returns no path, and as the imitation target for the CfC pilot (Part 4).
    std::vector<Vec3> greedy_straight_line_planner(const NavigationCostmap& costmap,
                                                   const Vec3& start_pos,
                                                   const Vec3& goal_pos) {
        std::vector<Vec3> path;
        path.push_back(start_pos);

        Vec3 direction = normalize(goal_pos - start_pos);
        float distance = length(goal_pos - start_pos);
        float step_size = costmap.get_resolution();  // costmap-resolution steps

        for(float d = step_size; d < distance; d += step_size) {
            Vec3 waypoint = start_pos + direction * d;
            int grid_x, grid_y;
            costmap.world_to_grid(waypoint, grid_x, grid_y);
            if(!costmap.is_occupied(grid_x, grid_y)) {
                path.push_back(waypoint);
            } else {
                break;  // obstacle blocks the straight route; path stops short
            }
        }

        path.push_back(goal_pos);
        return path;
    }
    
    // Generate waypoints from path
    std::vector<NavigationWaypoint> generate_waypoints(const std::vector<Vec3>& path,
                                                      const Vec3& current_velocity) {
        std::vector<NavigationWaypoint> waypoints;
        
        if(path.size() < 2) return waypoints;
        
        Vec3 current_vel = current_velocity;
        
        for(size_t i = 1; i < path.size(); i++) {
            Vec3 target_pos = path[i];
            Vec3 prev_pos = path[i-1];
            
            // Compute desired velocity to reach target
            Vec3 direction = normalize(target_pos - prev_pos);
            Vec3 target_velocity = direction * max_velocity;
            
            // Create waypoint
            NavigationWaypoint waypoint;
            waypoint.position = target_pos;
            waypoint.velocity = target_velocity;
            waypoint.yaw = std::atan2(direction.y, direction.x);
            waypoint.timestamp = std::chrono::steady_clock::now() + 
                               std::chrono::milliseconds(static_cast<int>(i * 200)); // 200ms intervals
            
            waypoints.push_back(waypoint);
        }
        
        return waypoints;
    }
    
    // Limit velocity based on obstacles
    Vec3 limit_velocity_for_obstacles(const Vec3& desired_velocity,
                                     const Vec3& current_pos,
                                     const NavigationCostmap& costmap) {
        Vec3 limited_velocity = desired_velocity;
        
        // Check forward direction for obstacles
        Vec3 forward_pos = current_pos + desired_velocity * 0.1f; // Look 0.1s ahead
        
        int grid_x, grid_y;
        costmap.world_to_grid(forward_pos, grid_x, grid_y);
        
        if(costmap.is_occupied(grid_x, grid_y)) {
            // Obstacle ahead, reduce velocity
            limited_velocity = limited_velocity * 0.5f;
        }
        
        // Ensure velocity doesn't exceed limits
        float vel_magnitude = length(limited_velocity);
        if(vel_magnitude > max_velocity) {
            limited_velocity = normalize(limited_velocity) * max_velocity;
        }
        
        return limited_velocity;
    }
};

//----------------------------------------------
// Flight Controller Interface
//----------------------------------------------
class FlightControllerInterface {
private:
    std::function<void(const NavigationCommand&)> command_callback;
    std::chrono::steady_clock::time_point last_command_time;
    float command_rate;
    
public:
    FlightControllerInterface(float rate = 20.0f) : command_rate(rate) {
        last_command_time = std::chrono::steady_clock::now();
    }
    
    // Set command callback
    void set_command_callback(std::function<void(const NavigationCommand&)> callback) {
        command_callback = callback;
    }
    
    // Send navigation command
    void send_command(const NavigationCommand& command) {
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_command_time);
        
        // Rate limit commands
        if(time_since_last.count() >= (1000.0f / command_rate)) {
            if(command_callback) {
                command_callback(command);
            }
            last_command_time = current_time;
        }
    }
    
    // Send emergency stop
    void send_emergency_stop() {
        NavigationCommand stop_cmd;
        stop_cmd.type = NavigationCommand::EMERGENCY_STOP;
        stop_cmd.timestamp = std::chrono::steady_clock::now();
        
        if(command_callback) {
            command_callback(stop_cmd);
        }
    }
    
    // Set command rate
    void set_command_rate(float rate) {
        command_rate = rate;
    }
};
