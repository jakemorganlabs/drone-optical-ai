#pragma once

#include "math.hpp"
#include "grid.hpp"
#include <vector>
#include <map>
#include <unordered_set>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <limits>

//----------------------------------------------
// Cluster Structures
//----------------------------------------------
struct VoxelCluster {
    int cluster_id;
    Vec3 centroid;
    Vec3 bbox_min;
    Vec3 bbox_max;
    float total_intensity;
    int voxel_count;
    std::chrono::steady_clock::time_point first_seen;
    std::chrono::steady_clock::time_point last_seen;
    std::vector<int> voxel_indices;
    
    VoxelCluster() : cluster_id(-1), total_intensity(0.0f), voxel_count(0) {}
    
    // Compute cluster properties
    void compute_properties(const DualVoxelGrid& grid) {
        if(voxel_indices.empty()) return;
        
        const GridConfig& config = grid.get_config();
        Vec3 grid_min, grid_max;
        grid.get_bounds(grid_min, grid_max);
        
        // Compute centroid and bounding box
        Vec3 sum_pos(0, 0, 0);
        bbox_min = Vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        bbox_max = Vec3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
        
        for(int voxel_idx : voxel_indices) {
            // Convert linear index to 3D indices
            int iz = voxel_idx % config.N;
            int iy = (voxel_idx / config.N) % config.N;
            int ix = voxel_idx / (config.N * config.N);
            
            // Convert to world coordinates
            Vec3 world_pos = grid_min + Vec3(ix * config.voxel_size, iy * config.voxel_size, iz * config.voxel_size);
            
            sum_pos = sum_pos + world_pos;
            
            bbox_min.x = std::min(bbox_min.x, world_pos.x);
            bbox_min.y = std::min(bbox_min.y, world_pos.y);
            bbox_min.z = std::min(bbox_min.z, world_pos.z);
            
            bbox_max.x = std::max(bbox_max.x, world_pos.x);
            bbox_max.y = std::max(bbox_max.y, world_pos.y);
            bbox_max.z = std::max(bbox_max.z, world_pos.z);
        }
        
        centroid = sum_pos / static_cast<float>(voxel_indices.size());
        
        // Compute total intensity
        total_intensity = 0.0f;
        for(int voxel_idx : voxel_indices) {
            total_intensity += grid.get_dynamic_voxel(voxel_idx);
        }
        
        voxel_count = static_cast<int>(voxel_indices.size());
    }
    
    // Get cluster size in meters
    Vec3 get_size() const {
        return bbox_max - bbox_min;
    }
    
    // Get cluster volume
    float get_volume() const {
        Vec3 size = get_size();
        return size.x * size.y * size.z;
    }
    
    // Check if cluster is active (recently updated)
    bool is_active(std::chrono::steady_clock::time_point current_time, 
                   std::chrono::milliseconds max_age = std::chrono::milliseconds(5000)) const {
        return (current_time - last_seen) < max_age;
    }
};

//----------------------------------------------
// Track Structure
//----------------------------------------------
struct VoxelTrack {
    int track_id;
    std::vector<Vec3> positions;
    std::vector<std::chrono::steady_clock::time_point> timestamps;
    std::vector<float> intensities;
    Vec3 velocity;
    float velocity_magnitude;
    std::chrono::steady_clock::time_point first_seen;
    std::chrono::steady_clock::time_point last_seen;
    int consecutive_matches;
    int total_matches;
    
    VoxelTrack() : track_id(-1), velocity_magnitude(0.0f), 
                   consecutive_matches(0), total_matches(0) {}
    
    // Add new position to track
    void add_position(const Vec3& pos, float intensity, std::chrono::steady_clock::time_point timestamp) {
        positions.push_back(pos);
        timestamps.push_back(timestamp);
        intensities.push_back(intensity);
        
        if(timestamps.size() == 1) {
            first_seen = timestamp;
        }
        last_seen = timestamp;
        
        // Update velocity if we have at least 2 positions
        if(positions.size() >= 2) {
            Vec3 pos_diff = positions.back() - positions[positions.size() - 2];
            auto time_diff = timestamps.back() - timestamps[timestamps.size() - 2];
            float dt = std::chrono::duration<float>(time_diff).count();
            
            if(dt > 0.0f) {
                velocity = pos_diff / dt;
                velocity_magnitude = length(velocity);
            }
        }
    }
    
    // Get track duration
    std::chrono::milliseconds get_duration() const {
        if(timestamps.empty()) return std::chrono::milliseconds(0);
        return std::chrono::duration_cast<std::chrono::milliseconds>(last_seen - first_seen);
    }
    
    // Get average intensity
    float get_average_intensity() const {
        if(intensities.empty()) return 0.0f;
        float sum = 0.0f;
        for(float intensity : intensities) {
            sum += intensity;
        }
        return sum / intensities.size();
    }
    
    // Check if track is active
    bool is_active(std::chrono::steady_clock::time_point current_time,
                   std::chrono::milliseconds max_age = std::chrono::milliseconds(10000)) const {
        return (current_time - last_seen) < max_age;
    }
    
    // Predict position at given time
    Vec3 predict_position(std::chrono::steady_clock::time_point target_time) const {
        if(positions.size() < 2) return positions.empty() ? Vec3(0,0,0) : positions.back();
        
        auto time_diff = target_time - last_seen;
        float dt = std::chrono::duration<float>(time_diff).count();
        
        return positions.back() + velocity * dt;
    }
};

//----------------------------------------------
// Clustering Algorithm (DBSCAN-like)
//----------------------------------------------
class VoxelClusterer {
private:
    float eps;  // Distance threshold for clustering
    int min_points;  // Minimum points to form a cluster
    std::vector<bool> visited;
    std::vector<bool> noise;
    
public:
    VoxelClusterer(float epsilon = 2.0f, int min_pts = 4) 
        : eps(epsilon), min_points(min_pts) {}
    
    // Find clusters in dynamic voxel grid
    std::vector<VoxelCluster> find_clusters(const DualVoxelGrid& grid, float intensity_threshold = 0.1f) {
        const GridConfig& config = grid.get_config();
        int total_voxels = grid.get_total_voxels();
        
        // Reset state
        visited.clear();
        noise.clear();
        visited.resize(total_voxels, false);
        noise.resize(total_voxels, false);
        
        std::vector<VoxelCluster> clusters;
        int cluster_id = 0;
        
        // Find all voxels above threshold
        std::vector<int> active_voxels;
        for(int i = 0; i < total_voxels; i++) {
            if(grid.get_dynamic_voxel(i) > intensity_threshold) {
                active_voxels.push_back(i);
            }
        }
        
        // Run DBSCAN-like clustering
        for(int voxel_idx : active_voxels) {
            if(visited[voxel_idx]) continue;
            
            visited[voxel_idx] = true;
            
            // Find neighbors
            std::vector<int> neighbors = find_neighbors(voxel_idx, grid, intensity_threshold);
            
            if(neighbors.size() < min_points) {
                noise[voxel_idx] = true;
            } else {
                // Start new cluster
                VoxelCluster cluster;
                cluster.cluster_id = cluster_id++;
                cluster.first_seen = std::chrono::steady_clock::now();
                cluster.last_seen = cluster.first_seen;
                
                // Expand cluster
                expand_cluster(voxel_idx, neighbors, cluster, grid, intensity_threshold);
                
                // Compute cluster properties
                cluster.compute_properties(grid);
                
                if(cluster.voxel_count >= min_points) {
                    clusters.push_back(cluster);
                }
            }
        }
        
        return clusters;
    }
    
private:
    // Find neighboring voxels
    std::vector<int> find_neighbors(int voxel_idx, const DualVoxelGrid& grid, float intensity_threshold) {
        const GridConfig& config = grid.get_config();
        std::vector<int> neighbors;
        
        // Convert linear index to 3D indices
        int iz = voxel_idx % config.N;
        int iy = (voxel_idx / config.N) % config.N;
        int ix = voxel_idx / (config.N * config.N);
        
        // Check 26-connected neighbors
        for(int dz = -1; dz <= 1; dz++) {
            for(int dy = -1; dy <= 1; dy++) {
                for(int dx = -1; dx <= 1; dx++) {
                    if(dx == 0 && dy == 0 && dz == 0) continue;
                    
                    int nx = ix + dx;
                    int ny = iy + dy;
                    int nz = iz + dz;
                    
                    if(nx >= 0 && nx < config.N && ny >= 0 && ny < config.N && nz >= 0 && nz < config.N) {
                        int neighbor_idx = grid.voxel_to_index(nx, ny, nz);
                        if(grid.get_dynamic_voxel(neighbor_idx) > intensity_threshold) {
                            neighbors.push_back(neighbor_idx);
                        }
                    }
                }
            }
        }
        
        return neighbors;
    }
    
    // Expand cluster recursively
    void expand_cluster(int voxel_idx, std::vector<int>& neighbors, VoxelCluster& cluster,
                       const DualVoxelGrid& grid, float intensity_threshold) {
        cluster.voxel_indices.push_back(voxel_idx);
        
        for(int neighbor_idx : neighbors) {
            if(visited[neighbor_idx]) continue;
            
            visited[neighbor_idx] = true;
            
            // Find neighbors of this neighbor
            std::vector<int> neighbor_neighbors = find_neighbors(neighbor_idx, grid, intensity_threshold);
            
            if(neighbor_neighbors.size() >= min_points) {
                // Add new neighbors to the list
                for(int new_neighbor : neighbor_neighbors) {
                    if(std::find(neighbors.begin(), neighbors.end(), new_neighbor) == neighbors.end()) {
                        neighbors.push_back(new_neighbor);
                    }
                }
            }
            
            // Add to cluster
            cluster.voxel_indices.push_back(neighbor_idx);
        }
    }
};

//----------------------------------------------
// Track Association
//----------------------------------------------
class VoxelTracker {
private:
    std::vector<VoxelTrack> tracks;
    int next_track_id;
    float max_association_distance;
    std::chrono::milliseconds max_track_age;
    
public:
    VoxelTracker(float max_dist = 5.0f, std::chrono::milliseconds max_age = std::chrono::milliseconds(10000))
        : next_track_id(0), max_association_distance(max_dist), max_track_age(max_age) {}
    
    // Update tracks with new clusters
    void update_tracks(const std::vector<VoxelCluster>& clusters) {
        auto current_time = std::chrono::steady_clock::now();
        
        // Remove old tracks
        tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
            [&](const VoxelTrack& track) {
                return !track.is_active(current_time, max_track_age);
            }), tracks.end());
        
        // Associate clusters with existing tracks
        std::vector<bool> cluster_matched(clusters.size(), false);
        
        for(VoxelTrack& track : tracks) {
            float best_distance = std::numeric_limits<float>::max();
            int best_cluster_idx = -1;
            
            // Find best matching cluster
            for(size_t i = 0; i < clusters.size(); i++) {
                if(cluster_matched[i]) continue;
                
                float distance = length(track.positions.back() - clusters[i].centroid);
                if(distance < best_distance && distance < max_association_distance) {
                    best_distance = distance;
                    best_cluster_idx = i;
                }
            }
            
            if(best_cluster_idx >= 0) {
                // Update track
                const VoxelCluster& cluster = clusters[best_cluster_idx];
                track.add_position(cluster.centroid, cluster.total_intensity, current_time);
                track.consecutive_matches++;
                track.total_matches++;
                cluster_matched[best_cluster_idx] = true;
            } else {
                // No match found
                track.consecutive_matches = 0;
            }
        }
        
        // Create new tracks for unmatched clusters
        for(size_t i = 0; i < clusters.size(); i++) {
            if(!cluster_matched[i]) {
                VoxelTrack new_track;
                new_track.track_id = next_track_id++;
                new_track.add_position(clusters[i].centroid, clusters[i].total_intensity, current_time);
                new_track.consecutive_matches = 1;
                new_track.total_matches = 1;
                tracks.push_back(new_track);
            }
        }
    }
    
    // Get all tracks
    const std::vector<VoxelTrack>& get_tracks() const { return tracks; }
    
    // Get active tracks
    std::vector<VoxelTrack> get_active_tracks() const {
        auto current_time = std::chrono::steady_clock::now();
        std::vector<VoxelTrack> active_tracks;
        
        for(const VoxelTrack& track : tracks) {
            if(track.is_active(current_time, max_track_age)) {
                active_tracks.push_back(track);
            }
        }
        
        return active_tracks;
    }
    
    // Get track by ID
    const VoxelTrack* get_track(int track_id) const {
        for(const VoxelTrack& track : tracks) {
            if(track.track_id == track_id) {
                return &track;
            }
        }
        return nullptr;
    }
};

//----------------------------------------------
// Telemetry Generation
//----------------------------------------------
struct TelemetryReport {
    std::chrono::steady_clock::time_point timestamp;
    Vec3 drone_position;
    int total_clusters;
    int active_tracks;
    std::vector<VoxelCluster> clusters;
    std::vector<VoxelTrack> tracks;
    
    TelemetryReport() : total_clusters(0), active_tracks(0) {}
    
    // Generate compact telemetry message
    std::string to_compact_string() const {
        std::stringstream ss;
        ss << "T:" << std::chrono::duration_cast<std::chrono::milliseconds>(
            timestamp.time_since_epoch()).count();
        ss << " P:" << drone_position.x << "," << drone_position.y << "," << drone_position.z;
        ss << " C:" << total_clusters << " T:" << active_tracks;
        
        // Add cluster summaries
        for(const auto& cluster : clusters) {
            ss << " c" << cluster.cluster_id << ":" << cluster.centroid.x << "," 
               << cluster.centroid.y << "," << cluster.centroid.z << "," << cluster.total_intensity;
        }
        
        // Add track summaries
        for(const auto& track : tracks) {
            ss << " t" << track.track_id << ":" << track.positions.back().x << "," 
               << track.positions.back().y << "," << track.positions.back().z << "," 
               << track.velocity_magnitude;
        }
        
        return ss.str();
    }
    
    // Get report size in bytes
    size_t get_size_bytes() const {
        return to_compact_string().length();
    }
};
