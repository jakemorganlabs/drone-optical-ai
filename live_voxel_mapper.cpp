#include "math.hpp"
#include "grid.hpp"
#include "pose.hpp"
#include "motion.hpp"
#include "ray_marching.hpp"
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <algorithm>

//----------------------------------------------
// Helpers: convert a CameraFrame to an ImageGray view
//----------------------------------------------
static inline ImageGray camera_frame_to_image_gray(const CameraFrame& frame) {
    ImageGray img(frame.width, frame.height);
    img.pixels = frame.pixels;
    return img;
}

//----------------------------------------------
// Live Voxel Mapping System
//----------------------------------------------
class LiveVoxelMapper {
private:
    // Core components
    std::unique_ptr<DualVoxelGrid> voxel_grid;
    std::unique_ptr<PoseProvider> pose_provider;
    
    // Configuration
    GridConfig grid_config;
    float motion_threshold;
    float adaptive_threshold_percentile;
    bool use_adaptive_threshold;
    bool use_noise_filtering;
    int min_region_size;
    
    // State
    std::atomic<bool> running;
    std::thread processing_thread;
    std::chrono::steady_clock::time_point last_frame_time;
    
    // Frame buffers
    ImageGray prev_frame;
    bool prev_frame_valid;
    
    // Statistics
    struct ProcessingStats {
        int total_frames;
        int motion_frames;
        float avg_motion_percentage;
        float avg_processing_time_ms;
        std::chrono::steady_clock::time_point last_update;
        
        ProcessingStats() : total_frames(0), motion_frames(0), 
                           avg_motion_percentage(0.0f), avg_processing_time_ms(0.0f) {}
    } stats;
    
public:
    LiveVoxelMapper(const GridConfig& config = GridConfig()) 
        : grid_config(config), motion_threshold(2.0f), adaptive_threshold_percentile(95.0f),
          use_adaptive_threshold(true), use_noise_filtering(true), min_region_size(4),
          running(false), prev_frame_valid(false) {
        
        // Initialize voxel grid
        voxel_grid = std::make_unique<DualVoxelGrid>(grid_config);
        
        // Initialize pose provider (default to mock for development)
        pose_provider = std::make_unique<MockPoseProvider>();
        
        // Set up callbacks
        pose_provider->register_frame_callback([this](const CameraFrame& frame) {
            this->process_frame(frame);
        });
        
        pose_provider->register_pose_callback([this](const Pose& pose) {
            this->update_grid_origin(pose);
        });
    }
    
    ~LiveVoxelMapper() {
        stop();
    }
    
    // Configuration
    void set_motion_threshold(float threshold) { motion_threshold = threshold; }
    void set_adaptive_threshold_percentile(float percentile) { adaptive_threshold_percentile = percentile; }
    void set_use_adaptive_threshold(bool use) { use_adaptive_threshold = use; }
    void set_use_noise_filtering(bool use) { use_noise_filtering = use; }
    void set_min_region_size(int size) { min_region_size = size; }
    
    // Start/stop the system
    bool start() {
        if(running.load()) return true;
        
        if(!pose_provider->start()) {
            std::cerr << "Failed to start pose provider" << std::endl;
            return false;
        }
        
        running.store(true);
        processing_thread = std::thread(&LiveVoxelMapper::processing_loop, this);
        
        std::cout << "Live voxel mapper started" << std::endl;
        return true;
    }
    
    void stop() {
        if(!running.load()) return;
        
        running.store(false);
        pose_provider->stop();
        
        if(processing_thread.joinable()) {
            processing_thread.join();
        }
        
        std::cout << "Live voxel mapper stopped" << std::endl;
    }
    
    // Get current voxel grid
    const DualVoxelGrid* get_voxel_grid() const { return voxel_grid.get(); }
    
    // Get processing statistics
    const ProcessingStats& get_stats() const { return stats; }
    
    // Save voxel grid to file (for development/debugging)
    bool save_voxel_grid(const std::string& filename) const {
        std::ofstream ofs(filename, std::ios::binary);
        if(!ofs) {
            std::cerr << "Cannot open output file: " << filename << std::endl;
            return false;
        }
        
        const GridConfig& config = voxel_grid->get_config();
        int N = config.N;
        float voxel_size = config.voxel_size;
        
        // Write metadata
        ofs.write(reinterpret_cast<const char*>(&N), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(&voxel_size), sizeof(float));
        
        // Write dynamic voxel data
        for(int i = 0; i < voxel_grid->get_total_voxels(); i++) {
            float val = voxel_grid->get_dynamic_voxel(i);
            ofs.write(reinterpret_cast<const char*>(&val), sizeof(float));
        }
        
        // Write static voxel data
        for(int i = 0; i < voxel_grid->get_total_voxels(); i++) {
            float val = voxel_grid->get_static_voxel(i);
            ofs.write(reinterpret_cast<const char*>(&val), sizeof(float));
        }
        
        ofs.close();
        std::cout << "Saved voxel grid to " << filename << std::endl;
        return true;
    }
    
private:
    // Main processing loop
    void processing_loop() {
        while(running.load()) {
            // Apply decay to voxel grids
            voxel_grid->apply_decay();
            
            // Sleep for a short time
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        }
    }
    
    // Process incoming camera frame
    void process_frame(const CameraFrame& frame) {
        auto start_time = std::chrono::steady_clock::now();
        
        // Check if we need to recenter the grid
        if(voxel_grid->needs_recentering(frame.camera_pose.position)) {
            update_grid_origin(frame.camera_pose);
        }
        
        // Process frame for motion detection
        if(prev_frame_valid) {
            ImageGray current_img = camera_frame_to_image_gray(frame);
            // Detect motion
            MotionMask motion_mask;
            if(use_adaptive_threshold) {
                if(use_noise_filtering) {
                    motion_mask = detect_motion_filtered(prev_frame, current_img, motion_threshold, min_region_size);
                } else {
                    motion_mask = detect_motion_adaptive(prev_frame, current_img, motion_threshold, adaptive_threshold_percentile);
                }
            } else {
                if(use_noise_filtering) {
                    motion_mask = detect_motion_filtered(prev_frame, current_img, motion_threshold, min_region_size);
                } else {
                    motion_mask = detect_motion(prev_frame, current_img, motion_threshold);
                }
            }

            // Update statistics
            update_stats(motion_mask);

            // Process motion pixels
            if(motion_mask.count_changed() > 0) {
                process_motion_pixels(motion_mask, frame);
            }
        }

        // Store current frame for next iteration
        prev_frame = camera_frame_to_image_gray(frame);
        prev_frame_valid = true;
        
        // Update timing
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        last_frame_time = end_time;
        
        // Update processing time statistics
        float processing_time_ms = duration.count() / 1000.0f;
        stats.avg_processing_time_ms = 0.9f * stats.avg_processing_time_ms + 0.1f * processing_time_ms;
    }
    
    // Process motion pixels with ray casting
    void process_motion_pixels(const MotionMask& motion_mask, const CameraFrame& frame) {
        const GridConfig& config = voxel_grid->get_config();
        const Pose& camera_pose = frame.camera_pose;
        Mat3 camera_rot = camera_pose.get_rotation_matrix();
        float focal_length = frame.get_focal_length();
        
        // Collect motion pixel coordinates and values
        std::vector<std::pair<int, int>> motion_pixels;
        std::vector<float> motion_values;
        
        for(int v = 0; v < motion_mask.height; v++) {
            for(int u = 0; u < motion_mask.width; u++) {
                if(motion_mask.is_changed(u, v)) {
                    motion_pixels.emplace_back(u, v);
                    motion_values.push_back(motion_mask.get_diff(u, v));
                }
            }
        }
        
        if(motion_pixels.empty()) return;
        
        // Cast rays for motion pixels
        auto ray_results = cast_rays_batch(
            motion_pixels, motion_values, camera_pose.position, 
            camera_rot, focal_length, *voxel_grid);
        
        // Accumulate into voxel grid using thread tiles
        auto thread_tile = voxel_grid->create_thread_tile();
        
        for(const auto& ray_result : ray_results) {
            for(const auto& step : ray_result.steps) {
                int voxel_idx = voxel_grid->voxel_to_index(step.ix, step.iy, step.iz);
                
                // Compute distance-aware weight
                float weight = compute_distance_weight(
                    step.distance, focal_length, config.voxel_size);
                
                // Accumulate into dynamic grid (motion-based)
                float value = ray_result.pixel_value * weight;
                voxel_grid->accumulate_dynamic_tile(voxel_idx, value, *thread_tile);
            }
        }
        
        // Commit thread tile to main grid
        voxel_grid->commit_tile(*thread_tile);
        
        // Also update static grid with lower intensity (for dense mapping)
        update_static_grid(frame);
    }
    
    // Update static grid with dense ray casting
    void update_static_grid(const CameraFrame& frame) {
        const GridConfig& config = voxel_grid->get_config();
        const Pose& camera_pose = frame.camera_pose;
        Mat3 camera_rot = camera_pose.get_rotation_matrix();
        float focal_length = frame.get_focal_length();
        
        // Sample pixels at regular intervals for static mapping
        int sample_interval = 8; // Sample every 8th pixel
        std::vector<std::pair<int, int>> sample_pixels;
        std::vector<float> sample_values;
        
        for(int v = 0; v < frame.height; v += sample_interval) {
            for(int u = 0; u < frame.width; u += sample_interval) {
                sample_pixels.emplace_back(u, v);
                sample_values.push_back(frame.pixels[frame.get_index(u, v)]);
            }
        }
        
        if(sample_pixels.empty()) return;
        
        // Cast rays for sampled pixels
        auto ray_results = cast_rays_batch(
            sample_pixels, sample_values, camera_pose.position, 
            camera_rot, focal_length, *voxel_grid);
        
        // Accumulate into static grid
        auto thread_tile = voxel_grid->create_thread_tile();
        
        for(const auto& ray_result : ray_results) {
            for(const auto& step : ray_result.steps) {
                int voxel_idx = voxel_grid->voxel_to_index(step.ix, step.iy, step.iz);
                
                // Compute distance-aware weight (lower for static mapping)
                float weight = compute_distance_weight(
                    step.distance, focal_length, config.voxel_size, 0.05f) * 0.1f;
                
                // Accumulate into static grid
                float value = ray_result.pixel_value * weight;
                voxel_grid->accumulate_static_tile(voxel_idx, value, *thread_tile);
            }
        }
        
        // Commit thread tile to main grid
        voxel_grid->commit_tile(*thread_tile);
    }
    
    // Update grid origin based on drone position
    void update_grid_origin(const Pose& pose) {
        voxel_grid->recenter_origin(pose.position);
    }
    
    // Update processing statistics
    void update_stats(const MotionMask& motion_mask) {
        stats.total_frames++;
        
        if(motion_mask.count_changed() > 0) {
            stats.motion_frames++;
        }
        
        float motion_percentage = motion_mask.get_change_percentage();
        stats.avg_motion_percentage = 0.9f * stats.avg_motion_percentage + 0.1f * motion_percentage;
        
        stats.last_update = std::chrono::steady_clock::now();
    }
};

//----------------------------------------------
// Main Function (for testing/development)
//----------------------------------------------
int main(int argc, char** argv) {
    std::cout << "Live Voxel Mapper - Development Mode" << std::endl;

    // Defaults
    int grid_size = 160;
    float voxel_size = 0.5f;
    float horizon = 40.0f;
    int duration_seconds = 30;
    std::string output_file = "live_voxel_grid.bin";

    // Tiny CLI parser
    for(int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if(i + 1 >= argc) return "";
            return argv[++i];
        };
        if(arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: embedded_voxel_mapper [options]\n"
                << "  --grid N          Grid size N^3 (default 160)\n"
                << "  --voxel M         Voxel size in meters (default 0.5)\n"
                << "  --horizon M       Ray horizon distance in meters (default 40.0)\n"
                << "  --duration S      Run duration in seconds (default 30)\n"
                << "  --out FILE        Output voxel grid file (default live_voxel_grid.bin)\n"
                << "  --help, -h        Show this help\n";
            return 0;
        } else if(arg == "--grid")        grid_size = std::stoi(next());
        else if(arg == "--voxel")         voxel_size = std::stof(next());
        else if(arg == "--horizon")       horizon = std::stof(next());
        else if(arg == "--duration")      duration_seconds = std::stoi(next());
        else if(arg == "--out")           output_file = next();
        else {
            std::cerr << "Unknown argument: " << arg << " (try --help)" << std::endl;
            return 1;
        }
    }

    std::cout << "Config: grid=" << grid_size << "^3 voxel=" << voxel_size
              << "m horizon=" << horizon << "m duration=" << duration_seconds << "s" << std::endl;

    // Create grid configuration
    GridConfig config;
    config.N = grid_size;
    config.voxel_size = voxel_size;
    config.horizon_distance = horizon;
    config.origin = Vec3(0, 0, 10); // Start at 10m altitude

    // Create live voxel mapper
    LiveVoxelMapper mapper(config);

    // Configure parameters
    mapper.set_motion_threshold(3.0f);
    mapper.set_use_adaptive_threshold(true);
    mapper.set_use_noise_filtering(true);
    mapper.set_min_region_size(4);

    // Start the system
    if(!mapper.start()) {
        std::cerr << "Failed to start live voxel mapper" << std::endl;
        return 1;
    }

    // Run for a while to collect data
    std::cout << "Running for " << duration_seconds << " seconds to collect data..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(duration_seconds));

    // Stop and save results
    mapper.stop();

    // Save voxel grid
    mapper.save_voxel_grid(output_file);

    // Print statistics
    const auto& stats = mapper.get_stats();
    std::cout << "\nProcessing Statistics:" << std::endl;
    std::cout << "Total frames: " << stats.total_frames << std::endl;
    std::cout << "Motion frames: " << stats.motion_frames << std::endl;
    std::cout << "Average motion percentage: " << stats.avg_motion_percentage * 100.0f << "%" << std::endl;
    std::cout << "Average processing time: " << stats.avg_processing_time_ms << " ms" << std::endl;

    std::cout << "\nLive voxel mapping completed!" << std::endl;
    return 0;
}
