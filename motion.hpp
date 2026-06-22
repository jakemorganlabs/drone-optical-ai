#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

//----------------------------------------------
// Motion Detection Structures
//----------------------------------------------
struct MotionMask {
    int width;
    int height;
    std::vector<bool> changed;
    std::vector<float> diff; // absolute difference
    
    MotionMask() : width(0), height(0) {}
    MotionMask(int w, int h) : width(w), height(h) {
        changed.resize(width * height, false);
        diff.resize(width * height, 0.0f);
    }
    
    // Get pixel index
    int get_index(int u, int v) const {
        return v * width + u;
    }
    
    // Check if pixel changed
    bool is_changed(int u, int v) const {
        if(u < 0 || u >= width || v < 0 || v >= height) return false;
        return changed[get_index(u, v)];
    }
    
    // Get pixel difference
    float get_diff(int u, int v) const {
        if(u < 0 || u >= width || v < 0 || v >= height) return 0.0f;
        return diff[get_index(u, v)];
    }
    
    // Count total changed pixels
    int count_changed() const {
        int count = 0;
        for(bool changed_pixel : changed) {
            if(changed_pixel) count++;
        }
        return count;
    }
    
    // Get percentage of changed pixels
    float get_change_percentage() const {
        if(width * height == 0) return 0.0f;
        return static_cast<float>(count_changed()) / (width * height);
    }
};

//----------------------------------------------
// Image Structure
//----------------------------------------------
struct ImageGray {
    int width;
    int height;
    std::vector<float> pixels;  // grayscale float values
    
    ImageGray() : width(0), height(0) {}
    ImageGray(int w, int h) : width(w), height(h) {
        pixels.resize(width * height, 0.0f);
    }
    
    // Get pixel index
    int get_index(int u, int v) const {
        return v * width + u;
    }
    
    // Get pixel value
    float get_pixel(int u, int v) const {
        if(u < 0 || u >= width || v < 0 || v >= height) return 0.0f;
        return pixels[get_index(u, v)];
    }
    
    // Set pixel value
    void set_pixel(int u, int v, float value) {
        if(u < 0 || u >= width || v < 0 || v >= height) return;
        pixels[get_index(u, v)] = value;
    }
    
    // Check if images have same dimensions
    bool same_dimensions(const ImageGray& other) const {
        return width == other.width && height == other.height;
    }
    
    // Get total pixels
    int get_total_pixels() const {
        return width * height;
    }
};

//----------------------------------------------
// Motion Detection Functions
//----------------------------------------------

// Detect motion by absolute difference
// Returns a boolean mask + the difference for each pixel
MotionMask detect_motion(const ImageGray &prev, const ImageGray &next, float threshold) {
    MotionMask mm;
    if(!prev.same_dimensions(next)) {
        std::cerr << "Images differ in size. Can't do motion detection!\n";
        return mm;
    }
    
    mm = MotionMask(prev.width, prev.height);
    
    for(int v = 0; v < mm.height; v++) {
        for(int u = 0; u < mm.width; u++) {
            int idx = mm.get_index(u, v);
            float d = std::fabs(prev.get_pixel(u, v) - next.get_pixel(u, v));
            mm.diff[idx] = d;
            mm.changed[idx] = (d > threshold);
        }
    }
    
    return mm;
}

// Adaptive threshold based on image statistics
float compute_adaptive_threshold(const ImageGray &prev, const ImageGray &next, 
                                float base_threshold = 2.0f, float percentile = 95.0f) {
    if(!prev.same_dimensions(next)) {
        return base_threshold;
    }
    
    std::vector<float> differences;
    differences.reserve(prev.get_total_pixels());
    
    // Compute all differences
    for(int v = 0; v < prev.height; v++) {
        for(int u = 0; u < prev.width; u++) {
            float diff = std::fabs(prev.get_pixel(u, v) - next.get_pixel(u, v));
            differences.push_back(diff);
        }
    }
    
    // Sort differences
    std::sort(differences.begin(), differences.end());
    
    // Find percentile threshold
    int percentile_idx = static_cast<int>((percentile / 100.0f) * differences.size());
    percentile_idx = std::min(percentile_idx, static_cast<int>(differences.size() - 1));
    
    float adaptive_threshold = differences[percentile_idx];
    
    // Use the larger of base threshold or adaptive threshold
    return std::max(base_threshold, adaptive_threshold);
}

// Motion detection with adaptive threshold
MotionMask detect_motion_adaptive(const ImageGray &prev, const ImageGray &next, 
                                 float base_threshold = 2.0f, float percentile = 95.0f) {
    float threshold = compute_adaptive_threshold(prev, next, base_threshold, percentile);
    return detect_motion(prev, next, threshold);
}

// Motion detection with noise filtering
MotionMask detect_motion_filtered(const ImageGray &prev, const ImageGray &next, 
                                 float threshold, int min_region_size = 4) {
    MotionMask mm = detect_motion(prev, next, threshold);
    
    // Simple noise filtering: remove isolated pixels
    if(min_region_size > 1) {
        std::vector<bool> filtered_changed = mm.changed;
        
        for(int v = 1; v < mm.height - 1; v++) {
            for(int u = 1; u < mm.width - 1; u++) {
                int idx = mm.get_index(u, v);
                if(mm.changed[idx]) {
                    // Count neighbors that also changed
                    int neighbor_count = 0;
                    for(int dv = -1; dv <= 1; dv++) {
                        for(int du = -1; du <= 1; du++) {
                            if(du == 0 && dv == 0) continue;
                            if(mm.is_changed(u + du, v + dv)) {
                                neighbor_count++;
                            }
                        }
                    }
                    
                    // If too few neighbors, mark as unchanged
                    if(neighbor_count < min_region_size) {
                        filtered_changed[idx] = false;
                    }
                }
            }
        }
        
        mm.changed = filtered_changed;
    }
    
    return mm;
}

// Compute motion statistics
struct MotionStats {
    int total_pixels;
    int changed_pixels;
    float change_percentage;
    float max_diff;
    float mean_diff;
    float std_diff;
    
    MotionStats() : total_pixels(0), changed_pixels(0), change_percentage(0.0f),
                    max_diff(0.0f), mean_diff(0.0f), std_diff(0.0f) {}
};

MotionStats compute_motion_stats(const MotionMask& mm) {
    MotionStats stats;
    stats.total_pixels = mm.width * mm.height;
    stats.changed_pixels = mm.count_changed();
    stats.change_percentage = mm.get_change_percentage();
    
    if(stats.changed_pixels == 0) {
        return stats;
    }
    
    // Compute statistics only for changed pixels
    std::vector<float> changed_diffs;
    changed_diffs.reserve(stats.changed_pixels);
    
    for(size_t i = 0; i < mm.diff.size(); i++) {
        if(mm.changed[i]) {
            changed_diffs.push_back(mm.diff[i]);
        }
    }
    
    if(changed_diffs.empty()) {
        return stats;
    }
    
    // Find max
    stats.max_diff = *std::max_element(changed_diffs.begin(), changed_diffs.end());
    
    // Compute mean
    float sum = 0.0f;
    for(float diff : changed_diffs) {
        sum += diff;
    }
    stats.mean_diff = sum / changed_diffs.size();
    
    // Compute standard deviation
    float variance_sum = 0.0f;
    for(float diff : changed_diffs) {
        float dev = diff - stats.mean_diff;
        variance_sum += dev * dev;
    }
    stats.std_diff = std::sqrt(variance_sum / changed_diffs.size());
    
    return stats;
}
