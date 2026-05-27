#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>

struct FrameEntry {
    double timestamp = 0.0;
    std::string rgb;
    std::string depth;
};

struct ImuEntry {
    double timestamp = 0.0;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
};

static std::vector<FrameEntry> LoadAssociations(const std::string& path) {
    std::ifstream file(path);
    std::vector<FrameEntry> frames;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream ss(line);
        FrameEntry frame;
        double depth_timestamp = 0.0;
        ss >> frame.timestamp >> frame.rgb >> depth_timestamp >> frame.depth;
        if (!frame.rgb.empty() && !frame.depth.empty()) {
            frames.push_back(frame);
        }
    }
    return frames;
}

static std::vector<ImuEntry> LoadImu(const std::string& path) {
    std::ifstream file(path);
    std::vector<ImuEntry> measurements;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (line.rfind("timestamp", 0) == 0) {
            continue;
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        ImuEntry imu;
        ss >> imu.timestamp >> imu.ax >> imu.ay >> imu.az >> imu.gx >> imu.gy >> imu.gz;
        if (ss) {
            measurements.push_back(imu);
        }
    }
    return measurements;
}

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "Usage: " << argv[0]
                  << " path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_imu_csv"
                  << std::endl;
        return 1;
    }

    const std::string vocabulary = argv[1];
    const std::string settings = argv[2];
    const std::string sequence = argv[3];
    const std::string associations = argv[4];
    const std::string imu_csv = argv[5];

    std::vector<FrameEntry> frames = LoadAssociations(associations);
    std::vector<ImuEntry> imu = LoadImu(imu_csv);

    if (frames.empty()) {
        std::cerr << "No RGB-D frames found in association file." << std::endl;
        return 1;
    }
    if (imu.empty()) {
        std::cerr << "No IMU measurements found in imu.csv." << std::endl;
        return 1;
    }

    ORB_SLAM3::System slam(vocabulary, settings, ORB_SLAM3::System::IMU_RGBD, false);
    const float image_scale = slam.GetImageScale();
    std::vector<float> tracking_times(frames.size(), 0.0f);

    size_t imu_index = 0;
    double previous_frame_time = frames.front().timestamp;

    std::cout << "Start processing RGB-D-inertial sequence" << std::endl;
    std::cout << "Frames: " << frames.size() << ", IMU measurements: " << imu.size() << std::endl;

    for (size_t i = 0; i < frames.size(); ++i) {
        const FrameEntry& frame = frames[i];
        cv::Mat rgb = cv::imread(sequence + "/" + frame.rgb, cv::IMREAD_UNCHANGED);
        cv::Mat depth = cv::imread(sequence + "/" + frame.depth, cv::IMREAD_UNCHANGED);
        if (rgb.empty() || depth.empty()) {
            std::cerr << "Failed to load frame " << i << ": " << frame.rgb << " / " << frame.depth << std::endl;
            return 1;
        }

        if (image_scale != 1.0f) {
            const int width = static_cast<int>(rgb.cols * image_scale);
            const int height = static_cast<int>(rgb.rows * image_scale);
            cv::resize(rgb, rgb, cv::Size(width, height));
            // Nearest-neighbour for depth: bilinear interpolation invents
            // depth values across discontinuities (object/background edges).
            cv::resize(depth, depth, cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);
        }

        // IMU window for this frame: (previous_frame_time, frame.timestamp].
        // For frame 0 we include all IMU samples up to and including the
        // first frame timestamp so initialization sees pre-roll motion.
        std::vector<ORB_SLAM3::IMU::Point> imu_points;
        while (imu_index < imu.size() && imu[imu_index].timestamp <= frame.timestamp) {
            if (i == 0 || imu[imu_index].timestamp > previous_frame_time) {
                const ImuEntry& m = imu[imu_index];
                imu_points.emplace_back(m.ax, m.ay, m.az, m.gx, m.gy, m.gz, m.timestamp);
            }
            ++imu_index;
        }

        const auto t1 = std::chrono::steady_clock::now();
        slam.TrackRGBD(rgb, depth, frame.timestamp, imu_points);
        const auto t2 = std::chrono::steady_clock::now();
        tracking_times[i] = std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1).count();

        if (i + 1 < frames.size()) {
            const double frame_dt = std::max(0.0, frames[i + 1].timestamp - frame.timestamp);
            const double sleep_dt = std::min(0.03, std::max(0.0, frame_dt - tracking_times[i]));
            if (sleep_dt > 0.0) {
                usleep(static_cast<useconds_t>(sleep_dt * 1e6));
            }
        }
        previous_frame_time = frame.timestamp;
    }

    // Save trajectories BEFORE Shutdown. ORB-SLAM3's Shutdown() can SIGSEGV
    // on short IMU-inertial sequences where initialization never converged;
    // saving first guarantees a usable keyframe trajectory even if Shutdown
    // crashes. SaveKeyFrameTrajectoryTUM is cheaper and more robust than
    // SaveTrajectoryTUM, so it runs first.
    slam.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    slam.SaveTrajectoryTUM("CameraTrajectory.txt");

    std::sort(tracking_times.begin(), tracking_times.end());
    float total_tracking = 0.0f;
    for (float value : tracking_times) {
        total_tracking += value;
    }
    std::cout << "median tracking time: " << tracking_times[tracking_times.size() / 2] << std::endl;
    std::cout << "mean tracking time: " << total_tracking / tracking_times.size() << std::endl;

    slam.Shutdown();
    return 0;
}
