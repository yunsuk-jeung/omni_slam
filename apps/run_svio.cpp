#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <rerun.hpp>

#include "utils/logger.hpp"
#include "device/dataset_simulator.hpp"
#include "device/euroc_loader.hpp"
#include "odometry/odometry_result.hpp"
#include "odometry/stereo_vio.hpp"
#include "utils/types.hpp"

namespace {

constexpr float kTrackPointRadiusUi = 3.0f;
constexpr float kMapPointRadiusUi   = 1.5f;
constexpr float kTrajectoryRadiusUi = 1.0f;
constexpr float kAxisLength         = 0.2f;
constexpr float kAxisRadius         = 0.01f;
constexpr float kOriginAxisLength   = 1.0f;
constexpr float kOriginAxisRadius   = 0.03f;

const rerun::components::Color kTrackColor(255, 255, 0, 200);
const rerun::components::Color kMapColor(0, 255, 255, 200);
const rerun::components::Color kTrajectoryColor(255, 255, 255, 200);
const rerun::components::Color kAxisXColor(255, 0, 0, 255);
const rerun::components::Color kAxisYColor(0, 255, 0, 255);
const rerun::components::Color kAxisZColor(0, 0, 255, 255);

rerun::Image MakeRerunImage(const cv::Mat& image) {
  if (image.empty()) {
    return rerun::Image::clear_fields();
  }

  cv::Mat source = image;
  if (!source.isContinuous()) {
    source = source.clone();
  }

  const auto width  = static_cast<uint32_t>(source.cols);
  const auto height = static_cast<uint32_t>(source.rows);
  const auto bytes  = static_cast<size_t>(source.total() * source.elemSize());
  auto       data   = rerun::Collection<uint8_t>::borrow(source.data, bytes);

  if (source.type() == CV_8UC1) {
    return rerun::Image::from_grayscale8(std::move(data), {width, height});
  }
  if (source.type() == CV_8UC3) {
    return rerun::Image(std::move(data),
                        {width, height},
                        rerun::datatypes::ColorModel::BGR,
                        rerun::datatypes::ChannelDatatype::U8);
  }

  cv::Mat converted;
  source.convertTo(converted, CV_8U);
  if (!converted.isContinuous()) {
    converted = converted.clone();
  }
  const auto converted_bytes = static_cast<size_t>(converted.total()
                                                   * converted.elemSize());
  auto       converted_data  = rerun::Collection<uint8_t>::borrow(converted.data,
                                                           converted_bytes);

  if (converted.type() == CV_8UC1) {
    return rerun::Image::from_grayscale8(std::move(converted_data), {width, height});
  }
  if (converted.type() == CV_8UC3) {
    return rerun::Image(std::move(converted_data),
                        {width, height},
                        rerun::datatypes::ColorModel::BGR,
                        rerun::datatypes::ChannelDatatype::U8);
  }

  return rerun::Image::clear_fields();
}

rerun::Transform3D MakeTransform(const Sophus::SE3d& T) {
  const Eigen::Vector3d    t = T.translation();
  const Eigen::Quaterniond q = T.so3().unit_quaternion();

  rerun::components::Translation3D translation(static_cast<float>(t.x()),
                                               static_cast<float>(t.y()),
                                               static_cast<float>(t.z()));
  const auto quat = rerun::datatypes::Quaternion::from_wxyz(static_cast<float>(q.w()),
                                                            static_cast<float>(q.x()),
                                                            static_cast<float>(q.y()),
                                                            static_cast<float>(q.z()));
  return rerun::Transform3D(translation, rerun::Rotation3D(quat), true);
}

void LogOriginAxes(rerun::RecordingStream& rec) {
  std::vector<rerun::components::Position3D> origins;
  std::vector<rerun::components::Vector3D>   vectors;
  std::vector<rerun::components::Color>      colors;
  std::vector<rerun::components::Radius>     radii;

  origins.reserve(3);
  vectors.reserve(3);
  colors.reserve(3);
  radii.reserve(3);

  const rerun::components::Position3D origin(0.0f, 0.0f, 0.0f);

  origins.push_back(origin);
  vectors.emplace_back(kOriginAxisLength, 0.0f, 0.0f);
  colors.emplace_back(kAxisXColor);
  radii.emplace_back(rerun::components::Radius::scene_units(kOriginAxisRadius));

  origins.push_back(origin);
  vectors.emplace_back(0.0f, kOriginAxisLength, 0.0f);
  colors.emplace_back(kAxisYColor);
  radii.emplace_back(rerun::components::Radius::scene_units(kOriginAxisRadius));

  origins.push_back(origin);
  vectors.emplace_back(0.0f, 0.0f, kOriginAxisLength);
  colors.emplace_back(kAxisZColor);
  radii.emplace_back(rerun::components::Radius::scene_units(kOriginAxisRadius));

  rec.log_static("world/origin_axes",
          rerun::Arrows3D::from_vectors(vectors)
            .with_origins(origins)
            .with_colors(colors)
            .with_radii(radii));
}

}  // namespace

int main(int argc, char** argv) {
  LogI("Starting SVIO application");

  const auto project_root = std::filesystem::path(__FILE__).parent_path().parent_path();

  // std::filesystem::path dataset_path = project_root / "datasets/EUROC/V1_02_medium";
  std::filesystem::path dataset_path = project_root / "datasets/EUROC/V1_01_easy";

  omni_slam::EurocLoader loader;
  if (!loader.Setup(dataset_path.string())) {
    LogE("Failed to initialize EuRoC loader");
    return -1;
  }

  if (!loader.HasCameraData()) {
    LogE("No camera data available in dataset");
    return -1;
  }

  omni_slam::StereoVIO  stereo_vio;
  std::filesystem::path config_path = project_root / "configs/svio_euroc.json";

  if (!stereo_vio.Setup(config_path.string())) {
    LogE("Failed to initialize SVIO pipeline");
    return -1;
  }

  rerun::RecordingStream rec("stereo_svio");
  rec.spawn().exit_on_failure();
  LogOriginAxes(rec);

  omni_slam::DatasetSimulator simulator(loader);
  simulator.SetCameraCallback(
    [&stereo_vio](int64_t                                        timestamp_ns,
                  const std::vector<cv::Mat>&                    images,
                  const std::vector<omni_slam::CameraParameter>& camera_parameters) {
      stereo_vio.OnCameraFrame(timestamp_ns, images, camera_parameters);
    });
  simulator.SetImuCallback([&stereo_vio](const omni_slam::ImuData& imu_data) {
    stereo_vio.OnImuData(imu_data);
  });

  simulator.Start();
  stereo_vio.Run();

  int64_t                           last_timestamp = std::numeric_limits<int64_t>::min();
  omni_slam::OdometryResult         result;
  std::vector<std::array<float, 3>> trajectory_points;

  while (loader.HasCameraData()) {
    if (stereo_vio.FetchResult(result) && result.timestamp_ns != last_timestamp) {
      last_timestamp = result.timestamp_ns;
      rec.set_time_sequence("frame_id", static_cast<int64_t>(result.frame_id));
      rec.set_time_timestamp_nanos_since_epoch("frame_time", result.timestamp_ns);

      if (result.images.size() > 0) {
        rec.log("cam0/image", MakeRerunImage(result.images[0]));
      }
      if (result.images.size() > 1) {
        rec.log("cam1/image", MakeRerunImage(result.images[1]));
      }

      auto log_overlay = [&](size_t cam_idx, const std::string& points_path) {
        std::vector<std::array<float, 2>>      points2d;
        std::vector<rerun::components::Radius> radii;
        std::vector<rerun::components::Color>  colors;

        if (cam_idx < result.tracking.uvs.size()) {
          const auto&  uvs     = result.tracking.uvs[cam_idx];
          const size_t reserve = uvs.size()
                                 + ((cam_idx < result.map_point_uvs.size())
                                      ? result.map_point_uvs[cam_idx].size()
                                      : 0);
          points2d.reserve(reserve);
          radii.reserve(reserve);
          colors.reserve(reserve);
          for (const auto& uv : uvs) {
            points2d.push_back({uv.x, uv.y});
            radii.emplace_back(rerun::components::Radius::ui_points(kTrackPointRadiusUi));
            colors.emplace_back(kTrackColor);
          }
        }

        if (cam_idx < result.map_point_uvs.size()) {
          const auto& mp_uvs = result.map_point_uvs[cam_idx];
          if (points2d.empty()) {
            points2d.reserve(mp_uvs.size());
            radii.reserve(mp_uvs.size());
            colors.reserve(mp_uvs.size());
          }
          for (const auto& uv : mp_uvs) {
            points2d.push_back({uv.x, uv.y});
            radii.emplace_back(rerun::components::Radius::ui_points(kMapPointRadiusUi));
            colors.emplace_back(kMapColor);
          }
        }

        if (!points2d.empty()) {
          rec.log(points_path,
                  rerun::Points2D(points2d).with_radii(radii).with_colors(colors));
        }
      };

      log_overlay(0, "cam0/image");
      log_overlay(1, "cam1/image");

      if (!result.map_points.empty()) {
        std::vector<std::array<float, 3>> points3d;
        points3d.reserve(result.map_points.size());
        for (const auto& packed : result.map_points) {
          points3d.push_back({packed.x(), packed.y(), packed.z()});
        }
        rec.log("world/map_points", rerun::Points3D(points3d));
      }

      if (!result.T_w_b_window.empty()) {
        const auto&           T_w_b = result.T_w_b_window.back();
        const Eigen::Vector3d t     = T_w_b.translation();
        trajectory_points.push_back({static_cast<float>(t.x()),
                                     static_cast<float>(t.y()),
                                     static_cast<float>(t.z())});
        rerun::LineStrip3D strip(trajectory_points);
        rec.log("world/trajectory",
                rerun::LineStrips3D(strip)
                  .with_colors({kTrajectoryColor})
                  .with_radii(
                    {rerun::components::Radius::ui_points(kTrajectoryRadiusUi)}));
      }

      rec.log("world/window", rerun::Clear::RECURSIVE);
      for (size_t i = 0; i < result.T_w_b_window.size(); ++i) {
        const std::string path = "world/window/body_" + std::to_string(i);
        rec.log(path, MakeTransform(result.T_w_b_window[i]));
      }

      if (!result.T_w_b_window.empty() && !result.T_b_c.empty()) {
        for (size_t i = 0; i < result.T_w_b_window.size(); ++i) {
          const auto& T_w_b = result.T_w_b_window[i];
          for (size_t cam_idx = 0; cam_idx < result.T_b_c.size(); ++cam_idx) {
            const auto        T_w_c = T_w_b * result.T_b_c[cam_idx];
            const std::string path  = "world/window/cam_" + std::to_string(i) + "_"
                                     + std::to_string(cam_idx);
            rec.log(path, MakeTransform(T_w_c));
          }
        }
      }

      if (!result.T_w_b_window.empty()) {
        std::vector<rerun::components::Position3D> origins;
        std::vector<rerun::components::Vector3D>   vectors;
        std::vector<rerun::components::Color>      colors;
        std::vector<rerun::components::Radius>     radii;

        origins.reserve(result.T_w_b_window.size() * 3);
        vectors.reserve(result.T_w_b_window.size() * 3);
        colors.reserve(result.T_w_b_window.size() * 3);
        radii.reserve(result.T_w_b_window.size() * 3);

        for (const auto& T_w_b : result.T_w_b_window) {
          const Eigen::Vector3d t = T_w_b.translation();
          const Eigen::Matrix3d R = T_w_b.so3().matrix();

          const Eigen::Vector3d x = R.col(0) * kAxisLength;
          const Eigen::Vector3d y = R.col(1) * kAxisLength;
          const Eigen::Vector3d z = R.col(2) * kAxisLength;

          const rerun::components::Position3D origin(static_cast<float>(t.x()),
                                                     static_cast<float>(t.y()),
                                                     static_cast<float>(t.z()));

          origins.push_back(origin);
          vectors.emplace_back(static_cast<float>(x.x()),
                               static_cast<float>(x.y()),
                               static_cast<float>(x.z()));
          colors.emplace_back(kAxisXColor);
          radii.emplace_back(rerun::components::Radius::scene_units(kAxisRadius));

          origins.push_back(origin);
          vectors.emplace_back(static_cast<float>(y.x()),
                               static_cast<float>(y.y()),
                               static_cast<float>(y.z()));
          colors.emplace_back(kAxisYColor);
          radii.emplace_back(rerun::components::Radius::scene_units(kAxisRadius));

          origins.push_back(origin);
          vectors.emplace_back(static_cast<float>(z.x()),
                               static_cast<float>(z.y()),
                               static_cast<float>(z.z()));
          colors.emplace_back(kAxisZColor);
          radii.emplace_back(rerun::components::Radius::scene_units(kAxisRadius));
        }

        rec.log("world/body_axes",
                rerun::Arrows3D::from_vectors(vectors)
                  .with_origins(origins)
                  .with_colors(colors)
                  .with_radii(radii));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  simulator.Stop();

  stereo_vio.Shutdown();
  LogI("SVIO application finished");

  return 0;
}
