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
#include "odometry/stereo_vo.hpp"
#include "utils/types.hpp"

namespace {

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
  const auto converted_bytes = static_cast<size_t>(converted.total() * converted.elemSize());
  auto       converted_data =
    rerun::Collection<uint8_t>::borrow(converted.data, converted_bytes);

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
  const auto quat =
    rerun::datatypes::Quaternion::from_wxyz(static_cast<float>(q.w()),
                                            static_cast<float>(q.x()),
                                            static_cast<float>(q.y()),
                                            static_cast<float>(q.z()));
  return rerun::Transform3D(translation, rerun::Rotation3D(quat), true);
}

}  // namespace

int main(int argc, char** argv) {
  LogI("Starting VO application");

  const auto project_root = std::filesystem::path(__FILE__).parent_path().parent_path();

  std::filesystem::path dataset_path = project_root
                                       / "datasets/EUROC/vicon_room1/V1_01_easy";

  omni_slam::EurocLoader loader;
  if (!loader.Initialize(dataset_path.string())) {
    LogE("Failed to initialize EuRoC loader");
    return -1;
  }

  if (!loader.HasCameraData()) {
    LogE("No camera data available in dataset");
    return -1;
  }

  omni_slam::StereoVO   stereo_vo;
  std::filesystem::path config_path = project_root / "configs/svo_euroc.json";

  if (!stereo_vo.Initialize(config_path.string())) {
    LogE("Failed to initialize VO pipeline");
    return -1;
  }

  rerun::RecordingStream rec("stereo_vo");
  rec.spawn().exit_on_failure();

  omni_slam::DatasetSimulator simulator(loader);
  simulator.SetCameraCallback(
    [&stereo_vo](int64_t                                          timestamp_ns,
                 const std::vector<cv::Mat>&                    images,
                 const std::vector<omni_slam::CameraParameter>& camera_parameters) {
      stereo_vo.OnCameraFrame(timestamp_ns, images, camera_parameters);
    });

  simulator.Start();
  stereo_vo.Run();

  int64_t last_timestamp = std::numeric_limits<int64_t>::min();
  omni_slam::OdometryResult result;

  while (loader.HasCameraData()) {
    if (stereo_vo.FetchResult(result) && result.timestamp_ns != last_timestamp) {
      last_timestamp = result.timestamp_ns;
      rec.set_time_duration_nanos("t", result.timestamp_ns);

      if (result.images.size() > 0) {
        rec.log("cam0/image", MakeRerunImage(result.images[0]));
      }
      if (result.images.size() > 1) {
        rec.log("cam1/image", MakeRerunImage(result.images[1]));
      }

      if (!result.tracking.uvs.empty()) {
        const auto& uvs = result.tracking.uvs[0];
        std::vector<std::array<float, 2>> points2d;
        std::vector<rerun::components::Radius> radii;
        std::vector<rerun::components::Color>  colors;
        points2d.reserve(uvs.size());
        radii.reserve(uvs.size());
        colors.reserve(uvs.size());
        for (const auto& uv : uvs) {
          points2d.push_back({uv.x, uv.y});
          radii.emplace_back(rerun::components::Radius::ui_points(2.0f));
          colors.emplace_back(255, 255, 0, 200);
        }
        rec.log("cam0/points",
                rerun::Points2D(points2d).with_radii(radii).with_colors(colors));
      }

      if (!result.map_points.empty()) {
        std::vector<std::array<float, 3>> points3d;
        points3d.reserve(result.map_points.size());
        for (const auto& packed : result.map_points) {
          points3d.push_back({packed.x(), packed.y(), packed.z()});
        }
        rec.log("world/map_points", rerun::Points3D(points3d));
      }

      for (size_t i = 0; i < result.T_w_b_window.size(); ++i) {
        const std::string path = "world/window/body_" + std::to_string(i);
        rec.log(path, MakeTransform(result.T_w_b_window[i]));
      }

      if (!result.T_w_b_window.empty()) {
        std::vector<rerun::components::Position3D> origins;
        std::vector<rerun::components::Vector3D>   vectors;
        std::vector<rerun::components::Color>      colors;
        std::vector<rerun::components::Radius>     radii;

        const float axis_length = 0.2f;
        const float axis_radius = 0.01f;

        origins.reserve(result.T_w_b_window.size() * 3);
        vectors.reserve(result.T_w_b_window.size() * 3);
        colors.reserve(result.T_w_b_window.size() * 3);
        radii.reserve(result.T_w_b_window.size() * 3);

        for (const auto& T_w_b : result.T_w_b_window) {
          const Eigen::Vector3d t = T_w_b.translation();
          const Eigen::Matrix3d R = T_w_b.so3().matrix();

          const Eigen::Vector3d x = R.col(0) * axis_length;
          const Eigen::Vector3d y = R.col(1) * axis_length;
          const Eigen::Vector3d z = R.col(2) * axis_length;

          const rerun::components::Position3D origin(static_cast<float>(t.x()),
                                                     static_cast<float>(t.y()),
                                                     static_cast<float>(t.z()));

          origins.push_back(origin);
          vectors.emplace_back(static_cast<float>(x.x()),
                               static_cast<float>(x.y()),
                               static_cast<float>(x.z()));
          colors.emplace_back(255, 0, 0, 255);
          radii.emplace_back(rerun::components::Radius::scene_units(axis_radius));

          origins.push_back(origin);
          vectors.emplace_back(static_cast<float>(y.x()),
                               static_cast<float>(y.y()),
                               static_cast<float>(y.z()));
          colors.emplace_back(0, 255, 0, 255);
          radii.emplace_back(rerun::components::Radius::scene_units(axis_radius));

          origins.push_back(origin);
          vectors.emplace_back(static_cast<float>(z.x()),
                               static_cast<float>(z.y()),
                               static_cast<float>(z.z()));
          colors.emplace_back(0, 0, 255, 255);
          radii.emplace_back(rerun::components::Radius::scene_units(axis_radius));
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

  stereo_vo.Shutdown();
  LogI("VO application finished");

  return 0;
}
