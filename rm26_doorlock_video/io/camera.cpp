// 精简版相机入口：本独立工程只保留海康 (hikrobot / MVS SDK) 相机。
// 原始 sp_vision_25 工程还支持 mindvision / daheng / usbcamera，
// 为了不依赖多家厂商 SDK，这里只保留 hikrobot 分支。
#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    auto frame_rate = tools::read<double>(yaml, "usb_frame_rate", 150.0);
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, frame_rate);
  } else {
    throw std::runtime_error(
      "本精简工程只支持 camera_name: hikrobot, 当前为: " + camera_name);
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
