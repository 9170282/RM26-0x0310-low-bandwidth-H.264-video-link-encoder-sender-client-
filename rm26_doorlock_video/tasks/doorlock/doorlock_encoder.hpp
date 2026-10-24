#ifndef TASKS__DOORLOCK__DOORLOCK_ENCODER_HPP_
#define TASKS__DOORLOCK__DOORLOCK_ENCODER_HPP_

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gst/gst.h>
#include <opencv2/opencv.hpp>

#include "tools/thread_safe_queue.hpp"

namespace doorlock
{
// 低带宽落点图传编码器（Pacific_doorlock_sniper）：
// 准星区域中心裁剪放大 + 静态简化 + H.264 低码率压缩。
// push 帧接口（线程安全），压缩结果按编码帧（access unit）回调输出，发送协议由 sink 决定。
class DoorlockEncoder
{
public:
  // 每个 H.264 编码帧回调一次（frame_id 由编码器递增分配，ts_ns 为编码时刻）
  using FrameHandler = std::function<void(
    uint64_t frame_id, int64_t ts_ns, const uint8_t * data, size_t size)>;

  explicit DoorlockEncoder(const std::string & config_path);
  ~DoorlockEncoder();

  // 深拷贝后入队，由工作线程处理，不阻塞调用者（主循环/相机线程）
  void encode(const cv::Mat & frame, int64_t timestamp_ns = -1);

  void set_frame_handler(FrameHandler handler);

private:
  struct Frame
  {
    cv::Mat img;
    int64_t timestamp_ns;
  };

  void worker_loop();   // 取帧 → 预处理 → 推 gst → 拉编码帧 → 回调
  void display_loop();  // 4 窗口 + debug_dump
  void initialize_gstreamer();
  void shutdown_gstreamer();
  void push_frame_to_gstreamer(const cv::Mat & frame);
  void pull_stream_and_emit();
  void poll_gst_bus();
  cv::Mat preprocess_image(const cv::Mat & input, cv::Mat * roi_downsample, cv::Mat * static_removed);

  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * appsink_ = nullptr;
  GstBus * bus_ = nullptr;

  tools::ThreadSafeQueue<Frame, true> frame_queue_{3};
  std::thread worker_thread_;
  std::thread display_thread_;
  std::atomic<bool> running_ = true;
  std::atomic<bool> display_running_ = false;

  std::mutex handler_mutex_;
  FrameHandler frame_handler_;  // 仅在 worker 线程调用

  uint64_t encoded_frame_id_ = 0;
  uint32_t frame_count_ = 0;
  uint64_t emitted_bytes_ = 0;
  int64_t last_telemetry_ns_ = 0;

  // 显示帧槽
  std::mutex frame_mutex_;
  cv::Mat display_raw_frame_;
  cv::Mat display_roi_frame_;
  cv::Mat display_static_frame_;
  cv::Mat display_frame_;
  uint64_t display_frame_counter_ = 0;

  // 帧率控制 / 背景建模 / 拖影历史
  int64_t last_encode_stamp_ns_ = 0;
  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;

  // 参数（yaml doorlock 段，默认值取 Pacific launch）
  int crop_size_ = 800;
  int output_size_ = 300;
  int output_fps_ = 60;
  int target_bitrate_ = 80;  // kbps
  std::string x264_preset_ = "veryfast";
  bool static_simplify_ = true;
  int motion_threshold_ = 14;
  int motion_erode_px_ = 2;
  int motion_dilate_px_ = 6;
  int motion_trail_frames_ = 90;
  double trail_disable_motion_ratio_ = 0.30;
  double bg_update_alpha_ = 0.01;
  double bg_blur_sigma_ = 1.8;
  int center_clear_size_ = 150;
  bool force_monochrome_ = false;
  bool enable_display_ = true;
  bool debug_dump_enable_ = false;
  int debug_dump_every_n_frames_ = 20;
  bool debug_dump_save_raw_ = true;
  bool debug_dump_save_roi_ = true;
  bool debug_dump_save_static_ = true;
  bool debug_dump_save_final_ = true;
  std::string debug_dump_dir_ = "logs/doorlock_debug_imgs";
};

}  // namespace doorlock

#endif  // TASKS__DOORLOCK__DOORLOCK_ENCODER_HPP_
