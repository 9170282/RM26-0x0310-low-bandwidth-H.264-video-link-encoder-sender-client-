#include "tasks/doorlock/doorlock_encoder.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace doorlock
{
namespace
{
int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

// speed-preset 是枚举属性, 直接传字符串会被 GObject 当成整型指针收集,
// 触发 "value ... is invalid or out of range" 警告且不生效, 故转成枚举整数再设。
// x264enc 的 GstX264EncPreset: 1=ultrafast .. 9=veryslow, 10=placebo。
int x264_preset_to_int(const std::string & preset)
{
  if (preset == "ultrafast") return 1;
  if (preset == "superfast") return 2;
  if (preset == "veryfast") return 3;
  if (preset == "faster") return 4;
  if (preset == "fast") return 5;
  if (preset == "medium") return 6;
  if (preset == "slow") return 7;
  if (preset == "slower") return 8;
  if (preset == "veryslow") return 9;
  if (preset == "placebo") return 10;
  return -1;
}
}  // namespace

DoorlockEncoder::DoorlockEncoder(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto d = yaml["doorlock"];

  crop_size_ = tools::read<int>(d, "crop_size", crop_size_);
  output_size_ = tools::read<int>(d, "output_size", output_size_);
  output_fps_ = tools::read<int>(d, "output_fps", output_fps_);
  target_bitrate_ = tools::read<int>(d, "target_bitrate", target_bitrate_);
  x264_preset_ = tools::read<std::string>(d, "x264_preset", x264_preset_);
  static_simplify_ = tools::read<bool>(d, "static_simplify", static_simplify_);
  motion_threshold_ = tools::read<int>(d, "motion_threshold", motion_threshold_);
  motion_erode_px_ = tools::read<int>(d, "motion_erode_px", motion_erode_px_);
  motion_dilate_px_ = tools::read<int>(d, "motion_dilate_px", motion_dilate_px_);
  motion_trail_frames_ = tools::read<int>(d, "motion_trail_frames", motion_trail_frames_);
  trail_disable_motion_ratio_ = tools::read<double>(d, "trail_disable_motion_ratio", trail_disable_motion_ratio_);
  bg_update_alpha_ = tools::read<double>(d, "bg_update_alpha", bg_update_alpha_);
  bg_blur_sigma_ = tools::read<double>(d, "bg_blur_sigma", bg_blur_sigma_);
  center_clear_size_ = tools::read<int>(d, "center_clear_size", center_clear_size_);
  force_monochrome_ = tools::read<bool>(d, "force_monochrome", force_monochrome_);
  enable_display_ = tools::read<bool>(d, "enable_display", enable_display_);
  debug_dump_enable_ = tools::read<bool>(d, "debug_dump_enable", debug_dump_enable_);
  debug_dump_every_n_frames_ =  tools::read<int>(d, "debug_dump_every_n_frames", debug_dump_every_n_frames_);
  debug_dump_save_raw_ = tools::read<bool>(d, "debug_dump_save_raw", debug_dump_save_raw_);
  debug_dump_save_roi_ = tools::read<bool>(d, "debug_dump_save_roi", debug_dump_save_roi_);
  debug_dump_save_static_ = tools::read<bool>(d, "debug_dump_save_static", debug_dump_save_static_);
  debug_dump_save_final_ = tools::read<bool>(d, "debug_dump_save_final", debug_dump_save_final_);
  debug_dump_dir_ = tools::read<std::string>(d, "debug_dump_dir", debug_dump_dir_);

  // 参数钳制
  crop_size_ = std::max(1, crop_size_);
  output_size_ = std::max(1, output_size_);
  output_fps_ = std::clamp(output_fps_, 1, 60);
  motion_trail_frames_ = std::clamp(motion_trail_frames_, 0, 300);
  motion_erode_px_ = std::clamp(motion_erode_px_, 0, 20);
  motion_dilate_px_ = std::clamp(motion_dilate_px_, 0, 20);
  trail_disable_motion_ratio_ = std::clamp(trail_disable_motion_ratio_, 0.0, 1.0);
  debug_dump_every_n_frames_ = std::max(1, debug_dump_every_n_frames_);
  std::transform(
    x264_preset_.begin(), x264_preset_.end(), x264_preset_.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (debug_dump_enable_) {
    const std::filesystem::path dump_dir = std::filesystem::path(debug_dump_dir_) / "encoder";
    std::error_code ec;
    std::filesystem::create_directories(dump_dir, ec);
    if (ec) {
      tools::logger()->warn(
        "[doorlock] Create debug dump dir failed: {} ({})", dump_dir.string(), ec.message());
      debug_dump_enable_ = false;
    }
  }

  gst_init(nullptr, nullptr);
  initialize_gstreamer();

  display_running_ = enable_display_;
  if (display_running_) display_thread_ = std::thread(&DoorlockEncoder::display_loop, this);

  worker_thread_ = std::thread(&DoorlockEncoder::worker_loop, this);

  tools::logger()->info(
    "[doorlock] Encoder ready: crop={} -> {}x{}@{}fps {}kbps x264={} "
    "static_simplify={} display={}",
    crop_size_, output_size_, output_size_, output_fps_, target_bitrate_, x264_preset_,
    static_simplify_ ? "on" : "off", enable_display_ ? "on" : "off");
}

DoorlockEncoder::~DoorlockEncoder()
{
  running_ = false;
  display_running_ = false;

  frame_queue_.push({cv::Mat(), 0});  // 唤醒 worker 退出

  if (worker_thread_.joinable()) worker_thread_.join();
  if (display_thread_.joinable()) display_thread_.join();

  shutdown_gstreamer();
  tools::logger()->info("[doorlock] Encoder destructed.");
}

void DoorlockEncoder::encode(const cv::Mat & frame, int64_t timestamp_ns)
{
  if (!running_ || frame.empty()) return;
  // 深拷贝：调用方主循环复用同一 Mat 缓冲，浅拷贝会被下一帧覆盖
  frame_queue_.push({frame.clone(), timestamp_ns});
}

void DoorlockEncoder::set_frame_handler(FrameHandler handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex_);
  frame_handler_ = std::move(handler);
}

void DoorlockEncoder::worker_loop()
{
  tools::logger()->info("[doorlock] worker thread started");

  while (true) {
    Frame frame;
    frame_queue_.pop(frame);
    if (frame.img.empty() || !running_) break;

    // 帧率节流：仅在低于 60fps 时抽帧
    if (output_fps_ < 60) {
      const int64_t stamp = (frame.timestamp_ns >= 0) ? frame.timestamp_ns : now_ns();
      const int64_t frame_interval_ns = 1000000000LL / output_fps_;
      if (last_encode_stamp_ns_ > 0 && (stamp - last_encode_stamp_ns_) < frame_interval_ns)
        continue;
      last_encode_stamp_ns_ = stamp;
    }

    cv::Mat roi_downsample;
    cv::Mat static_removed;
    cv::Mat processed = preprocess_image(frame.img, &roi_downsample, &static_removed);

    if (display_running_) {
      cv::Mat raw_preview;
      cv::resize(
        frame.img, raw_preview,
        cv::Size(std::max(1, frame.img.cols / 2), std::max(1, frame.img.rows / 2)), 0, 0,
        cv::INTER_AREA);
      std::lock_guard<std::mutex> lock(frame_mutex_);
      raw_preview.copyTo(display_raw_frame_);
      roi_downsample.copyTo(display_roi_frame_);
      static_removed.copyTo(display_static_frame_);
      processed.copyTo(display_frame_);
    }

    push_frame_to_gstreamer(processed);
    pull_stream_and_emit();
    poll_gst_bus();

    frame_count_++;
  }

  tools::logger()->info("[doorlock] worker thread stopped");
}

void DoorlockEncoder::initialize_gstreamer()
{
  pipeline_ = gst_pipeline_new("encoder_pipe");
  appsrc_ = gst_element_factory_make("appsrc", "source");
  appsink_ = gst_element_factory_make("appsink", "sink");
  GstElement * convert = gst_element_factory_make("videoconvert", "convert");
  GstElement * encoder = gst_element_factory_make("x264enc", "encoder");
  GstElement * parser = gst_element_factory_make("h264parse", "parser");

  if (!pipeline_ || !appsrc_ || !appsink_ || !convert || !encoder || !parser) {
    tools::logger()->error(
      "[doorlock] GStreamer element creation failed: pipeline={} appsrc={} appsink={} "
      "videoconvert={} x264enc={} h264parse={}",
      pipeline_ != nullptr, appsrc_ != nullptr, appsink_ != nullptr,
      convert != nullptr, encoder != nullptr, parser != nullptr);
    return;
  }

  GstCaps * caps = gst_caps_new_simple(
    "video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT, output_size_, "height",
    G_TYPE_INT, output_size_, "framerate", GST_TYPE_FRACTION, output_fps_, 1, nullptr);
  g_object_set(
    G_OBJECT(appsrc_), "caps", caps, "stream-type", 0, "format", GST_FORMAT_TIME, "is-live",
    TRUE, "do-timestamp", TRUE, nullptr);
  gst_caps_unref(caps);

  // 编码: bframes=0 + lookahead=0 + zerolatency
  const int key_int = 2 * output_fps_;
  int speed_preset = 3;  // veryfast
  if (!x264_preset_.empty() && x264_preset_ != "auto") {
    const int p = x264_preset_to_int(x264_preset_);
    if (p >= 0) {
      speed_preset = p;
    } else {
      tools::logger()->warn(
        "[doorlock] unknown x264 speed-preset '{}', using default", x264_preset_);
    }
  }

  g_object_set(
    G_OBJECT(encoder),
    "bitrate", target_bitrate_,
    "speed-preset", speed_preset,
    "tune", 0x00000004,         // zerolatency
    "byte-stream", TRUE,
    "key-int-max", key_int,
    "bframes", 0,
    "rc-lookahead", 0,
    "sync-lookahead", 0,
    "sliced-threads", TRUE,
    "aud", TRUE,
    "option-string", "repeat-headers=1:scenecut=0:ref=1:force-cfr=1",
    "pass", 0,
    nullptr);

  // 周期重复 SPS/PPS, 并保证下游输出 Annex-B byte-stream
  g_object_set(G_OBJECT(parser), "config-interval", -1, "disable-passthrough", TRUE, nullptr);

  GstCaps * h264_caps = gst_caps_new_simple(
    "video/x-h264", "stream-format", G_TYPE_STRING, "byte-stream", "alignment", G_TYPE_STRING,
    "au", nullptr);

  g_object_set(
    G_OBJECT(appsink_), "caps", h264_caps, "max-buffers", 5, "drop", FALSE, "emit-signals",
    FALSE, "sync", FALSE, nullptr);
  gst_caps_unref(h264_caps);

  gst_bin_add_many(GST_BIN(pipeline_), appsrc_, convert, encoder, parser, appsink_, nullptr);
  if (!gst_element_link_many(appsrc_, convert, encoder, parser, appsink_, nullptr)) {
    tools::logger()->error("[doorlock] GStreamer pipeline link failed");
    return;
  }

  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    tools::logger()->error("[doorlock] GStreamer pipeline start failed");
    return;
  }

  bus_ = gst_element_get_bus(pipeline_);
  tools::logger()->info("[doorlock] GStreamer encoder ready (zerolatency, byte-stream au)");
}

void DoorlockEncoder::shutdown_gstreamer()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (bus_) gst_object_unref(bus_);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    appsrc_ = nullptr;
    appsink_ = nullptr;
    bus_ = nullptr;
  }
}

void DoorlockEncoder::poll_gst_bus()
{
  if (!bus_) return;

  GstMessage * msg;
  while ((msg = gst_bus_pop(bus_))) {
    const bool is_error = (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR);
    if (is_error || GST_MESSAGE_TYPE(msg) == GST_MESSAGE_WARNING) {
      GError * err = nullptr;
      gchar * debug = nullptr;
      if (is_error) gst_message_parse_error(msg, &err, &debug);
      else gst_message_parse_warning(msg, &err, &debug);

      if (is_error)
        tools::logger()->error("[doorlock] gst error: {}", err ? err->message : "unknown");
      else
        tools::logger()->warn("[doorlock] gst warning: {}", err ? err->message : "unknown");

      if (err) g_error_free(err);
      if (debug) g_free(debug);
    }
    gst_message_unref(msg);
  }
}

cv::Mat DoorlockEncoder::preprocess_image(
  const cv::Mat & input, cv::Mat * roi_downsample, cv::Mat * static_removed)
{
  int x = (input.cols - crop_size_) / 2;
  int y = (input.rows - crop_size_) / 2;
  x = std::max(0, x);
  y = std::max(0, y);
  int w = std::min(crop_size_, input.cols - x);
  int h = std::min(crop_size_, input.rows - y);

  cv::Mat cropped = input(cv::Rect(x, y, w, h));
  cv::Mat resized;
  cv::resize(cropped, resized, cv::Size(output_size_, output_size_), 0, 0, cv::INTER_LINEAR);
  if (roi_downsample) {
    resized.copyTo(*roi_downsample);
  }
  cv::Mat working = resized;
  if (force_monochrome_) {
    cv::Mat gray_full;
    cv::cvtColor(working, gray_full, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray_full, working, cv::COLOR_GRAY2BGR);
  }

  if (!static_simplify_) {
    if (static_removed) {
      working.copyTo(*static_removed);
    }
    return working;
  }

  cv::Mat gray;
  cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty()) {
    gray.convertTo(background_gray_f32_, CV_32F);
    return working;
  }

  cv::Mat bg_u8;
  cv::convertScaleAbs(background_gray_f32_, bg_u8);

  cv::Mat diff;
  cv::absdiff(gray, bg_u8, diff);

  cv::Mat motion_mask;
  cv::threshold(diff, motion_mask, motion_threshold_, 255, cv::THRESH_BINARY);
  if (motion_erode_px_ > 0) {
    if (motion_erode_kernel_.empty()) {
      const int k = 2 * motion_erode_px_ + 1;
      motion_erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
  }
  if (motion_dilate_px_ > 0) {
    if (motion_dilate_kernel_.empty()) {
      const int k = 2 * motion_dilate_px_ + 1;
      motion_dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
  }
  const double motion_ratio_raw =
    static_cast<double>(cv::countNonZero(motion_mask)) / static_cast<double>(motion_mask.total());
  const bool suppress_trail = (motion_ratio_raw >= trail_disable_motion_ratio_);

  // 中心区域保护：不做静态模糊
  if (center_clear_size_ > 0) {
    const int clear_size = std::min({center_clear_size_, working.cols, working.rows});
    const int x0 = std::max(0, working.cols / 2 - clear_size / 2);
    const int y0 = std::max(0, working.rows / 2 - clear_size / 2);
    const int cw = std::min(clear_size, working.cols - x0);
    const int ch = std::min(clear_size, working.rows - y0);
    cv::rectangle(motion_mask, cv::Rect(x0, y0, cw, ch), cv::Scalar(255), cv::FILLED);
  }

  cv::Mat static_base = working.clone();
  if (!force_monochrome_ && target_bitrate_ <= 80) {
    cv::Mat gray_bg;
    cv::cvtColor(static_base, gray_bg, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray_bg, static_base, cv::COLOR_GRAY2BGR);
  }
  cv::Mat blurred_static;
  cv::GaussianBlur(
    static_base, blurred_static, cv::Size(), std::max(0.0, bg_blur_sigma_),
    std::max(0.0, bg_blur_sigma_));

  cv::Mat focused = blurred_static.clone();
  working.copyTo(focused, motion_mask);
  if (static_removed) {
    focused.copyTo(*static_removed);
  }

  // 运动拖影：简单时域 max（当前+历史N帧），仅作用在运动区域联合掩码
  if (motion_trail_frames_ > 0) {
    motion_mask_history_.push_back(motion_mask.clone());
    trail_frame_history_.push_back(working.clone());
    const size_t max_history = static_cast<size_t>(motion_trail_frames_ + 1);
    while (motion_mask_history_.size() > max_history) {
      motion_mask_history_.pop_front();
    }
    while (trail_frame_history_.size() > max_history) {
      trail_frame_history_.pop_front();
    }

    const size_t history_size = motion_mask_history_.size();
    if (!suppress_trail && history_size > 1 && history_size == trail_frame_history_.size()) {
      cv::Mat trail_mask = motion_mask.clone();
      cv::Mat trail_img = working.clone();
      for (size_t i = 0; i < history_size - 1; ++i) {
        cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
        cv::max(trail_img, trail_frame_history_[i], trail_img);
      }
      trail_img.copyTo(focused, trail_mask);
    }
  } else {
    motion_mask_history_.clear();
    trail_frame_history_.clear();
  }

  cv::accumulateWeighted(gray, background_gray_f32_, std::clamp(bg_update_alpha_, 0.001, 0.2));
  return focused;
}

void DoorlockEncoder::push_frame_to_gstreamer(const cv::Mat & frame)
{
  if (!appsrc_ || frame.empty()) return;

  size_t size = frame.total() * frame.elemSize();
  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    memcpy(map.data, frame.data, size);
    gst_buffer_unmap(buffer, &map);

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    if (ret != GST_FLOW_OK) {
      // 注意：fmt >= 11 不再自动格式化枚举，需显式转整型
      tools::logger()->warn("[doorlock] Push buffer failed: {}", static_cast<int>(ret));
    }
  }
  gst_buffer_unref(buffer);
}

void DoorlockEncoder::pull_stream_and_emit()
{
  if (!appsink_) return;

  FrameHandler handler;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handler = frame_handler_;
  }

  while (true) {
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 0);
    if (!sample) break;

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      gst_sample_unref(sample);
      continue;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      const int64_t ts = now_ns();
      if (handler) handler(encoded_frame_id_++, ts, map.data, map.size);
      emitted_bytes_ += map.size;

      // 1Hz 遥测
      if (ts - last_telemetry_ns_ > 1000000000LL) {
        tools::logger()->info(
          "[doorlock] encode stats: frames={} bytes={}B avg={:.2f}kB/s", frame_count_,
          emitted_bytes_, static_cast<double>(emitted_bytes_) / 1000.0 /
                             (static_cast<double>(ts - last_telemetry_ns_) / 1e9 + 1e-9));
        last_telemetry_ns_ = ts;
        emitted_bytes_ = 0;
      }

      gst_buffer_unmap(buffer, &map);
    }
    gst_sample_unref(sample);
  }
}

void DoorlockEncoder::display_loop()
{
  cv::namedWindow("Doorlock Sniper Raw", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper ROI", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper Static", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper", cv::WINDOW_NORMAL);
  cv::setWindowProperty("Doorlock Sniper Raw", cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
  cv::resizeWindow("Doorlock Sniper ROI", output_size_, output_size_);
  cv::resizeWindow("Doorlock Sniper Static", output_size_, output_size_);
  cv::resizeWindow("Doorlock Sniper", output_size_, output_size_);

  while (display_running_ && running_) {
    cv::Mat raw_frame;
    cv::Mat roi_frame;
    cv::Mat static_frame;
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!display_raw_frame_.empty()) display_raw_frame_.copyTo(raw_frame);
      if (!display_roi_frame_.empty()) display_roi_frame_.copyTo(roi_frame);
      if (!display_static_frame_.empty()) display_static_frame_.copyTo(static_frame);
      if (!display_frame_.empty()) display_frame_.copyTo(frame);
    }

    if (!raw_frame.empty()) cv::imshow("Doorlock Sniper Raw", raw_frame);
    if (!roi_frame.empty()) cv::imshow("Doorlock Sniper ROI", roi_frame);
    if (!static_frame.empty()) cv::imshow("Doorlock Sniper Static", static_frame);
    if (!frame.empty()) cv::imshow("Doorlock Sniper", frame);

    if (debug_dump_enable_ && !frame.empty()) {
      display_frame_counter_++;
      if ((display_frame_counter_ % static_cast<uint64_t>(debug_dump_every_n_frames_)) == 0U) {
        const std::filesystem::path dump_dir = std::filesystem::path(debug_dump_dir_) / "encoder";
        std::ostringstream idx;
        idx << std::setw(8) << std::setfill('0') << display_frame_counter_;
        const std::string frame_id = idx.str();
        if (debug_dump_save_raw_ && !raw_frame.empty()) {
          cv::imwrite((dump_dir / ("raw_" + frame_id + ".png")).string(), raw_frame);
        }
        if (debug_dump_save_roi_ && !roi_frame.empty()) {
          cv::imwrite((dump_dir / ("roi_" + frame_id + ".png")).string(), roi_frame);
        }
        if (debug_dump_save_static_ && !static_frame.empty()) {
          cv::imwrite((dump_dir / ("static_" + frame_id + ".png")).string(), static_frame);
        }
        if (debug_dump_save_final_) {
          cv::imwrite((dump_dir / ("final_" + frame_id + ".png")).string(), frame);
        }
      }
    }
    cv::waitKey(1);

    std::this_thread::sleep_for(16ms);
  }

  cv::destroyWindow("Doorlock Sniper Raw");
  cv::destroyWindow("Doorlock Sniper ROI");
  cv::destroyWindow("Doorlock Sniper Static");
  cv::destroyWindow("Doorlock Sniper");
}

}  // namespace doorlock
