// 低带宽落点图传：相机 → 准星区域裁剪放大 → 静态简化 → H.264 压缩
//               → 0x0310 串口帧(裸 Annex-B 字节流) → C板 → 图传 → 裁判系统服务器 → MQTT → 客户端


#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/doorlock/doorlock_encoder.hpp"
#include "tasks/doorlock/referee_serial_sender.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/doorlock.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  // 声明顺序很重要: sender 与 stream_file 必须"先声明"(即比 encoder 晚析构),
  // 因为 encoder 的 worker 线程在 encoder 析构前仍会调用 send_frame / 写 stream_file。
  io::Camera camera(config_path);
  doorlock::RefereeSerialSender sender(config_path);

  //码流落盘（每个 H.264 编码帧顺序写入，可用 ffplay -f h264 回放验证）
  auto yaml = tools::load(config_path);
  auto output_file =
    tools::read<std::string>(yaml["doorlock"], "output_file", std::string("logs/doorlock_stream.bin"));
  std::ofstream stream_file;
  if (!output_file.empty()) {
    stream_file.open(output_file, std::ios::binary | std::ios::trunc);
    if (!stream_file.is_open()) {
      tools::logger()->warn("[doorlock] Failed to open output file: {}", output_file);
    }
  }

  doorlock::DoorlockEncoder encoder(config_path);

  //回调: 每个 H.264 编码帧回调一次，frame_id 递增分配，ts_ns 为编码时刻
  encoder.set_frame_handler(
    [&](uint64_t frame_id, int64_t ts_ns, const uint8_t * data, size_t size) {
      sender.send_frame(frame_id, data, size);
      if (stream_file.is_open()) {
        stream_file.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
      }
    });

  tools::logger()->info("[doorlock] Main loop started. Ctrl+C to quit.");

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    encoder.encode(
      img, std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
  }

  return 0;
}
