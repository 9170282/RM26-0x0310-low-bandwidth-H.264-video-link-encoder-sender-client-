#include "tasks/doorlock/referee_serial_sender.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace doorlock
{
namespace
{
// 单帧 0x0310 协议开销：帧头 5B + 命令码 2B + CRC16 帧尾 2B
constexpr size_t kFrameOverhead = 9;
// 0x0310 数据段长度上限
constexpr int kMaxPayloadSize = 300;

int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}
}  // namespace

RefereeSerialSender::RefereeSerialSender(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto d = yaml["doorlock"];

  port_ = tools::read<std::string>(d, "serial_port", port_);
  baud_ = tools::read<uint32_t>(d, "serial_baud", baud_);
  payload_size_ = static_cast<size_t>(std::clamp(tools::read<int>(d, "payload_size", static_cast<int>(payload_size_)), 1, kMaxPayloadSize));
  cmd_id_ = static_cast<uint16_t>(tools::read<int>(d, "cmd_id", cmd_id_));
  bandwidth_limit_kbytes_ = tools::read<double>(d, "bandwidth_limit_kbytes", bandwidth_limit_kbytes_);
  bandwidth_window_s_ = tools::read<double>(d, "bandwidth_window_s", bandwidth_window_s_);
  if (bandwidth_window_s_ < 0.2) bandwidth_window_s_ = 0.2;
  // 官方协议 0x0310 频率上限 50Hz
  max_frame_rate_ = std::clamp(tools::read<int>(d, "max_frame_rate", max_frame_rate_), 1, 50);
  tokens_ = static_cast<double>(max_frame_rate_);  // 初始满桶，允许短暂突发

  // 未显式配置限速时按串口波特率（8N1: 每字节 10 bit），防串口缓冲溢出
  if (bandwidth_limit_kbytes_ <= 0) {
    bandwidth_limit_kbytes_ = static_cast<double>(baud_) / 10.0 / 1000.0 * 0.9;
  }

  open_serial();

  tools::logger()->info(
    "[doorlock] Referee serial sender ready: {} @{}baud payload={}B cmd=0x{:04X} "
    "limit={:.2f}kB/s@{}s max_rate={}Hz",
    port_, baud_, payload_size_, cmd_id_, bandwidth_limit_kbytes_, bandwidth_window_s_,
    max_frame_rate_);
}

RefereeSerialSender::~RefereeSerialSender()
{
  if (serial_.isOpen()) serial_.close();
}

void RefereeSerialSender::open_serial()
{
  try {
    // 重连前先干净关闭: 否则 setPort 内部"已打开就重开"会按旧配置重开,
    if (serial_.isOpen()) {
      serial_.close();
    }
    serial_.setPort(port_);
    serial_.setBaudrate(baud_);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    serial_ok_ = true;
    tools::logger()->info("[doorlock] Serial port {} @{} baud opened", port_, baud_);
  } catch (const std::exception & e) {
    serial_ok_ = false;
    tools::logger()->error("[doorlock] Failed to open serial {} @{}: {}", port_, baud_, e.what());
  }
}

void RefereeSerialSender::write_0310_frame(
  uint16_t cmd_id, uint8_t seq, const uint8_t * payload, size_t size)
{
  // 帧结构：SOF(0xA5) + data_len(2B 小端) + seq + CRC8(对前 4B)
  //          + cmd_id(2B 小端) + payload + CRC16(2B 小端，对整帧不含帧尾)
  std::vector<uint8_t> frame(kFrameOverhead + size);

  frame[0] = 0xA5;
  frame[1] = static_cast<uint8_t>(size & 0xFF);
  frame[2] = static_cast<uint8_t>((size >> 8) & 0xFF);
  frame[3] = seq;
  frame[4] = tools::get_crc8(frame.data(), 4);
  frame[5] = static_cast<uint8_t>(cmd_id & 0xFF);
  frame[6] = static_cast<uint8_t>((cmd_id >> 8) & 0xFF);
  if (size > 0) memcpy(frame.data() + 7, payload, size);

  const uint16_t crc16 = tools::get_crc16(frame.data(), 7 + size);
  frame[7 + size] = static_cast<uint8_t>(crc16 & 0xFF);
  frame[8 + size] = static_cast<uint8_t>((crc16 >> 8) & 0xFF);

  serial_.write(frame);
}

void RefereeSerialSender::send_frame(uint64_t frame_id, const uint8_t * data, size_t size)
{
  (void)frame_id;  // 裸字节流, 无帧语义(客户端按 Annex-B 起始码自行重组 AU)
  if (size == 0) return;

  // 串口未打开时定时重连
  if (!serial_ok_) {
    const int64_t ts = now_ns();
    if (ts - last_open_attempt_ns_ > 2000000000LL) {
      last_open_attempt_ns_ = ts;
      open_serial();
    }
    if (!serial_ok_) return;
  }

  const int64_t ts = now_ns();
  const size_t n_chunks = (size + payload_size_ - 1) / payload_size_;

  try {
    // 相邻两个 0x0310 分片严格拉开 ~21ms(50Hz + 5% 余量):
    // 图传链路/服务器按 50Hz 限流, 任何突发分片都会被丢。
    // 间隔跨帧/跨 AU 连续, 全程匀速, 不产生帧边界突发。
    const int64_t chunk_interval_ns =
      static_cast<int64_t>(1e9 * 1.05 / static_cast<double>(max_frame_rate_));
    for (size_t i = 0; i < n_chunks; ++i) {
      if (last_chunk_sent_ns_ != 0) {
        const int64_t wait_ns = last_chunk_sent_ns_ + chunk_interval_ns - now_ns();
        if (wait_ns > 0) {
          std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns));
        }
      }
      last_chunk_sent_ns_ = now_ns();
      const size_t offset = i * payload_size_;
      const size_t len = std::min(payload_size_, size - offset);
      // data 段 = 裸 H.264 Annex-B 字节切片(无自定义包头)
      write_0310_frame(cmd_id_, seq_++, data + offset, len);
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[doorlock] serial write failed: {}", e.what());
    serial_ok_ = false;  // 下一帧触发重连
    return;
  }

  sent_0310_frames_ += n_chunks;
  telemetry_bytes_ += size + n_chunks * kFrameOverhead;

  // 1Hz 遥测
  if (ts - last_telemetry_ns_ > 1000000000LL) {
    tools::logger()->info(
      "[doorlock] TX stats: {:.2f}kB/s frames_0310={}",
      static_cast<double>(telemetry_bytes_) / 1000.0 /
        (static_cast<double>(ts - last_telemetry_ns_) / 1e9),
      sent_0310_frames_);
    last_telemetry_ns_ = ts;
    telemetry_bytes_ = 0;
  }
}

}  // namespace doorlock
