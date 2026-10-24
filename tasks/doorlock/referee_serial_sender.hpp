#ifndef TASKS__DOORLOCK__REFEREE_SERIAL_SENDER_HPP_
#define TASKS__DOORLOCK__REFEREE_SERIAL_SENDER_HPP_

#include <serial/serial.h>

#include <cstdint>
#include <deque>
#include <string>
#include <utility>

namespace doorlock
{
// 裁判系统串口发送端（0x0310 自定义控制器交互数据）：
// 链路: NUC --串口--> C板 --> 图传模块 --无线--> 裁判系统服务器 --MQTT--> 自定义客户端
// 每个 H.264 编码帧按 payload_size 分块，每块封装成一帧裁判系统串口协议帧：
//   SOF(0xA5) + data_len(2B 小端) + seq(1B) + CRC8(1B, 对前 4B)
//   + cmd_id(2B 小端, 0x0310) + payload + CRC16(2B 小端, 对整帧不含帧尾)
// payload(data 段) = 裸 H.264 Annex-B 字节切片(无自定义包头)，
// 客户端按起始码重组 AU、缓存 SPS/PPS、等下一个 IDR 恢复。
class RefereeSerialSender
{
public:
  explicit RefereeSerialSender(const std::string & config_path);
  ~RefereeSerialSender();

  void send_frame(uint64_t frame_id, const uint8_t * data, size_t size);

private:
  void open_serial();
  void write_0310_frame(uint16_t cmd_id, uint8_t seq, const uint8_t * payload, size_t size);

  serial::Serial serial_;
  std::string port_;
  uint32_t baud_ = 921600;
  uint16_t cmd_id_ = 0x0310;
  size_t payload_size_ = 300;  // 0x0310 数据段长度上限 300 字节
  uint8_t seq_ = 0;            // 包序号，每发一帧 +1，0~255 回绕
  bool serial_ok_ = false;
  int64_t last_open_attempt_ns_ = 0;

  // 发送限速（0 = 按串口波特率自动取上限）：超预算时整帧丢弃（丢帧优于乱序切片）
  double bandwidth_limit_kbytes_ = 0.0;
  double bandwidth_window_s_ = 2.0;
  std::deque<std::pair<int64_t, size_t>> sent_window_;
  size_t sent_window_bytes_ = 0;

  // 0x0310 发送频率上限（官方协议 50Hz）：令牌桶，超限整帧丢弃
  int max_frame_rate_ = 50;
  double tokens_ = 0.0;
  int64_t last_refill_ns_ = 0;
  int64_t last_chunk_sent_ns_ = 0;  // 上一个 0x0310 分片写出时间(跨帧连续), 用于严格摊开 50Hz

  uint64_t dropped_frames_ = 0;
  uint64_t sent_0310_frames_ = 0;
  uint64_t telemetry_bytes_ = 0;
  int64_t last_telemetry_ns_ = 0;
};

}  // namespace doorlock

#endif  // TASKS__DOORLOCK__REFEREE_SERIAL_SENDER_HPP_
