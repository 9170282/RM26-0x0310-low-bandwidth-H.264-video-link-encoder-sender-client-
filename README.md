# RM26 0x0310 低带宽图传（doorlock / 落点图传）

把机器人上的相机画面压成 H.264，经裁判系统 **0x0310 自定义数据通道**（图传链路，300B × 50Hz ≈ 15 kB/s）传到操作手电脑上的自定义客户端，再重组解码显示。

本目录只保留“编码 + 发送”这条链路，以及配套的解码客户端。

```
机器人（小电脑）                          操作手电脑
┌─────────────────────────┐
│ camera (HikRobot)       │
│   ↓                    │
│ 预处理: 裁剪/降采样/静态简化/拖影 │
│   ↓                    │
│ x264enc (zerolatency)   │
│   ↓ 裸 H.264 Annex-B    │
│ 0x0310 分包(300B)+21ms匀速 │
│   ↓ 串口 921600         │
└──────────┬──────────────┘
           │  C板 → 图传发送端(VT03) → 空口 → 图传接收端(VT13)
           ▼
   裁判系统服务器 → MQTT `CustomByteBlock`
           ▼
┌─────────────────────────┐
│ 自定义客户端 (Python)      │
│ H264Reassembler 重组 AU   │
│   ↓                      │
│ PyAV 解码 → OpenCV 显示   │
└─────────────────────────┘
```

## 目录结构

```
rm26_doorlock_video/
├── CMakeLists.txt
├── configs/doorlock.yaml          # 相机 + 编码 + 发送 配置
├── src/doorlock_sniper_usb.cpp    # 主程序
├── tasks/doorlock/                # 编码器 + 0x0310 发送器（核心）
│   ├── doorlock_encoder.hpp
│   ├── referee_serial_sender.hpp
│   └── src/
│       ├── doorlock_encoder.cpp
│       └── referee_serial_sender.cpp
├── io/                            # 相机抽象 + 海康相机 + 串口库
├── tools/                         # crc / logger / yaml / 线程安全队列
└── client/                        # Python 解码端
    ├── src/custom_client.py
    ├── rm26_protocol/rm26_custom_client_pb2.py
    └── proto/rm26_custom_client.proto
```

## 0x0310 帧格式

```
┌──────┬────────────┬──────┬──────┬──────────┬──────────────┬────────┐
│ SOF  │ data_len   │ seq  │ CRC8 │ cmd_id   │ data (≤300B) │ CRC16  │
│ 0xA5 │ 2B 小端     │ 1B   │ 1B   │ 0x0310   │ 裸 H.264      │ 2B     │
└──────┴────────────┴──────┴──────┴──────────┴──────────────┴────────┘
```

- `data` = 裸 H.264 Annex-B 字节流切片（**无自定义包头**）。
- 分片 300B，发送端按 **21ms/片** 匀速摊开（50Hz + 5% 余量），避免图传/服务器的 50Hz 限流丢帧。
- **无重传**：丢了就丢了，客户端靠起始码 + IDR 自愈。

## 编码端（C++）

| 项 | 值 |
|---|---|
| 管线 | `appsrc → videoconvert → x264enc → h264parse → appsink` |
| 编码 | H.264，`byte-stream`，`alignment=au` |
| 关键参数 | `tune=zerolatency`、`bframes=0`、`rc-lookahead=0`、`sync-lookahead=0`、`speed-preset=veryfast`、`key-int-max=2*output_fps`、`aud=TRUE` |
| 预处理 | 中心裁剪 → 降采样 → 静态简化（背景灰度+模糊）→ 运动拖影 |
| 输出 | 每个编码帧（AU）回调 `RefereeSerialSender::send_frame()` |

> 预处理器把“静止背景”压成低码率灰度/模糊，只让运动区域保持清晰，这是在 15kB/s 下能看清目标的关键。

## 解码端（Python）

- 订阅 MQTT `CustomByteBlock`（300B/包）。
- `H264Reassembler`：按起始码切 NAL → AUD/slice 判 AU 边界 → 缓存 SPS/PPS → 等到 IDR 才同步输出。
- PyAV 解码（`thread_type='SLICE'` + `LOW_DELAY` + `FAST`），OpenCV 显示。

## 构建

### 系统依赖（Ubuntu/Debian）

```bash
sudo apt install build-essential cmake pkg-config \
    libopencv-dev libyaml-cpp-dev libfmt-dev libspdlog-dev libusb-1.0-0-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly
```


### 海康 MVS SDK

编码用海康相机，需要安装 [MVS SDK](https://www.hikrobotics.com/cn/machinevision/service/download)（默认装到 `/opt/MVS`）。
如果不在默认路径：

```bash
cmake -DMVS_SDK_ROOT=/your/path/to/MVS ..
```

### 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

生成可执行文件 `doorlock_sniper_usb`。

## 运行

```bash
# 先改 configs/doorlock.yaml 里的 serial_port 为实际设备（如 /dev/ttyACM0）
./doorlock_sniper_usb ../configs/doorlock.yaml
```

编码端会把码流落盘到 `logs/doorlock_stream.bin`，可用 `ffplay -f h264 logs/doorlock_stream.bin` 回放验证。

## 关键配置（configs/doorlock.yaml）

| 键 | 说明 |
|---|---|
| `output_size` | 编码分辨率（如 300） |
| `output_fps` | 输出帧率（30） |
| `target_bitrate` | 目标码率 kbps（80，别超 ~115） |
| `x264_preset` | 速度档位（veryfast） |
| `force_monochrome` | 强制灰度（省 ~40-50% 数据，帧率翻倍） |
| `static_simplify` | 静态区域简化（背景模糊+灰度） |
| `motion_trail_frames` | 运动拖影帧数 |
| `serial_port` | 连接 C板/图传发送端的串口 |
| `payload_size` | 单帧数据段长度（300） |
| `max_frame_rate` | 发送频率上限（50） |
| `output_file` | 码流落盘路径 |

## 带宽与延迟的硬约束（重要）

- **带宽上限**：`300B × 50Hz = 15 kB/s`，实际建议 ≤ 12 kB/s。
- **无重传**：丢包只能靠客户端等下一个 IDR 恢复。
- **延迟**：链路本身 ~130–200ms（协议），加上编码/发送/重组，实际约 250–400ms（低延迟配置下）。

### 一个重要的经验：压缩率也是“抗丢包能力”

在丢包严重的链路上，**帧越大越容易整帧丢光**。而 `bframes=0 + zerolatency` 虽然延迟最低，但压缩率最差、帧最大，在丢包下最容易“永远组不出完整 IDR → 黑屏”。

如果链路丢包严重，建议回到高压缩配置（`bframes=4 + rc-lookahead=40 + veryslow`），或降低分辨率/开灰度把帧压小，而不是一味追求零延迟。

## 客户端

```bash
cd client
pip install -r requirements.txt
python3 src/custom_client.py --player-id 1 --mqtt-host 192.168.12.1
```

- `--player-id`：机器人 ID（如红方英雄=1）。
- `--debug-dump dump.bin`：把重组出的完整帧落盘，`ffplay -f h264 dump.bin` 回放。
- 需要 `DISPLAY` 可用且装的是 `opencv-python`（不是 `opencv-python-headless`）。

## 已知问题 / 后续方向

1. **链路丢包**：0x0310 无重传，高丢包下画面会花/缺帧，需要从串口/C板透传/图传信号排查。
2. **相机抖动**：拖影/背景分离依赖“背景不动”，手臂/后座抖动会造成误判，可加防抖配准（相位相关/单应）。
3. **前景/背景分离**：目前是“静态简化”，更激进的做法是背景低频传输 + 前景 ROI 高频传输 + 客户端拼合（更省带宽，但改动大）。
4. **FEC**：可加前向纠错，用 10–30% 冗余换丢包恢复能力。

## 致谢 / 来源

- 相机抽象、串口库、tools 来自 `sp_vision_25` 工程。
- H.264 重组器思路参考 `Robomaster_Aurora_Client` 的 `H264Reassembler`。
- 编码/静态简化思路参考 `Pacific_doorlock_sniper`（落点图传）。

## License

代码仅供 RoboMaster 参赛队学习参考；使用前请自行确认依赖库（OpenCV / GStreamer / MVS SDK 等）的许可证。
