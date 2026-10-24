# RM2026 Custom Client (H.264)

只看**运算端自定义相机画面**的选手端：小电脑把相机画面压成 H.264，走裁判系统
0x0310（图传链路）→ MQTT `CustomByteBlock` → 本客户端重组解码显示。

## 快速开始

```bash
# 安装依赖
pip install -r requirements.txt

# 看运算端自定义相机画面
python3 src/custom_client.py --player-id 1 --mqtt-host 192.168.12.1
```

## 链路与协议

- 官方机器人→自定义客户端数据通道：MQTT topic `CustomByteBlock`（机器人端对应 0x0310，
  300 字节/包、50Hz ≈15kB/s、无重传）。
- MQTT `clientID` 必须填所连接机器人对应的 ID 编号（`--player-id`，红方英雄=1）。
- 服务器（裁判系统）固定 `192.168.12.1:3333`，自定义客户端 IP 固定 `192.168.12.2`。



已与 `sp_vision_25-main（英雄）（daheng）` 的 `doorlock_encoder` + `referee_serial_sender` 对齐：

1. 链路：NUC --串口 921600--> C板/图传发送端 --无线--> 裁判系统服务器 --MQTT--> 本客户端
2. 每个 0x0310 帧：`SOF(0xA5)+data_len(2B小端)+seq+CRC8 + cmd_id(0x0310,2B小端) + data + CRC16`
3. **data 字段 = 300B 裸 H.264 Annex-B 字节切片（无自定义包头）**：
   - 客户端按起始码切 NAL → AUD/slice 判 AU 边界 → 缓存 SPS/PPS → 等到 IDR 才同步输出
     （等价于 Robomaster_Aurora_Client 的 `H264Reassembler`），损坏帧交给解码器错误隐藏。
4. 发送频率 ≤50Hz，发送端按 21ms/片 匀速摊开，不产生突发。
5. 编码固定（zerolatency: bframes=0 + lookahead=0），
   300×300@30fps、80kbps、x264 veryfast、aud、静态简化。

## 显示

- OpenCV 窗口 `RM26 Custom Client | Q=Quit`，按 `Q` 退出。
- `--display-scale N` 缩放显示（默认 2.0）。
- 需在**有图形界面**的环境运行（`echo $DISPLAY` 非空，且用的是 `opencv-python` 而非
  `opencv-python-headless`）。

## 诊断参数

| 参数 | 作用 |
|---|---|
| `--debug-dump dump.bin` | 把重组出的完整 H.264 帧落盘，可 `ffplay -f h264 dump.bin` 回放 |
| 每 5 秒统计行 | `frames/lost/AU_out/drops/nal(idr,sps,pps,nonidr,aud)/synced`，判断出帧率与丢包 |

## 架构

```
MQTT 线程:  订阅 CustomByteBlock → 300B 裸 H.264 字节流
            → H264Reassembler 重组 AU → PyAV(FFmpeg) H.264 解码 → frame_queue
显示线程:   frame_queue → OpenCV 显示 + 准星
```
