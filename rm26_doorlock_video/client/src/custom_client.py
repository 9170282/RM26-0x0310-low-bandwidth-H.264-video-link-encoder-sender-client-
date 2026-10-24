#!/usr/bin/env python3
"""
RM2026 自定义客户端 (H.264 / 0x0310 CustomByteBlock)

运算端自定义相机画面
  python3 src/custom_client.py --player-id 1 --mqtt-host 192.168.12.1 --debug-dump dump.bin
"""

import argparse
import queue
import sys 
import threading
import time
from pathlib import Path

import av
from av.codec.context import Flags
import cv2

# --- Protobuf ---
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rm26_protocol"))
import rm26_custom_client_pb2 as pb

# --- MQTT ---
try:
    import paho.mqtt.client as mqtt
    HAS_MQTT = True
except ImportError:
    HAS_MQTT = False
    print("[ERROR] paho-mqtt not installed. Run: pip install paho-mqtt")
    sys.exit(1)



# MQTT topic -> Protobuf 消息类映射
DOWNSTREAM_TOPICS = {
    "GameStatus":                   pb.GameStatus,
    "GlobalUnitStatus":             pb.GlobalUnitStatus,
    "RobotStaticStatus":            pb.RobotStaticStatus,
    "RobotDynamicStatus":           pb.RobotDynamicStatus,
    "RobotPosition":                pb.RobotPosition,
    "DeployModeStatusSync":         pb.DeployModeStatusSync,
    "RobotRespawnStatus":           pb.RobotRespawnStatus,
    "Buff":                         pb.Buff,
    "PenaltyInfo":                  pb.PenaltyInfo,
    "Event":                        pb.Event,
    "CustomByteBlock":              pb.CustomByteBlock,
}



# GameState — 线程安全状态容器
class GameState:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {}

    def update(self, topic: str, message):
        with self._lock:
            self._data[topic] = message

    def get(self, topic: str):
        with self._lock:
            return self._data.get(topic)


class H264Reassembler:
    MAX_BUF = 2 * 1024 * 1024
    MAX_FRAME_SEARCH = 512 * 1024

    NAL_NONIDR = 1
    NAL_IDR = 5
    NAL_SEI = 6
    NAL_SPS = 7
    NAL_PPS = 8
    NAL_AUD = 9

    def __init__(self):
        self.buf = bytearray()
        self.pending_au = bytearray()
        self.pending_has_vcl = False
        self.pending_has_idr = False
        self.pending_has_sps = False
        self.pending_has_pps = False
        self.cached_sps = None
        self.cached_pps = None
        self.synced = False
        self._out = []

        self.nal_total = 0
        self.nal_nonidr = 0
        self.nal_idr = 0
        self.nal_sps = 0
        self.nal_pps = 0
        self.nal_aud = 0
        self.nal_sei = 0
        self.nal_other = 0
        self.no_start_code = 0
        self.frames_out = 0
        self.drops_out = 0

    def reset(self):
        self.buf = bytearray()
        self.pending_au = bytearray()
        self.pending_has_vcl = False
        self.pending_has_idr = False
        self.pending_has_sps = False
        self.pending_has_pps = False
        self.cached_sps = None
        self.cached_pps = None
        self.synced = False

    def feed(self, chunk):
        """输入一段裸字节, 返回本批已完成(可解码)的 AU 列表。"""
        if chunk:
            if len(self.buf) + len(chunk) > self.MAX_BUF:
                self.drops_out += 1
                self.reset()
            self.buf.extend(chunk)
            self._process()
        out = self._out
        self._out = []
        return out

    # -- 起始码扫描 ------------------------------------------------
    @staticmethod
    def _find_start_code(buf, offset=0):
        i = offset
        n = len(buf)
        while i + 2 < n:
            if buf[i] != 0 or buf[i + 1] != 0:
                i += 1
                continue
            if buf[i + 2] == 1:
                return i
            if i + 3 < n and buf[i + 2] == 0 and buf[i + 3] == 1:
                return i
            i += 1
        return -1

    @staticmethod
    def _start_code_len(buf, pos):
        if pos + 3 < len(buf) and buf[pos + 2] == 0 and buf[pos + 3] == 1:
            return 4
        return 3

    @staticmethod
    def _nal_type(nal):
        pos = H264Reassembler._find_start_code(nal)
        if pos < 0:
            return -1
        idx = pos + H264Reassembler._start_code_len(nal, pos)
        if idx >= len(nal):
            return -1
        return nal[idx] & 0x1F

    # -- NAL 提取 --------------------------------------------------
    def _extract_next_nal(self):
        if len(self.buf) < 5:
            return None
        first = self._find_start_code(self.buf)
        if first < 0:
            self.drops_out += 1
            self.no_start_code += 1
            self.buf = bytearray()
            return None
        if first > 0:
            del self.buf[:first]
        second = self._find_start_code(self.buf, self._start_code_len(self.buf, 0))
        if second < 0:
            if len(self.buf) > self.MAX_FRAME_SEARCH:
                self.drops_out += 1
                self.buf = bytearray()
            return None
        nal = bytes(self.buf[:second])
        del self.buf[:second]
        return nal

    # -- AU 状态机 -------------------------------------------------
    def _process(self):
        while True:
            nal = self._extract_next_nal()
            if nal is None:
                return
            self._process_nal(nal)

    def _process_nal(self, nal):
        t = self._nal_type(nal)
        if t < 0:
            return

        self.nal_total += 1
        if t == self.NAL_NONIDR:
            self.nal_nonidr += 1
        elif t == self.NAL_IDR:
            self.nal_idr += 1
        elif t == self.NAL_SPS:
            self.nal_sps += 1
        elif t == self.NAL_PPS:
            self.nal_pps += 1
        elif t == self.NAL_AUD:
            self.nal_aud += 1
        elif t == self.NAL_SEI:
            self.nal_sei += 1
        else:
            self.nal_other += 1

        if t == self.NAL_SPS:
            self.cached_sps = nal
        if t == self.NAL_PPS:
            self.cached_pps = nal

        is_vcl = t in (self.NAL_NONIDR, self.NAL_IDR)
        if is_vcl and self.pending_has_vcl:
            first_mb, ok = self._first_mb_in_slice(nal)
            if not ok or first_mb == 0:
                self._finish_pending()
        if not is_vcl and self.pending_has_vcl and t in (
            self.NAL_AUD, self.NAL_SEI, self.NAL_SPS, self.NAL_PPS):
            self._finish_pending()

        if is_vcl and not self.pending_au and t == self.NAL_IDR:
            if self.cached_sps:
                self._append_nal(self.cached_sps, self.NAL_SPS)
            if self.cached_pps:
                self._append_nal(self.cached_pps, self.NAL_PPS)

        self._append_nal(nal, t)

    def _append_nal(self, nal, t):
        self.pending_au.extend(nal)
        if t == self.NAL_SPS:
            self.pending_has_sps = True
        elif t == self.NAL_PPS:
            self.pending_has_pps = True
        elif t == self.NAL_IDR:
            self.pending_has_vcl = True
            self.pending_has_idr = True
        elif t == self.NAL_NONIDR:
            self.pending_has_vcl = True

    def _finish_pending(self):
        if not self.pending_au:
            return
        emit = False
        if self.pending_has_idr and (self.pending_has_sps or self.cached_sps) \
                and (self.pending_has_pps or self.cached_pps):
            emit = True
            self.synced = True
        elif self.synced and self.pending_has_vcl:
            emit = True
        if emit:
            self._out.append(bytes(self.pending_au))
            self.frames_out += 1
        elif self.pending_has_vcl:
            self.drops_out += 1
        self.pending_au = bytearray()
        self.pending_has_vcl = False
        self.pending_has_idr = False
        self.pending_has_sps = False
        self.pending_has_pps = False

    # -- slice header 里的 first_mb_in_slice (Exp-Golomb) ----------
    @staticmethod
    def _rbsp(payload):
        out = bytearray()
        zero = 0
        for v in payload:
            if zero >= 2 and v == 0x03:
                zero = 0
                continue
            out.append(v)
            zero = zero + 1 if v == 0 else 0
        return bytes(out)

    @staticmethod
    def _first_mb_in_slice(nal):
        pos = H264Reassembler._find_start_code(nal)
        if pos < 0:
            return 0, False
        idx = pos + H264Reassembler._start_code_len(nal, pos)
        if idx >= len(nal):
            return 0, False
        rbsp = H264Reassembler._rbsp(nal[idx + 1:])
        bit = 0
        n = len(rbsp)
        leading = 0
        while True:
            if bit >= n * 8:
                return 0, False
            b = (rbsp[bit // 8] >> (7 - (bit % 8))) & 1
            bit += 1
            if b == 1:
                break
            leading += 1
            if leading > 31:
                return 0, False
        if leading == 0:
            return 0, True
        suffix = 0
        for _ in range(leading):
            if bit >= n * 8:
                return 0, False
            b = (rbsp[bit // 8] >> (7 - (bit % 8))) & 1
            bit += 1
            suffix = (suffix << 1) | b
        return ((1 << leading) - 1 + suffix), True


# 0x0310 / CustomByteBlock 通道解码器: 裸 H.264 Annex-B 字节流
# → H264Reassembler 重组 AU → PyAV(FFmpeg) 解码
class H264StreamDecoder:
    def __init__(self, frame_queue: queue.Queue, log_stats: bool = False,
                 debug_dump_path: str = ""):
        self.frame_queue = frame_queue
        self.log_stats = log_stats
        self.chunk_count = 0
        self.frame_count = 0
        self.decode_error_count = 0
        self._last_stats_ts = time.monotonic()

        self.reassembler = H264Reassembler()
        self._debug_file = None
        if debug_dump_path:
            try:
                self._debug_file = open(debug_dump_path, "wb")
                print(f"[CustomVideo] dumping complete AUs to {debug_dump_path}")
            except OSError as e:
                print(f"[CustomVideo] cannot open dump file {debug_dump_path}: {e}")

        self.codec = av.CodecContext.create('h264', 'r')
        self.codec.thread_type = 'SLICE'  # slice 线程比 frame 线程少 1~2 帧排队延迟
        self.codec.flags2 |= 0x1          # AV_CODEC_FLAG2_FAST: 快速解码, 降低延迟
        try:
            self.codec.flags |= Flags.LOW_DELAY
        except AttributeError:
            try:
                self.codec.flags |= Flags.low_delay
            except AttributeError:
                self.codec.flags |= 8

    def _decode_au(self, au: bytes):
        for pkt in self.codec.parse(au):
            for fr in self.codec.decode(pkt):
                if fr is None or fr.width == 0:
                    continue
                img = fr.to_ndarray(format='bgr24')
                if img is None or img.size == 0:
                    continue
                self.frame_count += 1
                try:
                    self.frame_queue.put_nowait(img)
                except queue.Full:
                    pass

    def feed(self, chunk: bytes):
        self.chunk_count += 1
        now = time.monotonic()

        # 分片格式: 300B 裸 H.264 Annex-B 切片(无自定义包头)
        # 不做任何 reset: 重组器内部有 2MB 缓冲上限和起始码自同步,
        # 会在某个 IDR 完整到达时自然 sync, 损坏帧交给解码器错误隐藏。
        for au in self.reassembler.feed(chunk):
            if self._debug_file is not None:
                self._debug_file.write(au)
            try:
                self._decode_au(au)
            except av.AVError:
                self.decode_error_count += 1

        if self.log_stats and now - self._last_stats_ts >= 5.0:
            self._last_stats_ts = now
            r = self.reassembler
            print(f"[CustomVideo] chunks={self.chunk_count} frames={self.frame_count} "
                  f"errors={self.decode_error_count} "
                  f"AU_out={r.frames_out} drops={r.drops_out} "
                  f"nals(idr={r.nal_idr},sps={r.nal_sps},pps={r.nal_pps},"
                  f"nonidr={r.nal_nonidr},aud={r.nal_aud}) synced={r.synced}")


# MQTT 客户端
class MqttGameClient:
    def __init__(self, host: str, port: int, player_id: str, state: GameState,
                 custom_decoder=None):
        self.host = host
        self.port = port
        self.player_id = player_id
        self.state = state
        self.custom_decoder = custom_decoder  # 非空时 CustomByteBlock 喂入视频解码
        self._connected = threading.Event()

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=player_id,
            protocol=mqtt.MQTTv311,
        )
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

    def _on_connect(self, client, userdata, connect_flags, reason_code, properties):
        if reason_code == 0 or reason_code == "Success":
            print(f"[MQTT] Connected to {self.host}:{self.port} as '{self.player_id}'")
            for topic in DOWNSTREAM_TOPICS:
                client.subscribe(topic, qos=1)
            print(f"[MQTT] Subscribed to {len(DOWNSTREAM_TOPICS)} topics")
            self._connected.set()
        else:
            print(f"[MQTT] Connection failed, reason={reason_code}")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"[MQTT] Disconnected (reason={reason_code}), auto-reconnect...")
        self._connected.clear()

    def _on_message(self, client, userdata, msg):
        # 运算端自定义视频 (机器人 0x0310 → 图传链路 → CustomByteBlock):
        # data 字段 = 裸 H.264 Annex-B 切片, H264StreamDecoder 负责重组解码
        if msg.topic == "CustomByteBlock" and self.custom_decoder is not None:
            try:
                m = pb.CustomByteBlock()
                m.ParseFromString(msg.payload)
                if m.data:
                    self.custom_decoder.feed(m.data)
            except Exception as e:
                print(f"[MQTT] CustomByteBlock parse error: {e}")
            return
        proto_cls = DOWNSTREAM_TOPICS.get(msg.topic)
        if proto_cls is None:
            return
        try:
            message = proto_cls()
            message.ParseFromString(msg.payload)
            self.state.update(msg.topic, message)
        except Exception as e:
            print(f"[MQTT] Parse error on {msg.topic}: {e}")

    def connect(self):
        self.client.connect_async(self.host, self.port, keepalive=10)
        self.client.loop_start()

    def wait_connected(self, timeout: float = 5.0) -> bool:
        return self._connected.wait(timeout)

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


class DisplayEngine:
    def __init__(self, frame_queue: queue.Queue, display_scale: int = 1):
        self.frame_queue = frame_queue
        self.display_scale = max(1, display_scale)
        self._frame_seen = 0

    def run(self):
        window_name = "RM26 Custom Client | Q=Quit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 2560, 1440)

        while True:
            try:
                img = self.frame_queue.get(timeout=0.05)
                if img is None or img.size == 0:
                    continue

                # 亮度自检: 前 5 帧打印明暗, 判断画面是黑还是真有内容
                self._frame_seen += 1
                if self._frame_seen <= 5:
                    print(f"[Display] frame#{self._frame_seen} shape={img.shape} "
                          f"mean={img.mean():.1f} min={img.min()} max={img.max()}")

                # 缩放显示
                h, w = img.shape[:2]
                dw, dh = int(w * self.display_scale), int(h * self.display_scale)
                img_disp = cv2.resize(img, (dw, dh), interpolation=cv2.INTER_NEAREST)

                cv2.imshow(window_name, img_disp)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[Display] Quit")
                    break

            except queue.Empty:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
            except Exception as e:
                print(f"[Display] Error: {e}")
                break

        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="RM2026 Custom Client (H.264)")
    parser.add_argument("--player-id", default="1", help="选手端 ClientId (1=红方英雄, 101=蓝方英雄)")
    parser.add_argument("--mqtt-host", default="192.168.12.1", help="MQTT Broker 地址 (默认: 192.168.12.1)")
    parser.add_argument("--mqtt-port", type=int, default=3333, help="MQTT Broker 端口 (默认: 3333)")
    parser.add_argument("--debug-dump", default="", help="把重组后的完整 H.264 帧写入该文件 (可 ffplay -f h264 回放)")
    parser.add_argument("--display-scale", type=float, default=4.0, help="显示缩放比例")

    args = parser.parse_args()
                                                                                                                    
    print("=" * 50)
    print("  RM2026 Custom Client (H.264)")
    print(f"  Player ID: {args.player_id}")
    print(f"  MQTT: {args.mqtt_host}:{args.mqtt_port}")
    print("  Video Source: mqtt CustomByteBlock (运算端 0x0310 自定义 H.264 视频)")
    print("=" * 50)

    state = GameState()
    frame_queue = queue.Queue(maxsize=3)

    # 0x0310 通道: 裸 H.264 Annex-B 字节流, 起始码重组后解码
    mqtt_decoder = H264StreamDecoder(frame_queue, log_stats=True,
                                     debug_dump_path=args.debug_dump)

    # --- MQTT ---
    mqtt_client = MqttGameClient(
        args.mqtt_host, args.mqtt_port, args.player_id, state,
        custom_decoder=mqtt_decoder,
    )
    mqtt_client.connect()
    if not mqtt_client.wait_connected(timeout=5.0):
        print("[WARN] MQTT not connected, continuing without game state...")

    print("[Client] 等待 CustomByteBlock 视频数据... (带宽上限 300B×50Hz=15kB/s)")

    # --- 显示 ---
    print("\n[Client] Display started. Press Q to quit.\n")
    display = DisplayEngine(frame_queue, display_scale=args.display_scale)
    try:
        display.run()
    except KeyboardInterrupt:
        print("\n[Client] Interrupted")

    # --- 清理 ---
    print("[Client] Shutting down...")
    mqtt_client.stop()
    print("[Client] Done.")


if __name__ == "__main__":
    main()
