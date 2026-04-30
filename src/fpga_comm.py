"""
fpga_comm.py — Giao tiếp UART với DE10 + so sánh golden model
==============================================================
Cách dùng:
  1. Chạy gen_test.py trước để sinh x_data_sim.txt và các file .mif
  2. Compile + đổ bitstream lên DE10 bằng Quartus
  3. Chạy: python fpga_comm.py
     Hoặc chỉ định cổng: python fpga_comm.py --port COM5
     Hoặc chỉ tính golden model (không cần kit): python fpga_comm.py --dry-run

Giao thức UART với FPGA (8N1, 115200 baud):
  PC → 0xAA          : lệnh bắt đầu nhận dữ liệu
  PC → 384 byte X    : 192 word Q8.8, big-endian (high byte trước)
  PC → 0x55          : lệnh đọc kết quả
  FPGA → 18 byte Y   : 9 word Q8.8, big-endian (attention weights)
"""

import argparse
import struct
import sys
import time
import os

import numpy as np

# ── Cấu hình cổng UART (thay đổi cho phù hợp với máy) ──────────────────────
DEFAULT_PORT     = "COM6"          # Windows: COM3, COM4, ...
                                   # Linux:   /dev/ttyUSB0
                                   # macOS:   /dev/tty.usbserial-*
BAUD_RATE        = 115_200
PIPELINE_TIMEOUT = 10              # Giây chờ pipeline FPGA tính xong

# ── Tham số phải khớp với gen_test.py ───────────────────────────────────────
TOKENS     = 3
D_MODEL    = 64
FRAC_BITS  = 8
SEED       = 42
DATA_FILE  = "x_data_sim.txt"     # Sinh bởi gen_test.py

CMD_START  = 0xAA
CMD_READ   = 0x55


# ============================================================================
# PHẦN 1: Tính golden model (tái tạo chính xác logic gen_test.py)
# ============================================================================

def float_to_q88(val_array):
    q = np.round(val_array * (1 << FRAC_BITS))
    return np.clip(q, -32768, 32767).astype(np.int64)


def compute_golden():
    rng = np.random.RandomState(SEED)

    # Đã sửa range thành -0.35 đến 0.35
    X_fl  = rng.uniform(-0.35, 0.35, (TOKENS, D_MODEL)).astype(np.float32)
    Wq_fl = rng.uniform(-0.35, 0.35, (D_MODEL, D_MODEL)).astype(np.float32)
    Wk_fl = rng.uniform(-0.35, 0.35, (D_MODEL, D_MODEL)).astype(np.float32)
    Wv_fl = rng.uniform(-0.35, 0.35, (D_MODEL, D_MODEL)).astype(np.float32)

    X_int, Wq_int, Wk_int, Wv_int = map(float_to_q88, [X_fl, Wq_fl, Wk_fl, Wv_fl])

    # Phase 1
    Q_int = np.floor(np.dot(X_int, Wq_int.T) / 256.0).astype(np.int64)
    K_int = np.floor(np.dot(X_int, Wk_int.T) / 256.0).astype(np.int64)

    # Phase 2
    mac_sum = np.dot(Q_int, K_int.T)
    Score_int = np.floor(mac_sum / 2048.0).astype(np.int64) + ((mac_sum >> 10) & 1)

    # Phase 3
    max_score = np.max(Score_int, axis=1, keepdims=True)
    Z_int = Score_int - max_score

    # Exp ROM nội suy với độ sâu 2048
    exp_Z_int = np.zeros_like(Z_int)
    for i in range(Z_int.shape[0]):
        for j in range(Z_int.shape[1]):
            val = Z_int[i, j]
            if val <= 0:
                addr = (-val) & 0x7FF
                if addr < 2048:
                    x = -addr / 256.0
                    q_val = int(np.round(np.exp(x) * 256.0))
                    if q_val == 0 and np.exp(x) > 0: q_val = 1
                    exp_Z_int[i, j] = q_val
                    
    sum_exp_int = np.sum(exp_Z_int, axis=1, keepdims=True)

    # Phase 4
    weights_int = (exp_Z_int * 256) // sum_exp_int

    return {
        "X":           X_int,
        "Q_q88":       Q_int,
        "K_q88":       K_int,
        "Score_q88":   Score_int,
        "exp_Z_q88":   exp_Z_int,
        "sum_exp":     sum_exp_int,
        "weights_q88": weights_int,
        "weights_flat": weights_int.flatten(),
    }


def load_x_from_file(path=DATA_FILE):
    """
    Đọc x_data_sim.txt (mỗi dòng = 1 word hex 4 ký tự).
    Dùng file này để gửi lên FPGA — đảm bảo khớp với MIF đã nạp vào weight ROM.
    """
    if not os.path.exists(path):
        print(f"[WARN] Không tìm thấy {path}. Dùng golden model tính lại X.")
        gold = compute_golden()
        return gold["X"].flatten()
    with open(path) as f:
        words = []
        for line in f:
            line = line.strip()
            if line:
                val = int(line, 16)
                # Chuyển về signed 16-bit
                if val >= 0x8000:
                    val -= 0x10000
                words.append(val)
    return np.array(words, dtype=np.int16)


# ============================================================================
# PHẦN 2: Giao tiếp UART với FPGA
# ============================================================================

def open_serial(port, baud=BAUD_RATE):
    try:
        import serial
    except ImportError:
        print("[LỖI] Chưa cài pyserial. Chạy: pip install pyserial")
        sys.exit(1)
    try:
        ser = serial.Serial(port, baud, timeout=PIPELINE_TIMEOUT)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.2)
        ser.reset_input_buffer()
        print(f"[OK] Đã mở cổng {port} @ {baud} baud")
        return ser
    except Exception as e:
        print(f"[LỖI] Không mở được {port}: {e}")
        print("      Kiểm tra: kit đã cắm USB? Đúng cổng COM? Driver FT232R đã cài?")
        sys.exit(1)


def send_x_data(ser, x_words):
    """
    Gửi lệnh 0xAA rồi 384 byte dữ liệu X (big-endian, signed).

    Timing thực tế @ 115200 baud, 8N1:
      - 1 byte  = 10 bit-time = 86.8 µs
      - 385 byte (0xAA + 384 data) = ~33.4 ms để truyền hết
      - Pipeline FPGA (3 token linear + score + softmax) ≈ 0.25 ms @ 50 MHz
      → Pipeline xong từ lâu TRƯỚC khi byte cuối đến FPGA.
      → Không cần sleep sau khi gửi xong, gửi 0x55 ngay lập tức là ổn.

    Lý do KHÔNG chunk nhỏ:
      - PC có TX buffer (thường 4096 byte), pyserial write() non-blocking.
      - FPGA uart_rx xử lý từng byte one-by-one, không bị overflow nếu
        baud rate khớp — tốc độ gửi từ PC đúng bằng 115200 baud.
      - Chunk + sleep chỉ làm chậm không cần thiết.
    """
    print("\n[UART] Gửi lệnh CMD_START (0xAA)...")
    ser.write(bytes([CMD_START]))

    print(f"[UART] Gửi {TOKENS * D_MODEL} word ({TOKENS * D_MODEL * 2} byte) dữ liệu X...")
    payload = b""
    for w in x_words[:TOKENS * D_MODEL]:
        payload += struct.pack(">h", int(w))  # signed big-endian

    ser.write(payload)
    ser.flush()  # đảm bảo OS đẩy hết byte ra COM port ngay bây giờ

    # Tính thời gian lý thuyết để truyền xong (dùng để log, không dùng để sleep)
    total_bytes    = 1 + len(payload)          # 0xAA + 384 byte data
    uart_time_ms   = total_bytes * 10 / BAUD_RATE * 1000
    print(f"[UART] Đã gửi. Thời gian UART lý thuyết: {uart_time_ms:.1f} ms")
    print(f"[UART] Chờ FPGA truyền xong và pipeline tính ({PIPELINE_TIMEOUT}s timeout)...")


def sync_controller(ser):
    """
    Đồng bộ hóa uart_controller về WAIT_CMD trước khi gửi data mới.
    Nếu controller đang kẹt ở RECV_DATA (chờ thêm byte), gửi đủ 384 byte
    0x00 để nó chuyển sang RUN_LINEAR rồi DONE, sau đó sẵn sàng nhận 0xAA mới.
    Nếu controller đang ở DONE, gửi 0x55 để nó về WAIT_CMD.
    """
    print("[SYNC] Đồng bộ hóa controller...")
    ser.reset_input_buffer()
    # Gửi 384 byte 0x00 — nếu controller đang giữa RECV_DATA sẽ fill nốt
    # Nếu đang WAIT_CMD thì 0x00 bị ignore (không phải 0xAA hay 0x55)
    ser.write(bytes(385))   # 1 byte header giả + 384 byte data giả
    time.sleep(1.5)         # chờ pipeline chạy xong (nếu bị trigger)
    ser.reset_input_buffer()
    # Gửi 0x55 để drain kết quả nếu có
    ser.write(bytes([CMD_READ]))
    time.sleep(0.1)
    ser.reset_input_buffer()
    print("[SYNC] Xong — controller đã về WAIT_CMD.")


def read_results(ser):
    """Gửi lệnh 0x55, nhận 18 byte kết quả."""
    # Chờ pipeline chắc chắn xong trước khi hỏi.
    # Pipeline FPGA mất ~0.25ms, nhưng UART TX còn đang truyền nốt dữ liệu
    # 384 byte → thêm buffer 200ms là dư để mọi byte đến FPGA và xử lý xong.
    time.sleep(0.2)

    print("[UART] Gửi lệnh CMD_READ (0x55)...")
    ser.write(bytes([CMD_READ]))
    time.sleep(0.05)  # chờ controller chuyển sang SEND_RESULT

    result = ser.read(18)
    if len(result) < 18:
        print(f"[LỖI] Chỉ nhận được {len(result)}/18 byte. Timeout hoặc pipeline chưa xong.")
        print("       Thử tăng PIPELINE_TIMEOUT hoặc kiểm tra lại FPGA.")
        return None

    words = []
    for i in range(9):
        w = struct.unpack(">h", result[i*2 : i*2+2])[0]
        words.append(w)
    return words


# ============================================================================
# PHẦN 3: Hiển thị và so sánh kết quả
# ============================================================================

def print_golden_summary(gold):
    """In bảng tóm tắt golden model (giống gen_test.py nhưng gọn hơn)."""
    print("\n" + "="*62)
    print("  GOLDEN MODEL (PYTHON) — Tham chiếu cho FPGA")
    print("="*62)

    print("\n[Phase 1] Q, K — 4 phần tử đầu mỗi token:")
    for t in range(TOKENS):
        q_vals = " | ".join(f"{int(gold['Q_q88'][t,i]) & 0xFFFF:04x}({int(gold['Q_q88'][t,i]):4})"
                            for i in range(4))
        k_vals = " | ".join(f"{int(gold['K_q88'][t,i]) & 0xFFFF:04x}({int(gold['K_q88'][t,i]):4})"
                            for i in range(4))
        print(f"  Token {t}  Q: {q_vals}")
        print(f"           K: {k_vals}")

    print("\n[Phase 2] Attention Score (Q·Kᵀ / √64):")
    for i, s in enumerate(gold["Score_q88"].flatten()):
        print(f"  S[{i//3}][{i%3}] = {int(s) & 0xFFFF:04x}  ({int(s):5})  = {int(s)/256:.5f}")

    print("\n[Phase 3] Exp(Z) và tổng mỗi hàng:")
    for i, e in enumerate(gold["exp_Z_q88"].flatten()):
        print(f"  exp[{i//3}][{i%3}] = {int(e) & 0xFFFF:04x}  ({int(e):5})  = {int(e)/256:.5f}")
    for t in range(TOKENS):
        s = int(gold["sum_exp"][t, 0])
        print(f"  SUM_EXP[row {t}] = {s}  = {s/256:.5f}")

    print("\n[Phase 4] Attention Weights (kết quả cuối):")
    print(f"  {'Index':<10} {'Hex':>6}  {'Int':>6}  {'Q8.8 float':>12}")
    print(f"  {'-'*40}")
    for i, w in enumerate(gold["weights_flat"]):
        print(f"  Y[{i//3}][{i%3}]    {int(w) & 0xFFFF:04x}   {int(w):6}   {int(w)/256:.5f}")
    print()


def compare_results(fpga_words, golden_flat):
    """So sánh từng phần tử FPGA vs golden, in bảng, trả về pass/fail."""
    print("\n" + "="*62)
    print("  KẾT QUẢ FPGA vs GOLDEN MODEL")
    print("="*62)
    print(f"  {'Index':<10} {'FPGA':>8}  {' Golden':>8}  {'Diff':>6}  {'Status'}")
    print(f"  {'-'*52}")

    all_pass  = True
    max_diff  = 0

    for i in range(9):
        fpga_val   = fpga_words[i]
        gold_val   = int(golden_flat[i])
        diff       = abs(fpga_val - gold_val)
        max_diff   = max(max_diff, diff)

        # Tolerance: ±2 LSB do fixed-point rounding qua nhiều stage
        status = "OK" if diff <= 2 else "FAIL"
        if status == "FAIL":
            all_pass = False

        flag = "  <-- !" if status == "FAIL" else ""
        print(f"  Y[{i//3}][{i%3}]    "
              f"{int(fpga_val) & 0xFFFF:04x} ({fpga_val:5})  "
              f"{int(gold_val) & 0xFFFF:04x} ({gold_val:5})  "
              f"{diff:5}  {status}{flag}")

    print(f"\n  Max diff: {max_diff} LSB = {max_diff/256:.5f} (Q8.8)")

    print("\n  Kiểm tra tổng mỗi hàng softmax ≈ 1.0:")
    row_pass = True
    for row in range(TOKENS):
        s = sum(fpga_words[row*3 : row*3+3]) / 256.0
        ok = 0.9 < s < 1.1
        if not ok:
            row_pass = False
        print(f"  Hàng {row}: tổng = {s:.5f}  {'[OK]' if ok else '[WARN] ngoài 0.9~1.1'}")

    print()
    if all_pass and row_pass:
        print("  [PASS] FPGA hoạt động đúng!")
    elif all_pass and not row_pass:
        print("  [WARN] Giá trị từng phần tử OK nhưng tổng lệch — kiểm tra divider phase.")
    else:
        print("  [FAIL] Có phần tử lệch > 2 LSB — kiểm tra lại RTL hoặc giao thức UART.")

    return all_pass and row_pass


# ============================================================================
# MAIN
# ============================================================================

def main():
    global PIPELINE_TIMEOUT

    parser = argparse.ArgumentParser(description="Giao tiếp UART với DE10 attention FPGA")
    parser.add_argument("--port",    default=DEFAULT_PORT,
                        help=f"Cổng UART (mặc định: {DEFAULT_PORT})")
    parser.add_argument("--baud",    default=BAUD_RATE, type=int,
                        help=f"Baud rate (mặc định: {BAUD_RATE})")
    parser.add_argument("--dry-run", action="store_true",
                        help="Chỉ in golden model, không kết nối FPGA")
    parser.add_argument("--timeout", default=PIPELINE_TIMEOUT, type=int,
                        help=f"Giây chờ pipeline (mặc định: {PIPELINE_TIMEOUT}s)")
    args = parser.parse_args()

    PIPELINE_TIMEOUT = args.timeout

    # Bước 1: Tính golden model
    print("[GEN] Tính golden model...")
    gold = compute_golden()
    print_golden_summary(gold)

    if args.dry_run:
        print("[DRY-RUN] Không kết nối FPGA. Thoát.")
        return

    # Bước 2: Load X data từ file (ưu tiên file, fallback golden)
    x_words = load_x_from_file(DATA_FILE)
    print(f"[OK] Đã load {len(x_words)} word X từ file.")
    print(f"     x_words[0..3] = {[f'{int(w) & 0xFFFF:04x}' for w in x_words[:4]]}")

    # Bước 3: Mở UART
    ser = open_serial(args.port, args.baud)

    # Flush buffer cũ (phòng khi kit còn dữ liệu từ lần trước)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Bước 4: Đồng bộ controller rồi gửi dữ liệu
    sync_controller(ser)
    send_x_data(ser, x_words)

    # Bước 5: Đọc kết quả
    #
    # Không cần sleep thêm vì:
    #   - Pipeline FPGA xong trong ~0.25ms sau byte cuối đến
    #   - ser.read(18) có timeout=PIPELINE_TIMEOUT → tự chờ nếu chưa có data
    #   - ser.flush() đảm bảo OS đã đẩy hết byte ra cổng COM trước khi read
    #
    # Nếu nhận được 0 byte: kiểm tra uart_controller còn ở state RECV_DATA
    # (chưa nhận đủ 384 byte) → tăng timeout hoặc kiểm tra kết nối.
    fpga_words = read_results(ser)
    ser.close()

    if fpga_words is None:
        sys.exit(1)

    print(f"\n[UART] Nhận được raw bytes: {[f'{int(w) & 0xFFFF:04x}' for w in fpga_words]}")

    # Bước 6: So sánh và báo cáo
    compare_results(fpga_words, gold["weights_flat"])


if __name__ == "__main__":
    main()