import numpy as np
import os

# ==========================================
# 1. CẤU HÌNH THÔNG SỐ (Hệ thống Q8.8)
# ==========================================
TOKENS = 3
D_MODEL = 64
DATA_WIDTH = 16
FRAC_BITS = 8  # 1.0 thực = 256 đơn vị nguyên
ROM_DEPTH = 2048

def float_to_q88(val_array):
    """Chuyển đổi mảng số thực sang số nguyên Q8.8 (16-bit có dấu)"""
    q88_vals = np.round(val_array * (1 << FRAC_BITS))
    return np.clip(q88_vals, -32768, 32767).astype(np.int64)

# ==========================================
# 2. CÁC HÀM XUẤT FILE (MIF & INTEL HEX)
# ==========================================
def write_mif(filename, data, width, depth):
    """Ghi dữ liệu ra file .mif định dạng Quartus"""
    filepath = os.path.join(os.getcwd(), filename)
    with open(filepath, 'w') as f:
        f.write(f"DEPTH = {depth};\nWIDTH = {width};\n")
        f.write("ADDRESS_RADIX = DEC;\nDATA_RADIX = HEX;\n")
        f.write("CONTENT BEGIN\n")
        for i, val in enumerate(data):
            hex_val = int(val) & 0xFFFF
            f.write(f"  {i} : {hex_val:04X};\n")
        if len(data) < depth:
            f.write(f"  [{len(data)}..{depth-1}] : 0000;\n")
        f.write("END;\n")

def write_intel_hex(filename, data):
    """Ghi dữ liệu theo định dạng Intel HEX chuẩn (:LLAAAATTDDDDCC)"""
    filepath = os.path.join(os.getcwd(), filename)
    with open(filepath, 'w') as f:
        for i, val in enumerate(data):
            val = int(val) & 0xFFFF
            length = 2          # 2 bytes cho mỗi từ 16-bit
            addr = i            # Địa chỉ dòng
            rectype = 0         # Data Record
            high_byte = (val >> 8) & 0xFF
            low_byte = val & 0xFF
            
            # Tính Checksum: 0x100 - (sum & 0xFF)
            checksum_sum = length + (addr >> 8) + (addr & 0xFF) + rectype + high_byte + low_byte
            checksum = (0x100 - (checksum_sum & 0xFF)) & 0xFF
            
            f.write(f":{length:02X}{addr:04X}{rectype:02X}{high_byte:02X}{low_byte:02X}{checksum:02X}\n")
        
        # End of File Record (Bắt buộc cho Quartus IP Core)
        f.write(":00000001FF\n")

def write_verilog_hex(filename, data):
    """Ghi dữ liệu Hex thuần túy để ModelSim dùng với $readmemh"""
    filepath = os.path.join(os.getcwd(), filename)
    with open(filepath, 'w') as f:
        for val in data:
            f.write(f"{int(val) & 0xFFFF:04X}\n")

# ==========================================
# 3. TẠO BẢNG TRA CỨU EXP LUT
# ==========================================
def generate_exp_lut():
    lut = []
    for i in range(ROM_DEPTH):
        # Địa chỉ i tương ứng x = -i/256 (vì Softmax tính e^z với z <= 0)
        x = -i / (1 << FRAC_BITS)
        val = np.exp(x)
        q_val = int(np.round(val * (1 << FRAC_BITS)))
        
        # Đảm bảo không mất mát giá trị nhỏ (ép về tối thiểu 1 thay vì 0)
        if q_val == 0 and val > 0:
            q_val = 1
        lut.append(q_val)
    return lut

# ==========================================
# 4. CHƯƠNG TRÌNH CHÍNH (GENERATE ALL)
# ==========================================
if __name__ == "__main__":
    # A. Tạo ROM cho hàm Exp
    exp_lut_data = generate_exp_lut()
    write_verilog_hex("exp_rom.txt", exp_lut_data) # Xuất cho module exp_rom mới

    np.random.seed(42)
    range_value =0.3
    # B. Input và Weights [-0.35, 0.35]
    X  = np.random.uniform(-range_value, range_value, (TOKENS, D_MODEL)).astype(np.float32)
    Wq = np.random.uniform(-range_value, range_value, (D_MODEL, D_MODEL)).astype(np.float32)
    Wk = np.random.uniform(-range_value, range_value, (D_MODEL, D_MODEL)).astype(np.float32)
    Wv = np.random.uniform(-range_value, range_value, (D_MODEL, D_MODEL)).astype(np.float32)

    write_verilog_hex("x_data_sim.txt", float_to_q88(X).flatten())
    write_verilog_hex("weight_q.txt", float_to_q88(Wq).flatten())
    write_verilog_hex("weight_k.txt", float_to_q88(Wk).flatten())
    write_verilog_hex("weight_v.txt", float_to_q88(Wv).flatten())

    # C. GOLDEN MODEL - BIT-TRUE SIMULATION
    X_int, Wq_int, Wk_int, Wv_int = map(float_to_q88, [X, Wq, Wk, Wv])

    # Phase 1: FPGA cắt 8 bit thấp (= floor chia 256)
    Q_int = np.floor(np.dot(X_int, Wq_int.T) / 256.0).astype(np.int64)
    K_int = np.floor(np.dot(X_int, Wk_int.T) / 256.0).astype(np.int64)
    Q_q88, K_q88 = Q_int, K_int

    # Phase 2: Attention Score (dịch 11 bit và cộng bit làm tròn)
    mac_sum = np.dot(Q_int, K_int.T)
    Score_int = np.floor(mac_sum / 2048.0).astype(np.int64) + ((mac_sum >> 10) & 1)
    Score_flat = Score_int.flatten()

    # Phase 3: Softmax Max & Z
    max_score = np.max(Score_int, axis=1, keepdims=True)
    Z_int = Score_int - max_score

    # Giả lập phần cứng truy xuất exp_rom
    exp_lut_array = np.array(exp_lut_data)
    exp_Z_int = np.zeros_like(Z_int)
    for i in range(Z_int.shape[0]):
        for j in range(Z_int.shape[1]):
            val = Z_int[i, j]
            if val <= 0:
                addr = (-val) & 0x7FF
                exp_Z_int[i, j] = exp_lut_array[addr] if addr < ROM_DEPTH else 0

    exp_Z_flat = exp_Z_int.flatten()
    sum_exp_int = np.sum(exp_Z_int, axis=1, keepdims=True)

    # Phase 4: Divider
    weights_int_2d = (exp_Z_int * 256) // sum_exp_int
    weights_int = weights_int_2d.flatten()


    # ---------------------------------------------------------
    # IN KẾT QUẢ ĐỐI CHIẾU
    # ---------------------------------------------------------
    print("\n" + "="*65)
    print("  BẢNG ĐỐI CHIẾU GOLDEN MODEL (PYTHON) VS HARDWARE (VERILOG)")
    print("="*65)

    print("\n--- PHASE 1: LINEAR (Q & K MATRICES) ---")
    print(" * Ghi chú: Chỉ in 4 phần tử đầu tiên của mỗi Token để dễ kiểm tra")
    print("-" * 65)
    for t in range(TOKENS):
        print(f" [TOKEN {t}] Ma trận Q (4 ptử đầu): ", end="")
        for i in range(min(4, D_MODEL)):
            val = int(Q_q88[t, i])
            print(f"{val & 0xFFFF:04x} ({val:^4}) | ", end="")
        print("")
        print(f" [TOKEN {t}] Ma trận K (4 ptử đầu): ", end="")
        for i in range(min(4, D_MODEL)):
            val = int(K_q88[t, i])
            print(f"{val & 0xFFFF:04x} ({val:^4}) | ", end="")
        print("\n" + "-" * 65)

    print("\n--- PHASE 2: ATTENTION SCORE (Q * K^T / sqrt(d_model)) ---")
    print("-" * 58)
    print(" [SCORE] Dia chi | Score (Hex)  | Gia tri int | Thuc (Q8.8)")
    print("-" * 58)
    for i in range(len(Score_flat)):
        val_int = int(Score_flat[i])
        val_hex = f"{val_int & 0xFFFF:04x}"
        val_float = val_int / 256.0
        print(f" [SCORE]   [{i:^2}]   |      {val_hex}      |    {val_int:<4}     |   {val_float:.6f}")
    print("-" * 58)
    
    print("\n--- PHASE 3: SOFTMAX (MAX -> EXP & SUM) ---")
    print("-" * 58)
    print(" [EXP]   Dia chi | Exp(Z) Hex   | Gia tri int | Thuc (Q8.8)")
    print("-" * 58)
    for i in range(len(exp_Z_flat)):
        val_int = int(exp_Z_flat[i])
        val_hex = f"{val_int & 0xFFFF:04x}"
        val_float = val_int / 256.0
        print(f" [EXP]     [{i:^2}]   |      {val_hex}      |    {val_int:<4}     |   {val_float:.6f}")
    print("-" * 58)
    
    # ĐÃ SỬA: In tổng mẫu số riêng cho từng hàng (Token)
    for t in range(TOKENS):
        val_sum = int(sum_exp_int[t, 0])
        print(f" >> TONG MAU SO ROW {t} (SUM_EXP Q16.8): {val_sum / 256.0:.6f} (Gia tri int: {val_sum})")
    print("-" * 58)

    print("\n--- PHASE 4: SOFTMAX (DIVIDER -> ATTENTION WEIGHTS) ---")
    print("-" * 58)
    print(" [DIV]   Dia chi | Weight (Hex) | Gia tri int | Thuc (Q8.8)")
    print("-" * 58)
    for i in range(len(weights_int)):
        w_int = int(weights_int[i])
        w_hex = f"{w_int & 0xFFFF:04x}"
        w_float = w_int / 256.0
        print(f" [DIV]     [{i:^2}]   |      {w_hex}      |    {w_int:<4}     |   {w_float:.6f}")
    print("-" * 58)
    print("HOAN TAT PYTHON GEN TEST.\n")