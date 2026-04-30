// =============================================================================
// FILE: attention_top.sv
// Mô tả: File tổng hợp toàn bộ thiết kế Attention Mechanism
//
// Thứ tự khai báo module (từ primitive → top-level):
//   1. ram_1port                  — Bộ nhớ RAM 1 cổng dùng chung
//   2. linear_multiply_accumulate — MAC engine tính Q, K, V
//   3. attention_score_calc       — Tính điểm S = Q·Kᵀ / scale
//   4. linear_top                 — Ghép Linear MAC + Attention Score
//   5. softmax_findmax_sub        — Softmax Pass 1: Tìm Max & Z = X - Max
//   6. softmax_exp_sum            — Softmax Pass 2: Tính Exp(Z) & Tổng
//   7. softmax_divider            — Softmax Pass 3: Chia chuẩn hóa Y = Exp/Sum
//   8. attention_top              — Top-level điều phối toàn pipeline
//
// Tham số mặc định: DATA_WIDTH=16, SEQ_LEN=3, D_MODEL=64
// Định dạng số: Fixed-Point Q8.8 (16-bit signed)
// =============================================================================


// =============================================================================
// MODULE 1: ram_1port
// Chức năng: RAM đồng bộ 1 cổng, dùng chung cho Q/K/V/S/Z_INTER/Z_FINAL RAM
// Ghi và đọc đều trên sườn dương clock (synchronous read)
// =============================================================================
module ram_1port #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 8,
    parameter INIT_FILE  = ""
)(
    input  logic clk,
    input  logic we,
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic signed [DATA_WIDTH-1:0] data_in,
    output logic signed [DATA_WIDTH-1:0] data_out
);
    logic signed [DATA_WIDTH-1:0] ram_block [0:(1<<ADDR_WIDTH)-1];
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, ram_block);
        end
    end
    always_ff @(posedge clk) begin
        if (we) ram_block[addr] <= data_in;
        data_out <= ram_block[addr];
    end
endmodule

// =============================================================================
// MODULE: exp_rom (Thay thế IP Quartus)
// Chức năng: ROM 2048 phần tử tra cứu giá trị e^(Z)
// =============================================================================
module exp_rom (
    input  logic [10:0] address,
    input  logic        clock,
    output logic [15:0] q
);
    logic [15:0] rom_block [0:2047];
    initial begin
        $readmemh("exp_rom.txt", rom_block);
    end
    always_ff @(posedge clock) begin
        q <= rom_block[address];
    end
endmodule


// =============================================================================
// MODULE 2: linear_multiply_accumulate
// Chức năng: MAC Engine — tính phép biến đổi tuyến tính X·W cho Q, K, V
//   - Đọc vector X từ input_data ROM
//   - Đọc ma trận Wq, Wk, Wv từ ROM (song song)
//   - Tích lũy MAC qua D_MODEL=64 chiều
//   - Ghi kết quả Q[token][row], K[token][row], V[token][row] vào RAM
// FSM: STATE_IDLE → STATE_COMPUTE → STATE_FINALIZE
// =============================================================================
// =============================================================================
// MODULE 2: linear_multiply_accumulate (Fixed Pipeline Timing)
// =============================================================================
module linear_multiply_accumulate #(
    parameter int D_MODEL_DIMENSION = 64,
    parameter int DATA_WIDTH        = 16
)(
    input  logic i_clk,
    input  logic i_reset_n,
    input  logic i_start_processing,
    input  logic [1:0] i_token_index,
    output logic o_processing_done,
    output logic o_ram_we,
    output logic [7:0] o_ram_write_addr,
    input  logic signed [DATA_WIDTH-1:0] i_input_data_x,
    input  logic signed [DATA_WIDTH-1:0] i_weight_query_data,
    input  logic signed [DATA_WIDTH-1:0] i_weight_key_data,
    input  logic signed [DATA_WIDTH-1:0] i_weight_value_data,
    output logic [7:0]  o_input_x_address,
    output logic [11:0] o_weight_matrix_address,
    output logic signed [DATA_WIDTH-1:0] o_query_result,
    output logic signed [DATA_WIDTH-1:0] o_key_result,
    output logic signed [DATA_WIDTH-1:0] o_value_result
);
    typedef enum logic [1:0] {
        STATE_IDLE     = 2'b00,
        STATE_COMPUTE  = 2'b01,
        STATE_FINALIZE = 2'b10
    } state_t;
    state_t current_state;

    // Tách riêng bộ đếm địa chỉ (Address) và bộ đếm dữ liệu (Data)
    logic [5:0] addr_col, addr_row;
    logic [5:0] data_col, data_row;
    logic       data_valid, pipeline_valid_d1;
    logic       wr_en;
    logic [5:0] wr_row;

    logic signed [39:0] query_accumulator, key_accumulator, value_accumulator;

    // 1. Cấp địa chỉ cho RAM
    assign o_input_x_address       = (i_token_index * D_MODEL_DIMENSION) + addr_col;
    assign o_weight_matrix_address = (addr_row * D_MODEL_DIMENSION) + addr_col;

    always_ff @(posedge i_clk or negedge i_reset_n) begin
        if (!i_reset_n) begin
            current_state <= STATE_IDLE;
            addr_col      <= '0;
            addr_row      <= '0;
            data_valid    <= 1'b0;
        end else begin
            case (current_state)
                STATE_IDLE: begin
                    addr_col   <= '0;
                    addr_row   <= '0;
                    data_valid <= 1'b0;
                    if (i_start_processing) begin
                        current_state <= STATE_COMPUTE;
                        data_valid    <= 1'b1;
                    end
                end
                STATE_COMPUTE: begin
                    if (addr_col == 6'd63) begin
                        addr_col <= '0;
                        if (addr_row == 6'd63) begin
                            current_state <= STATE_FINALIZE;
                            data_valid    <= 1'b0;
                        end else begin
                            addr_row <= addr_row + 1'b1;
                        end
                    end else begin
                        addr_col <= addr_col + 1'b1;
                    end
                end
                STATE_FINALIZE: begin
                    if (!i_start_processing) current_state <= STATE_IDLE;
                end
                default: current_state <= STATE_IDLE;
            endcase
        end
    end

    // 2. Data Stage (Đồng bộ trễ 1 clock chờ RAM xuất data)
    always_ff @(posedge i_clk or negedge i_reset_n) begin
        if (!i_reset_n) begin
            data_col          <= '0;
            data_row          <= '0;
            pipeline_valid_d1 <= 1'b0;
        end else begin
            data_col          <= addr_col;
            data_row          <= addr_row;
            pipeline_valid_d1 <= data_valid;
        end
    end

    // 3. Khối MAC (Sử dụng data_col để reset tổng chính xác)
    always_ff @(posedge i_clk or negedge i_reset_n) begin
        if (!i_reset_n) begin
            query_accumulator <= 40'd0;
            key_accumulator   <= 40'd0;
            value_accumulator <= 40'd0;
        end else if (pipeline_valid_d1) begin
            if (data_col == 6'd0) begin
                // Bắt đầu hàng mới: Reset tổng
                query_accumulator <= (i_input_data_x * i_weight_query_data);
                key_accumulator   <= (i_input_data_x * i_weight_key_data);
                value_accumulator <= (i_input_data_x * i_weight_value_data);
            end else begin
                // Cộng dồn
                query_accumulator <= query_accumulator + (i_input_data_x * i_weight_query_data);
                key_accumulator   <= key_accumulator   + (i_input_data_x * i_weight_key_data);
                value_accumulator <= value_accumulator + (i_input_data_x * i_weight_value_data);
            end
        end
    end

    // 4. Write Stage (Ghi kết quả vào RAM đệm)
    // Khi data_col = 63, phép cộng cuối cùng hoàn thành, chu kỳ sau sẽ ghi RAM
    always_ff @(posedge i_clk or negedge i_reset_n) begin
        if (!i_reset_n) begin
            wr_en  <= 1'b0;
            wr_row <= '0;
        end else begin
            wr_en <= (pipeline_valid_d1 && data_col == 6'd63);
            if (pipeline_valid_d1 && data_col == 6'd63)
                wr_row <= data_row;
        end
    end

    assign o_ram_we         = wr_en;
    assign o_ram_write_addr = (i_token_index * D_MODEL_DIMENSION) + wr_row;

    assign o_query_result = query_accumulator[23:8];
    assign o_key_result   = key_accumulator[23:8];
    assign o_value_result = value_accumulator[23:8];

    // 5. Cờ hoàn tất Phase 1
    logic done_reg;
    always_ff @(posedge i_clk or negedge i_reset_n) begin
        if (!i_reset_n) done_reg <= 1'b0;
        else done_reg <= (wr_en && wr_row == 6'd63);
    end
    assign o_processing_done = done_reg;

endmodule


// =============================================================================
// MODULE 3: attention_score_calc
// Chức năng: Tính ma trận điểm Attention S[q][k] = Q[q] · K[k] / scale
//   - Chiếm quyền đọc Q_RAM, K_RAM qua o_rd_ext_mode=1
//   - Tích lũy dot-product qua D_MODEL=64 chiều
//   - Dịch phải 11 bit để chuẩn hóa (≈ ÷ √64 trong fixed-point)
//   - Ghi 9 phần tử S[0..2][0..2] vào S_RAM (addr = q×3 + k)
// FSM: IDLE → COMPUTE → STORE → DONE
// =============================================================================
module attention_score_calc #(
    parameter D_MODEL = 64,
    parameter SEQ_LEN = 3
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start_calc,
    output logic o_rd_ext_mode,
    output logic [7:0] o_q_addr,
    output logic [7:0] o_k_addr,
    input  logic signed [15:0] i_q_data,
    input  logic signed [15:0] i_k_data,
    output logic o_s_ram_we,
    output logic [3:0] o_s_ram_addr,
    output logic signed [31:0] o_s_ram_data,
    output logic done_calc
);
    typedef enum logic [1:0] {IDLE, COMPUTE, STORE, DONE} state_t;
    state_t current_state;

    logic [1:0] q_idx, k_idx;
    logic [6:0] d_cnt;
    logic signed [39:0] mac_sum;
    logic [5:0] safe_d_cnt;

    // Clamp d_cnt tại 63 để tránh tràn địa chỉ khi d_cnt=64
    assign safe_d_cnt = (d_cnt == 7'd64) ? 6'd63 : d_cnt[5:0];
    // Địa chỉ đọc Q_RAM: q_idx × 64 + dimension
    assign o_q_addr = (q_idx * D_MODEL) + safe_d_cnt;
    // Địa chỉ đọc K_RAM: k_idx × 64 + dimension
    assign o_k_addr = (k_idx * D_MODEL) + safe_d_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            o_rd_ext_mode <= 1'b0;
            o_s_ram_we    <= 1'b0;
            done_calc     <= 1'b0;
            q_idx         <= 2'd0;
            k_idx         <= 2'd0;
            d_cnt         <= 7'd0;
            mac_sum       <= 40'd0;
        end else begin
            case (current_state)
                IDLE: begin
                    done_calc  <= 1'b0;
                    o_s_ram_we <= 1'b0;
                    if (start_calc) begin
                        current_state <= COMPUTE;
                        o_rd_ext_mode <= 1'b1; // Chiếm quyền đọc Q/K RAM
                        q_idx   <= 2'd0;
                        k_idx   <= 2'd0;
                        d_cnt   <= 7'd0;
                        mac_sum <= 40'd0;
                    end
                end
                COMPUTE: begin
                    o_s_ram_we <= 1'b0;
                    // d_cnt=0: clock gửi địa chỉ, chưa có data → không cộng
                    if (d_cnt > 0) mac_sum <= mac_sum + (i_q_data * i_k_data);
                    if (d_cnt == 7'd64) current_state <= STORE;
                    else d_cnt <= d_cnt + 1'b1;
                end
                STORE: begin
                    // Dịch phải 11 bit + làm tròn (cộng bit[10])
                    o_s_ram_data <= (mac_sum >>> 11) + $signed({1'b0, mac_sum[10]});
                    o_s_ram_addr <= (q_idx * SEQ_LEN) + k_idx; // addr = q×3 + k
                    o_s_ram_we   <= 1'b1;
                    d_cnt        <= 7'd0;
                    mac_sum      <= 40'd0;
                    if (k_idx == SEQ_LEN - 1) begin
                        k_idx <= 2'd0;
                        if (q_idx == SEQ_LEN - 1) begin
                            current_state <= DONE;
                        end else begin
                            q_idx         <= q_idx + 1'b1;
                            current_state <= COMPUTE;
                        end
                    end else begin
                        k_idx         <= k_idx + 1'b1;
                        current_state <= COMPUTE;
                    end
                end
                DONE: begin
                    o_s_ram_we    <= 1'b0;
                    o_rd_ext_mode <= 1'b0; // Trả quyền đọc Q/K RAM
                    done_calc     <= 1'b1;
                    if (!start_calc) current_state <= IDLE;
                end
            endcase
        end
    end
endmodule


// =============================================================================
// MODULE 4: linear_top
// Chức năng: Ghép linear_multiply_accumulate + attention_score_calc
//   - Quản lý MUX địa chỉ Q_RAM/K_RAM giữa 2 module
//   - Phase 1: Linear MAC ghi vào Q/K/V RAM
//   - Phase 2: Attention Score đọc Q/K RAM, ghi ra S_RAM
// =============================================================================
module linear_top #(
    parameter D_MODEL = 64,
    parameter SEQ_LEN = 3
)(
    input  logic i_clock,
    input  logic i_reset_n,

    // Cổng ghi RAM đầu vào X (từ uart_controller)
    input  logic        i_rx_ram_we,
    input  logic [7:0]  i_rx_ram_addr,
    input  logic signed [15:0] i_rx_ram_data,

    // Cổng ghi Weight RAM Wq (từ uart_controller)
    input  logic        i_wq_we,
    input  logic [11:0] i_wq_addr,
    input  logic [15:0] i_wq_data,

    // Cổng ghi Weight RAM Wk (từ uart_controller)
    input  logic        i_wk_we,
    input  logic [11:0] i_wk_addr,
    input  logic [15:0] i_wk_data,

    // Cổng ghi Weight RAM Wv (từ uart_controller)
    input  logic        i_wv_we,
    input  logic [11:0] i_wv_addr,
    input  logic [15:0] i_wv_data,

    // Phase 1: Linear
    input  logic i_start_linear,
    input  logic [1:0] i_token_selection,
    output logic o_linear_done,

    // Phase 2: Attention Score
    input  logic i_start_attn,
    output logic o_attn_done,
    output logic o_s_ram_we,
    output logic [3:0] o_s_ram_addr,
    output logic signed [31:0] o_s_ram_data
);

    // Dây nối nội bộ cho Linear MAC
    logic [7:0]  wire_input_x_address;
    logic [11:0] wire_weight_matrix_address;
    logic signed [15:0] wire_data_x_from_ram;
    logic signed [15:0] wire_wq, wire_wk, wire_wv;
    logic wire_linear_we;
    logic [7:0]  wire_linear_wr_addr;
    logic signed [15:0] wire_q_res, wire_k_res, wire_v_res;

    // Dây nối cho Attention Score ↔ Q/K RAM
    logic wire_rd_ext_mode;
    logic [7:0] wire_rd_q_addr, wire_rd_k_addr;
    logic signed [15:0] wire_q_data_out, wire_k_data_out;

    // MUX địa chỉ Q/K RAM: rd_ext_mode=1 → Attention Score đọc; =0 → Linear MAC ghi
    logic [7:0] q_ram_addr_final, k_ram_addr_final;
    assign q_ram_addr_final = wire_rd_ext_mode ? wire_rd_q_addr : wire_linear_wr_addr;
    assign k_ram_addr_final = wire_rd_ext_mode ? wire_rd_k_addr : wire_linear_wr_addr;

    // ROM đầu vào (giả định có sẵn từ công cụ tổng hợp)
    // input_data u_input_ram (
    //     .address(wire_input_x_address), .clock(i_clock),
    //     .data(16'd0), .wren(1'b0), .q(wire_data_x_from_ram)
    // );

    // Thay bằng RAM động:
    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(8)) u_input_ram (
        .clk     (i_clock),
        .we      (i_rx_ram_we),
        .addr    (i_rx_ram_we ? i_rx_ram_addr : wire_input_x_address),
        .data_in (i_rx_ram_data),
        .data_out(wire_data_x_from_ram)
    );

    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(12), .INIT_FILE("weight_q.txt")) u_wq_ram (
        .clk(i_clock), .we(i_wq_we), .addr(i_wq_we ? i_wq_addr : wire_weight_matrix_address),
        .data_in(i_wq_data), .data_out(wire_wq)
    );

    // Weight RAM Wk
    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(12), .INIT_FILE("weight_k.txt")) u_wk_ram (
        .clk(i_clock), .we(i_wk_we), .addr(i_wk_we ? i_wk_addr : wire_weight_matrix_address),
        .data_in(i_wk_data), .data_out(wire_wk)
    );

    // Weight RAM Wv
    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(12), .INIT_FILE("weight_v.txt")) u_wv_ram (
        .clk(i_clock), .we(i_wv_we), .addr(i_wv_we ? i_wv_addr : wire_weight_matrix_address),
        .data_in(i_wv_data), .data_out(wire_wv)
    );

    // MAC Engine
    linear_multiply_accumulate u_compute_engine (
        .i_clk(i_clock),             .i_reset_n(i_reset_n),
        .i_start_processing(i_start_linear),
        .i_token_index(i_token_selection),
        .o_processing_done(o_linear_done),
        .i_input_data_x(wire_data_x_from_ram),
        .i_weight_query_data(wire_wq),
        .i_weight_key_data(wire_wk),
        .i_weight_value_data(wire_wv),
        .o_input_x_address(wire_input_x_address),
        .o_weight_matrix_address(wire_weight_matrix_address),
        .o_ram_we(wire_linear_we),
        .o_ram_write_addr(wire_linear_wr_addr),
        .o_query_result(wire_q_res),
        .o_key_result(wire_k_res),
        .o_value_result(wire_v_res)
    );

    // Q_RAM: khóa ghi khi rd_ext_mode=1 (Attention Score đang đọc)
    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(8)) u_q_ram (
        .clk(i_clock),
        .we(wire_rd_ext_mode ? 1'b0 : wire_linear_we),
        .addr(q_ram_addr_final),
        .data_in(wire_q_res),
        .data_out(wire_q_data_out)
    );

    // K_RAM: tương tự Q_RAM
    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(8)) u_k_ram (
        .clk(i_clock),
        .we(wire_rd_ext_mode ? 1'b0 : wire_linear_we),
        .addr(k_ram_addr_final),
        .data_in(wire_k_res),
        .data_out(wire_k_data_out)
    );

    // V_RAM: chỉ ghi, chưa dùng đọc trong pipeline này
    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(8)) u_v_ram (
        .clk(i_clock),
        .we(wire_rd_ext_mode ? 1'b0 : wire_linear_we),
        .addr(wire_linear_wr_addr),
        .data_in(wire_v_res),
        .data_out()
    );

    // Attention Score Calculator
    attention_score_calc #(.D_MODEL(D_MODEL), .SEQ_LEN(SEQ_LEN)) u_attn_calc (
        .clk(i_clock),             .rst_n(i_reset_n),
        .start_calc(i_start_attn),
        .o_rd_ext_mode(wire_rd_ext_mode),
        .o_q_addr(wire_rd_q_addr), .o_k_addr(wire_rd_k_addr),
        .i_q_data(wire_q_data_out), .i_k_data(wire_k_data_out),
        .o_s_ram_we(o_s_ram_we),
        .o_s_ram_addr(o_s_ram_addr),
        .o_s_ram_data(o_s_ram_data),
        .done_calc(o_attn_done)
    );

endmodule


// =============================================================================
// MODULE 5: softmax_findmax_sub
// Chức năng: Softmax Pass 1 — Tìm max từng hàng & tính Z = S - max
//   - Nhận S_RAM (nội bộ) từ dữ liệu ghi của attention_score_calc
//   - Duyệt 2 vòng mỗi hàng: Vòng 1 tìm max, Vòng 2 trừ max
//   - Ghi Z[row][col] ra Z_INTER_RAM (addr = row×SEQ_LEN + col)
//   - Kết quả Z luôn ≤ 0 (bảo đảm exp(Z) không tràn số)
// FSM: IDLE → RD_MAX → CALC_MAX → WAIT_SUB → RD_SUB
// =============================================================================
module softmax_findmax_sub #(
    parameter DATA_WIDTH = 16,
    parameter SEQ_LEN    = 3
)(
    input  logic clk,
    input  logic rst_n,

    // Nhận dữ liệu S từ attention_score_calc (ghi thụ động vào S_RAM nội bộ)
    input  logic        i_s_ram_we,
    input  logic [3:0]  i_s_ram_wr_addr,
    input  logic signed [31:0] i_s_ram_wr_data,

    input  logic start_softmax,
    output logic done_softmax,

    // Xuất Z ra Z_INTER_RAM
    output logic        o_z_ram_we,
    output logic [3:0]  o_z_ram_wr_addr,
    output logic signed [DATA_WIDTH-1:0] o_z_ram_data
);

    logic [7:0] s_ram_addr;
    logic signed [15:0] s_ram_rd_data;

    // S_RAM nội bộ: khóa ghi khi đang chạy softmax
    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(4)) u_s_ram (
        .clk(clk),
        .we(i_s_ram_we && !start_softmax),
        .addr(i_s_ram_we ? i_s_ram_wr_addr : s_ram_addr[3:0]),
        .data_in(i_s_ram_wr_data[15:0]),
        .data_out(s_ram_rd_data)
    );

    typedef enum logic [2:0] {IDLE, RD_MAX, CALC_MAX, WAIT_SUB, RD_SUB} state_t;
    state_t state;

    logic [7:0] row, col;
    logic signed [15:0] current_max;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= IDLE;
            done_softmax    <= 0;
            o_z_ram_we      <= 0;
            o_z_ram_wr_addr <= 0;
            o_z_ram_data    <= 0;
            row             <= 0;
            col             <= 0;
            s_ram_addr      <= 0;
            current_max     <= 16'h8000; // Âm vô cùng (−32768)
        end else begin
            o_z_ram_we   <= 0;
            done_softmax <= 0;

            case (state)
                IDLE: begin
                    if (start_softmax) begin
                        row         <= 0;
                        col         <= 0;
                        s_ram_addr  <= 0;
                        current_max <= 16'h8000;
                        state       <= RD_MAX;
                    end
                end

                RD_MAX: begin
                    // Đẩy địa chỉ → 1 clock sau RAM xuất data
                    s_ram_addr <= s_ram_addr + 1;
                    state      <= CALC_MAX;
                end

                CALC_MAX: begin
                    // So sánh và cập nhật max
                    if (s_ram_rd_data > current_max)
                        current_max <= s_ram_rd_data;

                    if (col == SEQ_LEN - 1) begin
                        // Xong cả hàng → quay lại đầu hàng để trừ
                        col        <= 0;
                        s_ram_addr <= row * SEQ_LEN;
                        state      <= WAIT_SUB;
                    end else begin
                        col        <= col + 1;
                        s_ram_addr <= s_ram_addr + 1;
                    end
                end

                WAIT_SUB: begin
                    // 1 clock chờ RAM ổn định sau khi đổi địa chỉ
                    s_ram_addr <= s_ram_addr + 1;
                    state      <= RD_SUB;
                end

                RD_SUB: begin
                    // Tính Z = S - max và ghi ra Z_INTER_RAM
                    o_z_ram_we      <= 1;
                    o_z_ram_wr_addr <= (row * SEQ_LEN) + col;
                    o_z_ram_data    <= s_ram_rd_data - current_max;

                    if (col == SEQ_LEN - 1) begin
                        if (row == SEQ_LEN - 1) begin
                            // Toàn bộ ma trận xong
                            done_softmax <= 1;
                            state        <= IDLE;
                        end else begin
                            // Chuyển sang hàng tiếp theo
                            row         <= row + 1;
                            col         <= 0;
                            s_ram_addr  <= (row + 1) * SEQ_LEN;
                            current_max <= 16'h8000;
                            state       <= RD_MAX;
                        end
                    end else begin
                        col        <= col + 1;
                        s_ram_addr <= s_ram_addr + 1;
                    end
                end
            endcase
        end
    end
endmodule


// =============================================================================
// MODULE 6: softmax_exp_sum
// Chức năng: Softmax Pass 2 — Tính Exp(Z) qua LUT và tích lũy tổng
//   - Đọc Z từ Z_INTER_RAM (addr = elem_cnt 0..8)
//   - Tra exp_rom LUT: addr = (-Z) & 0x7FF (2048 entries)
//   - Ghi Exp(Z) vào Z_FINAL_RAM
//   - Tích lũy sum_acc = Σ Exp(Z[i]), xuất o_sum_exp_q88 (24-bit Q16.8)
// FSM: IDLE → RD_SRAM → WAIT_SRAM → WAIT_ROM → WR_ZRAM
// LƯU Ý: o_sum_exp_q88 chỉ valid đúng 1 clock khi done_exp_sum=1
//         → attention_top phải chốt vào sum_exp_latched ngay lập tức
// =============================================================================
module softmax_exp_sum #(
    parameter DATA_WIDTH = 16,
    parameter SEQ_LEN    = 3
)(
    input  logic clk,
    input  logic rst_n,

    input  logic start_exp_sum,
    output logic done_exp_sum,

    // Đọc Z từ Z_INTER_RAM
    output logic [3:0]                   o_s_ram_rd_addr,
    input  logic signed [15:0]           i_s_ram_rd_data,

    // Ghi Exp(Z) vào Z_FINAL_RAM
    output logic                         o_z_ram_we,
    output logic [3:0]                   o_z_ram_wr_addr,
    output logic signed [DATA_WIDTH-1:0] o_z_ram_data,

    // Tổng Σ Exp(Z), Q16.8 (valid 1 clock khi done=1)
    output logic [23:0]                  o_sum_exp_q88 [0:SEQ_LEN-1] // ĐÃ SỬA THÀNH MẢNG
);
    localparam integer TOTAL_ELEM = SEQ_LEN * SEQ_LEN;

    typedef enum logic [2:0] {
        IDLE, RD_SRAM, WAIT_SRAM, WAIT_ROM, WR_ZRAM
    } state_t;
    state_t state;

    logic [3:0]  elem_cnt;
    logic [23:0] sum_acc [0:SEQ_LEN-1]; // ĐÃ SỬA THÀNH MẢNG
    logic [10:0] exp_rom_addr;
    logic [15:0] exp_rom_data;
    
    logic [3:0]  row_idx;
    assign row_idx = elem_cnt / SEQ_LEN;
    
    integer i;

    // Tính địa chỉ ROM combinational: clamp về 0 nếu Z > 0
    always_comb begin
        if (i_s_ram_rd_data <= 0)
            exp_rom_addr = (16'h0000 - i_s_ram_rd_data) & 11'h7FF;
        else
            exp_rom_addr = 11'h000;
    end

    // LUT: bảng exp(-z) được tính sẵn, synchronous output
    exp_rom u_exp_rom (.address(exp_rom_addr), .clock(clk), .q(exp_rom_data));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= IDLE;
            done_exp_sum    <= 1'b0;
            o_s_ram_rd_addr <= 4'h0;
            o_z_ram_we      <= 1'b0;
            o_z_ram_wr_addr <= 4'h0;
            o_z_ram_data    <= {DATA_WIDTH{1'b0}};
            for (i = 0; i < SEQ_LEN; i = i + 1) begin
                o_sum_exp_q88[i] <= 24'h0;
                sum_acc[i]       <= 24'h0;
            end
            elem_cnt        <= 4'h0;
        end else begin
            o_z_ram_we   <= 1'b0;
            done_exp_sum <= 1'b0;

            case (state)
                IDLE: begin
                    elem_cnt <= 4'h0;
                    for (i = 0; i < SEQ_LEN; i = i + 1) begin
                        sum_acc[i]       <= 24'h0;
                        o_sum_exp_q88[i] <= 24'h0;
                    end
                    if (start_exp_sum) begin
                        o_s_ram_rd_addr <= 4'h0;
                        state           <= RD_SRAM;
                    end
                end

                RD_SRAM: begin
                    state <= WAIT_SRAM;
                end

                WAIT_SRAM: begin
                    state <= WAIT_ROM;
                end

                WAIT_ROM: begin
                    state <= WR_ZRAM;
                end

                WR_ZRAM: begin
                    o_z_ram_we      <= 1'b1;
                    o_z_ram_wr_addr <= elem_cnt;
                    o_z_ram_data    <= $signed({1'b0, exp_rom_data[14:0]});
                    
                    // Cộng dồn vào đúng hàng
                    sum_acc[row_idx] <= sum_acc[row_idx] + {8'h0, exp_rom_data};

                    if (elem_cnt == (TOTAL_ELEM - 1)) begin
                        // Xuất tổng ra mảng
                        for (i = 0; i < SEQ_LEN; i = i + 1) begin
                            if (i == row_idx) o_sum_exp_q88[i] <= sum_acc[i] + {8'h0, exp_rom_data};
                            else              o_sum_exp_q88[i] <= sum_acc[i];
                        end
                        done_exp_sum  <= 1'b1;
                        state         <= IDLE;
                    end else begin
                        elem_cnt        <= elem_cnt + 1'b1;
                        o_s_ram_rd_addr <= elem_cnt + 1'b1;
                        state           <= RD_SRAM;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule


// =============================================================================
// MODULE 7: softmax_divider
// Chức năng: Softmax Pass 3 — Chuẩn hóa Y = Exp(Z) / Sum
//   - Đọc Exp(Z) từ Z_FINAL_RAM (addr = elem_cnt 0..8)
//   - Chia: Y = Exp(Z) × 256 / sum_exp  (fixed-point Q8.8)
//   - Bảo vệ chia cho 0: ghi 0 nếu sum=0 hoặc Exp=0
//   - Ghi Attention Weights Y ra output port
// FSM: IDLE → RD_RAM → WAIT_RAM_1 → WAIT_RAM_2 → DONE_ST
// =============================================================================
module softmax_divider #(
    parameter DATA_WIDTH  = 16,
    parameter SEQ_LEN     = 3,
    parameter DIV_LATENCY = 8
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start_div,
    output logic done_div,

    // Mẫu số (tổng Exp, từ sum_exp_latched)
    input  logic [23:0] i_sum_exp [0:SEQ_LEN-1], // ĐÃ SỬA THÀNH MẢNG

    // Đọc Exp(Z) từ Z_FINAL_RAM
    output logic [3:0]              o_z_ram_rd_addr,
    input  logic signed [15:0]      i_z_ram_rd_data,

    // Ghi Attention Weights ra output
    output logic                    o_y_ram_we,
    output logic [3:0]              o_y_ram_wr_addr,
    output logic signed [15:0]      o_y_ram_data
);
    localparam TOTAL_ELEM = SEQ_LEN * SEQ_LEN;

    typedef enum logic [2:0] {
        IDLE, RD_RAM, WAIT_RAM_1, WAIT_RAM_2, DONE_ST
    } state_t;
    state_t state;

    logic [3:0]  elem_cnt;
    logic [31:0] numer;
    
    logic [3:0]  row_idx;
    assign row_idx = elem_cnt / SEQ_LEN;

    // Scale tử số: nhân với 256 để bù fixed-point Q8.8
    assign numer = $unsigned(i_z_ram_rd_data) * 256;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= IDLE;
            done_div        <= 0;
            o_z_ram_rd_addr <= 0;
            o_y_ram_we      <= 0;
            o_y_ram_wr_addr <= 0;
            o_y_ram_data    <= 0;
            elem_cnt        <= 0;
        end else begin
            o_y_ram_we <= 1'b0;
            done_div   <= 1'b0;

            case (state)
                IDLE: begin
                    elem_cnt <= 0;
                    if (start_div) begin
                        o_z_ram_rd_addr <= 0;
                        state           <= RD_RAM;
                    end
                end

                RD_RAM: begin
                    state <= WAIT_RAM_1;
                end

                WAIT_RAM_1: begin
                    state <= WAIT_RAM_2;
                end

                WAIT_RAM_2: begin
                    o_y_ram_we      <= 1'b1;
                    o_y_ram_wr_addr <= elem_cnt;
                    
                    // Lấy i_sum_exp[row_idx] để chia thay vì i_sum_exp chung
                    if (i_sum_exp[row_idx] > 0 && i_z_ram_rd_data > 0)
                        o_y_ram_data <= $signed(numer / i_sum_exp[row_idx]);
                    else
                        o_y_ram_data <= 16'h0000;

                    if (elem_cnt == TOTAL_ELEM - 1) begin
                        state <= DONE_ST;
                    end else begin
                        elem_cnt        <= elem_cnt + 1;
                        o_z_ram_rd_addr <= elem_cnt + 1;
                        state           <= RD_RAM;
                    end
                end

                DONE_ST: begin
                    done_div <= 1'b1;
                    if (!start_div) state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule


// =============================================================================
// MODULE 8: attention_top  (TOP-LEVEL)
// Chức năng: Điều phối toàn bộ pipeline Attention Mechanism
//
//   Phase 1 — Linear (×3 lần, 1 lần/token):
//     i_start_linear → linear_top → o_linear_done
//
//   Phase 2 — Attention Score:
//     i_start_attn → attention_score_calc → o_attn_done
//     → S_RAM (nội bộ softmax_findmax_sub) được nạp tự động
//
//   Phase 3 — Softmax (3 pass tự động nối tiếp):
//     i_start_softmax
//       → Pass 1: softmax_findmax_sub → done → trigger Pass 2
//       → Pass 2: softmax_exp_sum     → done → chốt sum_exp + trigger Pass 3
//       → Pass 3: softmax_divider     → o_softmax_done
//
//   Output: Attention Weights Y[0..8] tại o_z_ram_we/wr_addr/data
//
// RAM trung gian:
//   Z_INTER_RAM: findmax_sub → exp_sum  (Z = S - max)
//   Z_FINAL_RAM: exp_sum    → divider   (Exp(Z))
// =============================================================================
// =============================================================================
// MODULE 8: attention_top  (TOP-LEVEL)
// =============================================================================
module attention_top #(
    parameter DATA_WIDTH = 16,
    parameter SEQ_LEN    = 3
)(
    input  logic clk,
    input  logic rst_n,

    // Phase 1 & 2
    input  logic        i_start_linear,
    input  logic [1:0]  i_token_selection,
    output logic        o_linear_done,
    input  logic        i_start_attn,
    output logic        o_attn_done,

    // Phase 3
    input  logic        i_start_softmax,
    output logic        o_softmax_done,
    output logic [23:0] o_softmax_sum [0:SEQ_LEN-1],

    // Tín hiệu RAM UART — Input X
    input  logic        i_rx_ram_we,
    input  logic [7:0]  i_rx_ram_addr,
    input  logic signed [15:0] i_rx_ram_data,

    // Tín hiệu RAM UART — Weight Wq
    input  logic        i_wq_we,
    input  logic [11:0] i_wq_addr,
    input  logic [15:0] i_wq_data,

    // Tín hiệu RAM UART — Weight Wk
    input  logic        i_wk_we,
    input  logic [11:0] i_wk_addr,
    input  logic [15:0] i_wk_data,

    // Tín hiệu RAM UART — Weight Wv
    input  logic        i_wv_we,
    input  logic [11:0] i_wv_addr,
    input  logic [15:0] i_wv_data,

    // Output: Attention Weights (kết quả cuối)
    output logic                         o_z_ram_we,
    output logic [3:0]                   o_z_ram_wr_addr,
    output logic signed [DATA_WIDTH-1:0] o_z_ram_data
);

    // ── Dây nối S_RAM (attention_score_calc → softmax_findmax_sub) ──
    logic        wire_s_ram_we;
    logic [3:0]  wire_s_ram_wr_addr;
    logic signed [31:0] wire_s_ram_wr_data;

    // ── Dây nối nội bộ Softmax ──
    logic [3:0]  wire_exp_rd_addr;
    logic        wire_find_max_done;
    logic        start_exp_sum_trigger;
    logic        exp_sum_done_trigger;
    logic        start_div_trigger;
    
    logic [23:0] sum_exp_latched [0:SEQ_LEN-1]; // ĐÃ SỬA THÀNH MẢNG
    integer i;

    // ── Z_INTER_RAM: Lưu Z = S - max ──
    logic        z_inter_we;
    logic [3:0]  z_inter_wr_addr;
    logic signed [15:0] z_inter_wr_data;
    logic signed [15:0] z_inter_rd_data;
    logic [3:0]  z_inter_ram_addr;

    // MUX địa chỉ: ghi ưu tiên khi z_inter_we=1
    assign z_inter_ram_addr = z_inter_we ? z_inter_wr_addr : wire_exp_rd_addr;

    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(4)) u_z_inter_ram (
        .clk(clk),
        .we(z_inter_we),
        .addr(z_inter_ram_addr),
        .data_in(z_inter_wr_data),
        .data_out(z_inter_rd_data)
    );

    // ── Z_FINAL_RAM: Lưu Exp(Z) ──
    logic        z_final_we;
    logic [3:0]  z_final_wr_addr;
    logic signed [15:0] z_final_wr_data;
    logic [3:0]  z_final_rd_addr;
    logic signed [15:0] z_final_rd_data;

    ram_1port #(.DATA_WIDTH(16), .ADDR_WIDTH(4)) u_z_final_ram (
        .clk(clk),
        .we(z_final_we),
        .addr(z_final_we ? z_final_wr_addr : z_final_rd_addr),
        .data_in(z_final_wr_data),
        .data_out(z_final_rd_data)
    );

    // ── 1. Linear + Attention Score ──
    linear_top #(.D_MODEL(64), .SEQ_LEN(SEQ_LEN)) u_linear (
        .i_clock(clk),              .i_reset_n(rst_n),

        // Input X RAM
        .i_rx_ram_we(i_rx_ram_we),
        .i_rx_ram_addr(i_rx_ram_addr),
        .i_rx_ram_data(i_rx_ram_data),

        // Weight RAMs
        .i_wq_we(i_wq_we),   .i_wq_addr(i_wq_addr),   .i_wq_data(i_wq_data),
        .i_wk_we(i_wk_we),   .i_wk_addr(i_wk_addr),   .i_wk_data(i_wk_data),
        .i_wv_we(i_wv_we),   .i_wv_addr(i_wv_addr),   .i_wv_data(i_wv_data),

        .i_start_linear(i_start_linear),
        .i_token_selection(i_token_selection),
        .o_linear_done(o_linear_done),
        .i_start_attn(i_start_attn), .o_attn_done(o_attn_done),
        .o_s_ram_we(wire_s_ram_we),
        .o_s_ram_addr(wire_s_ram_wr_addr),
        .o_s_ram_data(wire_s_ram_wr_data)
    );

    // ── 2. Softmax Pass 1: Tìm Max & Trừ ──
    softmax_findmax_sub #(.DATA_WIDTH(DATA_WIDTH), .SEQ_LEN(SEQ_LEN)) u_softmax_max (
        .clk(clk),           .rst_n(rst_n),
        .i_s_ram_we(wire_s_ram_we),
        .i_s_ram_wr_addr(wire_s_ram_wr_addr),
        .i_s_ram_wr_data(wire_s_ram_wr_data),
        .start_softmax(i_start_softmax),
        .done_softmax(wire_find_max_done),
        .o_z_ram_we(z_inter_we),
        .o_z_ram_wr_addr(z_inter_wr_addr),
        .o_z_ram_data(z_inter_wr_data)
    );

    // Trigger tự động Pass 2 khi Pass 1 xong
    assign start_exp_sum_trigger = wire_find_max_done;

    // ── 3. Softmax Pass 2: Exp & Sum ──
    softmax_exp_sum #(.DATA_WIDTH(DATA_WIDTH), .SEQ_LEN(SEQ_LEN)) u_exp_sum (
        .clk(clk),           .rst_n(rst_n),
        .start_exp_sum(start_exp_sum_trigger),
        .done_exp_sum(exp_sum_done_trigger),
        .o_s_ram_rd_addr(wire_exp_rd_addr),
        .i_s_ram_rd_data(z_inter_rd_data),
        .o_z_ram_we(z_final_we),
        .o_z_ram_wr_addr(z_final_wr_addr),
        .o_z_ram_data(z_final_wr_data),
        .o_sum_exp_q88(o_softmax_sum)
    );

    // FIX: Chốt mảng sum_exp ngay clock done
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < SEQ_LEN; i = i + 1) sum_exp_latched[i] <= 24'h0;
        end else if (exp_sum_done_trigger) begin
            for (i = 0; i < SEQ_LEN; i = i + 1) sum_exp_latched[i] <= o_softmax_sum[i];
        end
    end

    // Trigger tự động Pass 3 khi Pass 2 xong
    assign start_div_trigger     = exp_sum_done_trigger;
    // ── 4. Softmax Pass 3: Divider ──
    softmax_divider #(
        .DATA_WIDTH(DATA_WIDTH), .SEQ_LEN(SEQ_LEN), .DIV_LATENCY(9)
    ) u_div (
        .clk(clk),           .rst_n(rst_n),
        .start_div(start_div_trigger),
        .done_div(o_softmax_done),
        .i_sum_exp(sum_exp_latched),     // MẢNG ĐÃ ĐƯỢC CHUYỂN VÀO
        .o_z_ram_rd_addr(z_final_rd_addr),
        .i_z_ram_rd_data(z_final_rd_data),
        .o_y_ram_we(o_z_ram_we),
        .o_y_ram_wr_addr(o_z_ram_wr_addr),
        .o_y_ram_data(o_z_ram_data)
    );

endmodule