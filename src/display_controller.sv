// =============================================================================
// MODULE: display_controller
// Chức năng: Nhận 9 attention weights Y (Q8.8), hiển thị lên LCD 16x2
//
// Layout màn hình (2 trang, chuyển bằng KEY[1] active-low):
//
//   Trang 0:
//     Dòng 1: "Y0:.34 .31 .34  "
//     Dòng 2: "Y1:.32 .33 .34  "
//
//   Trang 1:
//     Dòng 1: "Y2:.34 .32 .34  "
//     Dòng 2: "SUM~1.0  DONE   "
// =============================================================================
module display_controller #(
    parameter int CLK_FREQ = 50_000_000,
    parameter int SEQ_LEN  = 3
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        i_result_ready,
    input  logic        i_y_we,
    input  logic [3:0]  i_y_addr,
    input  logic signed [15:0] i_y_data,

    input  logic        i_key_next,   // KEY[1], active-low

    output logic [7:0]  o_lcd_data,
    output logic        o_lcd_rs,
    output logic        o_lcd_valid,
    input  logic        i_lcd_ready
);

    // ── Chốt 9 giá trị Y ────────────────────────────────────────────────────
    logic [15:0] y_reg [0:8];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 9; i++) y_reg[i] <= 16'h0;
        end else if (i_y_we) begin
            y_reg[i_y_addr] <= i_y_data;
        end
    end

    // ── Debounce KEY (10ms) ─────────────────────────────────────────────────
    localparam int DEBOUNCE_CLKS = CLK_FREQ / 100;
    logic [19:0] dbnc_cnt;
    logic        key_prev, key_pulse;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbnc_cnt  <= '0;
            key_prev  <= 1'b1;
            key_pulse <= 1'b0;
        end else begin
            key_pulse <= 1'b0;
            if (i_key_next == key_prev) begin
                dbnc_cnt <= '0;
            end else begin
                if (dbnc_cnt == DEBOUNCE_CLKS - 1) begin
                    dbnc_cnt <= '0;
                    key_prev <= i_key_next;
                    if (i_key_next == 1'b1) key_pulse <= 1'b1;
                end else begin
                    dbnc_cnt <= dbnc_cnt + 1'b1;
                end
            end
        end
    end

    // ── Quản lý trang ───────────────────────────────────────────────────────
    logic page, page_latch;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)               page <= 1'b0;
        else if (!i_result_ready) page <= 1'b0;
        else if (key_pulse)       page <= ~page;
    end

    // ── Tính phần thập phân 2 chữ số từ Q8.8 ───────────────────────────────
    // frac = (val * 100) >> 8
    // tens = frac / 10, unit = frac % 10
    // Tính combinational để dùng trong always_ff khi build buffer
    logic [7:0] frac_val [0:8];
    logic [7:0] tens_val [0:8];
    logic [7:0] unit_val [0:8];

    genvar gi;
    generate
        for (gi = 0; gi < 9; gi++) begin : gen_frac
            assign frac_val[gi] = (y_reg[gi][15:0] * 16'd100) >> 8;
            assign tens_val[gi] = frac_val[gi] / 10;
            assign unit_val[gi] = frac_val[gi] % 10;
        end
    endgenerate

    // ── Buffer 16 ký tự mỗi dòng ────────────────────────────────────────────
    logic [7:0] line1 [0:15];
    logic [7:0] line2 [0:15];

    // ── FSM ghi LCD ─────────────────────────────────────────────────────────
    typedef enum logic [3:0] {
        WAIT_READY,
        CLEAR,
        WAIT_CLEAR,
        BUILD_BUF,
        SET_LINE1,
        SEND_LINE1,
        SET_LINE2,
        SEND_LINE2,
        DONE_DISP
    } disp_state_t;
    disp_state_t disp_state;

    logic [4:0]  char_idx;
    logic [19:0] wait_cnt;

    localparam int CLKS_2MS = CLK_FREQ / 500;

    // ── Build buffer (1 cycle, combinational từ y_reg) ───────────────────────
    // Thực hiện trong state BUILD_BUF để tránh dùng task/function phức tạp
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 16; i++) begin
                line1[i] <= 8'h20;
                line2[i] <= 8'h20;
            end
        end else if (disp_state == BUILD_BUF) begin
            // ── Dòng 1: hàng r1 ──────────────────────────────
            // Trang 0 → hàng 0 (y[0..2]), Trang 1 → hàng 2 (y[6..8])
            if (!page_latch) begin
                line1[0]  <= 8'h59;                  // 'Y'
                line1[1]  <= 8'h30;                  // '0'
                line1[2]  <= 8'h3A;                  // ':'
                line1[3]  <= 8'h2E;                  // '.'
                line1[4]  <= 8'h30 + tens_val[0];
                line1[5]  <= 8'h30 + unit_val[0];
                line1[6]  <= 8'h20;
                line1[7]  <= 8'h2E;
                line1[8]  <= 8'h30 + tens_val[1];
                line1[9]  <= 8'h30 + unit_val[1];
                line1[10] <= 8'h20;
                line1[11] <= 8'h2E;
                line1[12] <= 8'h30 + tens_val[2];
                line1[13] <= 8'h30 + unit_val[2];
                line1[14] <= 8'h20;
                line1[15] <= 8'h20;

                line2[0]  <= 8'h59;                  // 'Y'
                line2[1]  <= 8'h31;                  // '1'
                line2[2]  <= 8'h3A;
                line2[3]  <= 8'h2E;
                line2[4]  <= 8'h30 + tens_val[3];
                line2[5]  <= 8'h30 + unit_val[3];
                line2[6]  <= 8'h20;
                line2[7]  <= 8'h2E;
                line2[8]  <= 8'h30 + tens_val[4];
                line2[9]  <= 8'h30 + unit_val[4];
                line2[10] <= 8'h20;
                line2[11] <= 8'h2E;
                line2[12] <= 8'h30 + tens_val[5];
                line2[13] <= 8'h30 + unit_val[5];
                line2[14] <= 8'h20;
                line2[15] <= 8'h20;
            end else begin
                line1[0]  <= 8'h59;                  // 'Y'
                line1[1]  <= 8'h32;                  // '2'
                line1[2]  <= 8'h3A;
                line1[3]  <= 8'h2E;
                line1[4]  <= 8'h30 + tens_val[6];
                line1[5]  <= 8'h30 + unit_val[6];
                line1[6]  <= 8'h20;
                line1[7]  <= 8'h2E;
                line1[8]  <= 8'h30 + tens_val[7];
                line1[9]  <= 8'h30 + unit_val[7];
                line1[10] <= 8'h20;
                line1[11] <= 8'h2E;
                line1[12] <= 8'h30 + tens_val[8];
                line1[13] <= 8'h30 + unit_val[8];
                line1[14] <= 8'h20;
                line1[15] <= 8'h20;

                // "SUM~1.0  DONE   "
                line2[0]  <= 8'h53; // S
                line2[1]  <= 8'h55; // U
                line2[2]  <= 8'h4D; // M
                line2[3]  <= 8'h7E; // ~
                line2[4]  <= 8'h31; // 1
                line2[5]  <= 8'h2E; // .
                line2[6]  <= 8'h30; // 0
                line2[7]  <= 8'h20; // (space)
                line2[8]  <= 8'h20;
                line2[9]  <= 8'h44; // D
                line2[10] <= 8'h4F; // O
                line2[11] <= 8'h4E; // N
                line2[12] <= 8'h45; // E
                line2[13] <= 8'h20;
                line2[14] <= 8'h20;
                line2[15] <= 8'h20;
            end
        end
    end

    // ── FSM chính ────────────────────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disp_state  <= WAIT_READY;
            char_idx    <= '0;
            o_lcd_data  <= '0;
            o_lcd_rs    <= 1'b0;
            o_lcd_valid <= 1'b0;
            wait_cnt    <= '0;
            page_latch  <= 1'b0;
        end else begin
            o_lcd_valid <= 1'b0;

            case (disp_state)

                WAIT_READY: begin
                    if (i_result_ready) begin
                        page_latch <= page;
                        disp_state <= CLEAR;
                    end
                end

                CLEAR: begin
                    if (i_lcd_ready) begin
                        o_lcd_data  <= 8'h01; // Clear display
                        o_lcd_rs    <= 1'b0;
                        o_lcd_valid <= 1'b1;
                        wait_cnt    <= '0;
                        disp_state  <= WAIT_CLEAR;
                    end
                end

                WAIT_CLEAR: begin
                    // HD44780 cần ~1.52ms để clear, chờ 2ms cho chắc
                    if (wait_cnt == CLKS_2MS - 1) begin
                        wait_cnt   <= '0;
                        disp_state <= BUILD_BUF;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                BUILD_BUF: begin
                    // 1 cycle để always_ff build line1/line2 xong
                    disp_state <= SET_LINE1;
                end

                SET_LINE1: begin
                    if (i_lcd_ready) begin
                        o_lcd_data  <= 8'h80; // DDRAM addr dòng 1
                        o_lcd_rs    <= 1'b0;
                        o_lcd_valid <= 1'b1;
                        char_idx    <= '0;
                        disp_state  <= SEND_LINE1;
                    end
                end

                SEND_LINE1: begin
                    if (i_lcd_ready) begin
                        if (char_idx < 16) begin
                            o_lcd_data  <= line1[char_idx];
                            o_lcd_rs    <= 1'b1;
                            o_lcd_valid <= 1'b1;
                            char_idx    <= char_idx + 1'b1;
                        end else begin
                            disp_state <= SET_LINE2;
                        end
                    end
                end

                SET_LINE2: begin
                    if (i_lcd_ready) begin
                        o_lcd_data  <= 8'hC0; // DDRAM addr dòng 2
                        o_lcd_rs    <= 1'b0;
                        o_lcd_valid <= 1'b1;
                        char_idx    <= '0;
                        disp_state  <= SEND_LINE2;
                    end
                end

                SEND_LINE2: begin
                    if (i_lcd_ready) begin
                        if (char_idx < 16) begin
                            o_lcd_data  <= line2[char_idx];
                            o_lcd_rs    <= 1'b1;
                            o_lcd_valid <= 1'b1;
                            char_idx    <= char_idx + 1'b1;
                        end else begin
                            disp_state <= DONE_DISP;
                        end
                    end
                end

                DONE_DISP: begin
                    // Cập nhật lại nếu trang thay đổi hoặc mất result_ready
                    if (page != page_latch || !i_result_ready)
                        disp_state <= WAIT_READY;
                end

                default: disp_state <= WAIT_READY;
            endcase
        end
    end

endmodule