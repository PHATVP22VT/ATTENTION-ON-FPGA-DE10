// =============================================================================
// MODULE: hex_display_controller
// Chức năng: Hiển thị 9 attention weights Y lên 6 màn hình HEX 7-đoạn
//            của DE10-Standard
//
// Layout hiển thị (mỗi lần hiển thị 1 hàng = 3 giá trị hex):
//   HEX5-HEX4 : Y[row][0]  (4 chữ số hex)
//   HEX3-HEX2 : Y[row][1]
//   HEX1-HEX0 : Y[row][2]
//
//   Nhấn KEY[1] (active-low) để cuộn: row 0 → 1 → 2 → 0
//
// HEX 7-đoạn DE10-Standard: common anode → mức LOW = bật đoạn
//
//   Bố cục đoạn:
//      _
//     |_|   seg: 6=a(top) 5=b 4=c 3=d(bot) 2=e 1=f 0=g(mid)
//     |_|
//
// =============================================================================
module hex_display_controller #(
    parameter int CLK_FREQ = 50_000_000
)(
    input  logic        clk,
    input  logic        rst_n,

    // Kết quả Y từ attention_top (chốt từng giá trị khi i_y_we=1)
    input  logic        i_result_ready,
    input  logic        i_y_we,
    input  logic [3:0]  i_y_addr,
    input  logic [15:0] i_y_data,

    // KEY[1] active-low để chuyển hàng
    input  logic        i_key_next,

    // 6 màn hình HEX, mỗi cái 7 bit (active-low)
    output logic [6:0]  HEX0,
    output logic [6:0]  HEX1,
    output logic [6:0]  HEX2,
    output logic [6:0]  HEX3,
    output logic [6:0]  HEX4,
    output logic [6:0]  HEX5
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

    // ── Debounce KEY[1] (10ms) ───────────────────────────────────────────────
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
                    // Rising edge (nhả nút active-low) = chuyển hàng
                    if (i_key_next == 1'b1) key_pulse <= 1'b1;
                end else begin
                    dbnc_cnt <= dbnc_cnt + 1'b1;
                end
            end
        end
    end

    // ── Quản lý hàng hiển thị ───────────────────────────────────────────────
    logic [5:0] row_sel; // 0, 1, 2

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_sel <= 2'd0;
        end else if (!i_result_ready) begin
            row_sel <= 2'd0;       // Reset về hàng 0 khi chưa có kết quả
        end else if (key_pulse) begin
            if (row_sel == 2'd2)
                row_sel <= 2'd0;
            else
                row_sel <= row_sel + 1'b1;
        end
    end

    // ── Chọn 3 giá trị cần hiển thị theo hàng ───────────────────────────────
    logic [15:0] disp0, disp1, disp2;

    always_comb begin
        case (row_sel)
            2'd0: begin disp0 = y_reg[0]; disp1 = y_reg[1]; disp2 = y_reg[2]; end
            2'd1: begin disp0 = y_reg[3]; disp1 = y_reg[4]; disp2 = y_reg[5]; end
            2'd2: begin disp0 = y_reg[6]; disp1 = y_reg[7]; disp2 = y_reg[8]; end
            default: begin disp0 = 16'h0; disp1 = 16'h0; disp2 = 16'h0; end
        endcase
        // Nếu chưa có kết quả: hiển thị "------" (tất cả đoạn giữa)
        if (!i_result_ready) begin
            disp0 = 16'hFFFF;
            disp1 = 16'hFFFF;
            disp2 = 16'hFFFF;
        end
    end

    // ── Bảng mã 7-đoạn (active-low, common anode) ───────────────────────────
    // Thứ tự bit: [6]=a [5]=b [4]=c [3]=d [2]=e [1]=f [0]=g
    function automatic [6:0] seg7(input [3:0] nibble);
        case (nibble)
            4'h0: seg7 = 7'b1000000; // 0
            4'h1: seg7 = 7'b1111001; // 1
            4'h2: seg7 = 7'b0100100; // 2
            4'h3: seg7 = 7'b0110000; // 3
            4'h4: seg7 = 7'b0011001; // 4
            4'h5: seg7 = 7'b0010010; // 5
            4'h6: seg7 = 7'b0000010; // 6
            4'h7: seg7 = 7'b1111000; // 7
            4'h8: seg7 = 7'b0000000; // 8
            4'h9: seg7 = 7'b0010000; // 9
            4'hA: seg7 = 7'b0001000; // A
            4'hB: seg7 = 7'b0000011; // b
            4'hC: seg7 = 7'b1000110; // C
            4'hD: seg7 = 7'b0100001; // d
            4'hE: seg7 = 7'b0000110; // E
            4'hF: seg7 = 7'b0001110; // F
            default: seg7 = 7'b1111111; // tắt
        endcase
    endfunction

    // ── Xuất ra HEX ─────────────────────────────────────────────────────────
    // Nếu chưa có kết quả: hiển thị dấu "-" (chỉ đoạn giữa = g sáng)
    // active-low nên "-" = 7'b1111110 (chỉ bit g=0)
    localparam logic [6:0] SEG_DASH = 7'b1111110;
    localparam logic [6:0] SEG_OFF  = 7'b1111111;

    always_comb begin
        if (!i_result_ready) begin
            // Hiển thị "------" chờ kết quả
            HEX0 = SEG_DASH;
            HEX1 = SEG_DASH;
            HEX2 = SEG_DASH;
            HEX3 = SEG_DASH;
            HEX4 = SEG_DASH;
            HEX5 = SEG_DASH;
        end else begin
            // Mỗi giá trị Q8.8 có dạng 0x00YY — byte thấp [7:0] chứa phần nguyên
            // (VD: 0x0057 = 87 → phần nguyên = 0x57, float = 87/256 ≈ 0.340)
            // disp0 → HEX5 = nibble cao byte thấp [7:4], HEX4 = nibble thấp [3:0]
            // disp1 → HEX3 HEX2
            // disp2 → HEX1 HEX0
            HEX5 = seg7(disp0[ 7: 4]);
            HEX4 = seg7(disp0[ 3: 0]);
            HEX3 = seg7(disp1[ 7: 4]);
            HEX2 = seg7(disp1[ 3: 0]);
            HEX1 = seg7(disp2[ 7: 4]);
            HEX0 = seg7(disp2[ 3: 0]);
        end
    end

endmodule