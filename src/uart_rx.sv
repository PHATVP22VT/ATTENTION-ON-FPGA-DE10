// =============================================================================
// MODULE: uart_rx
// Chức năng: Nhận dữ liệu nối tiếp UART, xuất từng byte song song
//
// Giao thức UART: 1 start bit (LOW) + 8 data bits (LSB trước) + 1 stop bit (HIGH)
// Không parity, không flow control (8N1)
//
// Tham số:
//   CLK_FREQ  : Tần số clock hệ thống (Hz), mặc định 50 MHz (DE10)
//   BAUD_RATE : Tốc độ truyền (bps),       mặc định 115200
//
// Nguyên lý hoạt động:
//   - CLKS_PER_BIT = CLK_FREQ / BAUD_RATE = 50_000_000 / 115_200 ≈ 434
//   - Module lấy mẫu tín hiệu rx ở giữa mỗi bit (clock thứ CLKS_PER_BIT/2)
//     để tránh vùng chuyển tiếp (glitch) ở biên bit
//   - Khi nhận đủ 8 bit: xuất rx_data[7:0] và bật rx_valid = 1 đúng 1 clock
//
// Sử dụng:
//   Kết nối i_rx với chân UART_RXD của DE10 (qua FT232R)
//   Đọc o_rx_data khi o_rx_valid = 1
// =============================================================================
module uart_rx #(
    parameter int CLK_FREQ  = 50_000_000,
    parameter int BAUD_RATE = 115_200
)(
    input  logic       i_clk,
    input  logic       i_rst_n,
    input  logic       i_rx,        // Chân RX nối từ FT232R
    output logic [7:0] o_rx_data,   // Byte dữ liệu nhận được
    output logic       o_rx_valid   // Pulse 1 clock = byte sẵn sàng
);

    // Số clock tương ứng với 1 bit UART
    localparam int CLKS_PER_BIT  = CLK_FREQ / BAUD_RATE;      // ≈ 434
    // Điểm lấy mẫu = giữa bit = CLKS_PER_BIT / 2             // ≈ 217
    localparam int SAMPLE_POINT  = CLKS_PER_BIT / 2;

    typedef enum logic [2:0] {
        IDLE,       // Chờ start bit (đường HIGH → LOW)
        START,      // Xác nhận start bit ở giữa
        DATA,       // Nhận 8 bit dữ liệu
        STOP,       // Chờ stop bit
        DONE        // Xuất dữ liệu, bật valid
    } state_t;
    state_t state;

    logic [15:0] clk_cnt;     // Đếm clock trong 1 bit (cần đủ rộng cho 434)
    logic [2:0]  bit_idx;     // Đếm bit đang nhận (0..7)
    logic [7:0]  rx_shift;    // Thanh ghi dịch nhận bit
    logic        rx_sync_0;   // Đồng bộ hóa 2 tầng tránh metastability
    logic        rx_sync_1;

    // ── Đồng bộ hóa tín hiệu RX vào clock domain FPGA ──
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            rx_sync_0 <= 1'b1;
            rx_sync_1 <= 1'b1;
        end else begin
            rx_sync_0 <= i_rx;
            rx_sync_1 <= rx_sync_0;
        end
    end

    // ── FSM nhận UART ──
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state      <= IDLE;
            clk_cnt    <= '0;
            bit_idx    <= '0;
            rx_shift   <= '0;
            o_rx_data  <= '0;
            o_rx_valid <= 1'b0;
        end else begin
            o_rx_valid <= 1'b0; // Mặc định: tắt valid

            case (state)

                // ── Chờ cạnh xuống (start bit) ──
                IDLE: begin
                    clk_cnt <= '0;
                    bit_idx <= '0;
                    if (rx_sync_1 == 1'b0)   // Phát hiện LOW = start bit
                        state <= START;
                end

                // ── Xác nhận start bit tại điểm giữa ──
                // Nếu vẫn LOW → thật sự là start bit, bắt đầu nhận data
                // Nếu HIGH → là nhiễu, quay về IDLE
                START: begin
                    if (clk_cnt == SAMPLE_POINT - 1) begin
                        clk_cnt <= '0;
                        if (rx_sync_1 == 1'b0)
                            state <= DATA;
                        else
                            state <= IDLE; // Nhiễu → bỏ qua
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                // ── Nhận 8 bit dữ liệu, mỗi bit lấy mẫu tại giữa ──
                DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt            <= '0;
                        // Dịch phải, nhét bit mới vào MSB → sau 8 bit tự xếp đúng thứ tự
                        rx_shift           <= {rx_sync_1, rx_shift[7:1]};
                        if (bit_idx == 3'd7) begin
                            bit_idx <= '0;
                            state   <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                // ── Chờ stop bit (phải là HIGH) ──
                STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= '0;
                        state   <= DONE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                // ── Xuất dữ liệu: bật valid đúng 1 clock ──
                DONE: begin
                    o_rx_data  <= rx_shift;
                    o_rx_valid <= 1'b1;
                    state      <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule