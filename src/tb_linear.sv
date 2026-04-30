`timescale 1ns/1ps

// =============================================================================
// TESTBENCH: tb_linear (Full Print Log)
// =============================================================================

module tb_linear();

    // =========================================================================
    // Clock và Reset
    // =========================================================================
    logic clk, rst_n;
    always #10 clk = ~clk;  // 50 MHz (Chu kỳ 20ns)

    localparam int CLK_FREQ_SIM  = 50_000_000;
    localparam int BAUD_RATE_SIM = 115_200;
    localparam int CLKS_PER_BIT  = CLK_FREQ_SIM / BAUD_RATE_SIM; 

    // =========================================================================
    // Kết nối nội bộ
    // =========================================================================
    logic        ctrl_start_linear, ctrl_start_attn, ctrl_start_softmax;
    logic [1:0]  ctrl_token_sel;
    logic        ctrl_linear_done, ctrl_attn_done, ctrl_softmax_done;
    logic        ctrl_z_ram_we;
    logic [3:0]  ctrl_z_ram_addr;
    logic signed [15:0] ctrl_z_ram_data;
    logic [23:0] ctrl_softmax_sum [0:2];
    logic        ctrl_rx_ram_we;
    logic [7:0]  ctrl_rx_ram_addr;
    logic signed [15:0] ctrl_rx_ram_data;

    // =========================================================================
    // INSTANCE: Hệ thống lõi & UART
    // =========================================================================
    attention_top dut (
        .clk(clk),                           .rst_n(rst_n),
        .i_start_linear(ctrl_start_linear),  .i_token_selection(ctrl_token_sel),
        .o_linear_done(ctrl_linear_done),
        .i_start_attn(ctrl_start_attn),      .o_attn_done(ctrl_attn_done),
        .i_start_softmax(ctrl_start_softmax),.o_softmax_done(ctrl_softmax_done),
        .i_rx_ram_we(ctrl_rx_ram_we),        .i_rx_ram_addr(ctrl_rx_ram_addr),
        .i_rx_ram_data(ctrl_rx_ram_data),    .o_softmax_sum(ctrl_softmax_sum),
        .i_wq_we(1'b0), .i_wq_addr(12'd0), .i_wq_data(16'd0),
        .i_wk_we(1'b0), .i_wk_addr(12'd0), .i_wk_data(16'd0),
        .i_wv_we(1'b0), .i_wv_addr(12'd0), .i_wv_data(16'd0),
        .o_z_ram_we(ctrl_z_ram_we),
        .o_z_ram_wr_addr(ctrl_z_ram_addr),   .o_z_ram_data(ctrl_z_ram_data)
    );

    logic uart_rx_line, uart_tx_line;
    logic [7:0] phy_rx_data, phy_tx_data;
    logic       phy_rx_valid, phy_tx_start, phy_tx_busy, ctrl_result_ready;
    logic [3:0] ctrl_state;

    uart_rx #(.CLK_FREQ(CLK_FREQ_SIM), .BAUD_RATE(BAUD_RATE_SIM)) u_uart_rx (
        .i_clk(clk), .i_rst_n(rst_n), .i_rx(uart_rx_line),
        .o_rx_data(phy_rx_data), .o_rx_valid(phy_rx_valid)
    );

    uart_tx #(.CLK_FREQ(CLK_FREQ_SIM), .BAUD_RATE(BAUD_RATE_SIM)) u_uart_tx (
        .i_clk(clk), .i_rst_n(rst_n), .i_tx_data(phy_tx_data),
        .i_tx_start(phy_tx_start), .o_tx(uart_tx_line), .o_tx_busy(phy_tx_busy)
    );

    uart_controller #(
        .CLK_FREQ(CLK_FREQ_SIM), .BAUD_RATE(BAUD_RATE_SIM), .SEQ_LEN(3), .D_MODEL(64)
    ) u_uart_ctrl (
        .i_clk(clk), .i_rst_n(rst_n),
        .i_rx_data(phy_rx_data), .i_rx_valid(phy_rx_valid),
        .o_tx_data(phy_tx_data), .o_tx_start(phy_tx_start), .i_tx_busy(phy_tx_busy),
        .o_rx_ram_we(ctrl_rx_ram_we), .o_rx_ram_addr(ctrl_rx_ram_addr), .o_rx_ram_data(ctrl_rx_ram_data),
        .o_start_linear(ctrl_start_linear), .o_token_sel(ctrl_token_sel), .i_linear_done(ctrl_linear_done),
        .o_start_attn(ctrl_start_attn), .i_attn_done(ctrl_attn_done),
        .o_start_softmax(ctrl_start_softmax), .i_softmax_done(ctrl_softmax_done),
        .i_y_we(ctrl_z_ram_we), .i_y_addr(ctrl_z_ram_addr), .i_y_data(ctrl_z_ram_data),
        .o_result_ready(ctrl_result_ready), .o_state_debug(ctrl_state)
    );

    // =========================================================================
    // KHỐI GIÁM SÁT (MONITOR BLOCKS) - IN KẾT QUẢ TỪNG BƯỚC
    // =========================================================================

    // 1. In kết quả Q, K, V khi ghi vào RAM (chỉ in 4 phần tử đầu mỗi token)
    always @(posedge clk) begin
        if (rst_n && dut.u_linear.wire_linear_we) begin
            if (dut.u_linear.wire_linear_wr_addr % 64 < 4)
                $display(" [MAC] Token %0d, Dim %0d | Q = %04h (%5d) | K = %04h (%5d) | V = %04h (%5d)",
                         dut.u_linear.wire_linear_wr_addr / 64, dut.u_linear.wire_linear_wr_addr % 64,
                         dut.u_linear.wire_q_res, $signed(dut.u_linear.wire_q_res),
                         dut.u_linear.wire_k_res, $signed(dut.u_linear.wire_k_res),
                         dut.u_linear.wire_v_res, $signed(dut.u_linear.wire_v_res));
        end
    end

    // 2. In kết quả Attention Score
    always @(posedge clk) begin
        if (rst_n && dut.u_linear.o_s_ram_we) begin
            $display(" [SCORE] S[%0d][%0d] = %04h | Int = %6d | Real = %8.5f",
                     dut.u_linear.o_s_ram_addr / 3, dut.u_linear.o_s_ram_addr % 3,
                     dut.u_linear.o_s_ram_data[15:0], $signed(dut.u_linear.o_s_ram_data[15:0]),
                     real'($signed(dut.u_linear.o_s_ram_data[15:0]))/256.0);
        end
    end

    // 3. In kết quả Max & Z (Subtractor)
    always @(posedge clk) begin
        if (rst_n && dut.u_softmax_max.o_z_ram_we) begin
            $display(" [SUB] Z_INTER[%0d][%0d] | Max = %04h (%5d) | Z = S - Max = %04h (%5d)",
                     dut.u_softmax_max.o_z_ram_wr_addr / 3, dut.u_softmax_max.o_z_ram_wr_addr % 3,
                     dut.u_softmax_max.current_max, $signed(dut.u_softmax_max.current_max),
                     dut.u_softmax_max.o_z_ram_data, $signed(dut.u_softmax_max.o_z_ram_data));
        end
    end

    // 4. In kết quả Exp
    always @(posedge clk) begin
        if (rst_n && dut.u_exp_sum.o_z_ram_we) begin
            $display(" [EXP] Z_FINAL[%0d][%0d] = %04h | Int = %6d | Real = %8.5f",
                     dut.u_exp_sum.o_z_ram_wr_addr / 3, dut.u_exp_sum.o_z_ram_wr_addr % 3,
                     dut.u_exp_sum.o_z_ram_data, $signed(dut.u_exp_sum.o_z_ram_data),
                     real'($signed(dut.u_exp_sum.o_z_ram_data))/256.0);
        end
    end

    // 5. In kết quả Sum (Ngay khi Pass 2 xong)
    always @(posedge clk) begin
        if (rst_n && dut.u_exp_sum.done_exp_sum) begin
            $display(" [SUM] Hang 0: %0d (Real: %f)", dut.o_softmax_sum[0], real'(dut.o_softmax_sum[0])/256.0);
            $display(" [SUM] Hang 1: %0d (Real: %f)", dut.o_softmax_sum[1], real'(dut.o_softmax_sum[1])/256.0);
            $display(" [SUM] Hang 2: %0d (Real: %f)", dut.o_softmax_sum[2], real'(dut.o_softmax_sum[2])/256.0);
        end
    end

    // 6. In kết quả Divider / Softmax (Final Y)
    always @(posedge clk) begin
        if (rst_n && dut.u_div.o_y_ram_we) begin
             $display(" [DIV] Y_RAM[%0d][%0d]   = %04h | Int = %6d | Real = %8.5f",
                      dut.u_div.o_y_ram_wr_addr / 3, dut.u_div.o_y_ram_wr_addr % 3,
                      dut.u_div.o_y_ram_data, $signed(dut.u_div.o_y_ram_data),
                      real'($signed(dut.u_div.o_y_ram_data))/256.0);
        end
    end

    // =========================================================================
    // TASKS UART
    // =========================================================================
    task automatic uart_send_byte(input logic [7:0] data);
        integer i;
        @(posedge clk);
        uart_rx_line = 1'b0;
        repeat(CLKS_PER_BIT) @(posedge clk);
        for (i = 0; i < 8; i++) begin
            uart_rx_line = data[i];
            repeat(CLKS_PER_BIT) @(posedge clk);
        end
        uart_rx_line = 1'b1;
        repeat(CLKS_PER_BIT) @(posedge clk);
    endtask

    task automatic uart_recv_byte(output logic [7:0] data);
        integer i;
        if (uart_tx_line == 1'b1) begin
            fork
                begin : wait_negedge @(negedge uart_tx_line); end
                begin : recv_timeout repeat(2_500_000) @(posedge clk); $stop; end
            join_any disable fork;
            repeat(CLKS_PER_BIT + CLKS_PER_BIT/2) @(posedge clk);
        end else begin
            repeat(CLKS_PER_BIT/2) @(posedge clk);
            repeat(CLKS_PER_BIT) @(posedge clk);
        end
        for (i = 0; i < 8; i++) begin
            data[i] = uart_tx_line;
            repeat(CLKS_PER_BIT) @(posedge clk);
        end
        repeat(CLKS_PER_BIT/2) @(posedge clk);
    endtask

    // =========================================================================
    // MÔ PHỎNG CHÍNH
    // =========================================================================
    logic signed [15:0] x_sim_data [0:191];
    logic [7:0]         uart_rx_bytes [0:17];
    logic signed [15:0] uart_rx_words [0:8];

    initial begin
        clk = 0; rst_n = 0; uart_rx_line = 1'b1;
        $readmemh("x_data_sim.txt", x_sim_data);

        #100 rst_n = 1;
        repeat(10) @(posedge clk);

        $display("\n============================================================");
        $display(" BAT DAU NHOI DU LIEU QUA UART");
        $display("============================================================");

        uart_send_byte(8'hAA);
        begin
            int w;
            for (w = 0; w < 192; w++) begin
                if (w % 64 < 4) // In 4 phần tử đầu mỗi token lúc nạp
                    $display(" [INPUT X] Nap Word %0d: %04h", w, x_sim_data[w]);
                uart_send_byte(x_sim_data[w][15:8]);
                uart_send_byte(x_sim_data[w][7:0]);
            end
        end

        $display("\n============================================================");
        $display(" BAT DAU THEO DOI PIPELINE PHAN CUNG");
        $display("============================================================");
        
        fork
            begin
                wait(ctrl_state == 4'd2); $display("\n>>> PHASE 1: LINEAR MAC <<<");
                wait(ctrl_state == 4'd4); $display("\n>>> PHASE 2: ATTENTION SCORE <<<");
                wait(ctrl_state == 4'd6); $display("\n>>> PHASE 3: SOFTMAX (Max, Sub, Exp, Sum, Div) <<<");
                wait(ctrl_result_ready == 1'b1); 
            end
            begin
                repeat(25_000_000) @(posedge clk);
                $display("\n[LOI] Timeout."); $stop;
            end
        join_any disable fork;

        $display("\n============================================================");
        $display(" DOC KET QUA QUA UART");
        $display("============================================================");
        uart_send_byte(8'h55);
        begin
            int i;
            for (i = 0; i < 18; i++) uart_recv_byte(uart_rx_bytes[i]);
            for (i = 0; i < 9; i++) begin
                uart_rx_words[i] = {uart_rx_bytes[i*2], uart_rx_bytes[i*2+1]};
                $display(" [UART RX] Y[%0d][%0d] = %04h | Int = %6d | Real = %8.5f",
                         i/3, i%3, uart_rx_words[i], uart_rx_words[i], real'(uart_rx_words[i])/256.0);
            end
        end
        #500 $stop;
    end
endmodule