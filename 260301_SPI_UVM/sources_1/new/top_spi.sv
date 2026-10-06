`timescale 1ns / 1ps

// ============================================================
// top_spi : Master(ascii_send or sw_data_send) + Slave 통합
// mode = 0 : sw_data 기반 A/B/C/D 전송 (sw_data_send)
// mode = 1 : ASCII '0'~'z' 순회 전송  (ascii_send)
// cpol, cpha : SPI 모드 선택 (sw 등으로 연결)
// ============================================================
module top_spi (
    input  logic       clk,
    input  logic       reset,
    input  logic       mode,       // 0: sw_data 모드, 1: ascii 순회 모드
    input  logic [7:0] sw,         // sw[0]: send trigger, sw[7:4]: data_sel (mode=0)
    input  logic       cpol,
    input  logic       cpha,
    output logic       sclk,
    output logic       mosi,
    output logic       miso,       // 슬레이브 → 마스터 (관찰용 출력)
    output logic [7:0] master_rx,  // 마스터 수신 데이터 (관찰용)
    output logic [7:0] slave_rx,   // 슬레이브 수신 데이터 (관찰용)
    output logic       slave_done  // 슬레이브 수신 완료 펄스
);

    // ── 내부 신호 ──────────────────────────────────────────
    logic       tick;
    logic       mosi_w, miso_w;
    logic [1:0] cs_n_w;

    // ascii_send 출력
    logic       a_start;
    logic [7:0] a_tx_data;

    // sw_data_send 출력
    logic       s_start;
    logic [7:0] s_tx_data;

    // mux → spi_master 입력
    logic       start;
    logic [7:0] tx_data;
    logic       tx_ready;

    assign start   = mode ? a_start   : s_start;
    assign tx_data = mode ? a_tx_data : s_tx_data;

    // mosi/miso 내부 wire 연결
    assign mosi = mosi_w;
    assign miso = miso_w;

    // ── 서브모듈 ────────────────────────────────────────────

    clk_div u_clk_div (
        .clk  (clk),
        .reset(reset),
        .tick (tick)
    );

    // ASCII '0'~'z' 순회 송신기
    ascii_send u_ascii_send (
        .clk     (clk),
        .reset   (reset),
        .sw      (sw[0]),
        .tick    (tick),
        .start   (a_start),
        .tx_data (a_tx_data),
        .tx_ready(tx_ready)
    );

    // sw_data 기반 A/B/C/D 선택 송신기
    spi_sw_send u_sw_send (
        .clk     (clk),
        .reset   (reset),
        .sw      (sw[0]),
        .sw_data (sw[7:4]),
        .tick    (tick),
        .start   (s_start),
        .tx_data (s_tx_data),
        .tx_ready(tx_ready)
    );

    // SPI 마스터
    spi_master u_spi_master (
        .clk       (clk),
        .reset     (reset),
        .slave_sel (2'b01),    // slave 0 선택 고정
        .start     (start),
        .tx_data   (tx_data),
        .tx_ready  (tx_ready),
        .rx_data   (master_rx),
        .done      (),
        .cpol      (cpol),
        .cpha      (cpha),
        .sclk      (sclk),
        .mosi      (mosi_w),
        .miso      (miso_w),
        .cs_n      (cs_n_w)
    );

    // SPI 슬레이브 (slave 0)
    spi_slave u_spi_slave (
        .clk     (clk),
        .reset   (reset),
        .tx_data (8'h00),      // 슬레이브 → 마스터 송신 데이터 (필요시 연결)
        .tx_ready(),
        .rx_data (slave_rx),
        .done    (slave_done),
        .cpol    (cpol),
        .cpha    (cpha),
        .sclk    (sclk),
        .mosi    (mosi_w),
        .miso    (miso_w),
        .cs_n    (cs_n_w[0])
    );

endmodule

// ============================================================
// spi_sw_send : sw_data 기반 A/B/C/D 선택 송신기
// i2c_send 구조와 대칭되도록 설계
// ============================================================
module spi_sw_send (
    input  logic       clk,
    input  logic       reset,
    input  logic       sw,         // sw[0]: send trigger
    input  logic [3:0] sw_data,    // sw[7:4]: A/B/C/D 선택
    input  logic       tick,
    output logic       start,
    output logic [7:0] tx_data,
    input  logic       tx_ready
);
    typedef enum {
        IDLE,
        TX_WAIT,
        TX_DATA
    } state_t;

    state_t state, state_next;
    logic [7:0] tx_data_reg, tx_data_next;
    logic [7:0] selected_data;

    assign tx_data = tx_data_reg;

    // sw_data → ASCII 문자 선택
    always_comb begin
        case (sw_data)
            4'b0001: selected_data = "A";
            4'b0010: selected_data = "B";
            4'b0100: selected_data = "C";
            4'b1000: selected_data = "D";
            default: selected_data = 8'h00;
        endcase
    end

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            state       <= IDLE;
            tx_data_reg <= 8'h00;
        end else begin
            state       <= state_next;
            tx_data_reg <= tx_data_next;
        end
    end

    always_comb begin
        state_next   = state;
        tx_data_next = tx_data_reg;
        start        = 1'b0;

        case (state)
            IDLE: begin
                if (sw) state_next = TX_WAIT;
            end
            TX_WAIT: begin
                if (!sw) begin
                    state_next = IDLE;
                end else if (tick & tx_ready) begin
                    tx_data_next = selected_data;
                    state_next   = TX_DATA;
                end
            end
            TX_DATA: begin
                start      = 1'b1;
                state_next = TX_WAIT;  // 다음 tick 대기
            end
        endcase
    end
endmodule


module clk_div (
    input  logic clk,
    input  logic reset,
    output logic tick
);
    logic [$clog2(10_000_000)-1:0] div_counter;

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            div_counter <= 0;
            tick <= 1'b0;
        end else begin
            if (div_counter == 10_000_000 - 1) begin
                div_counter <= 0;
                tick <= 1'b1;
            end else begin
                div_counter <= div_counter + 1;
                tick <= 1'b0;
            end
        end
    end
endmodule

module ascii_send (
    input logic clk,
    input logic reset,
    input logic sw,
    input logic tick,
    output logic start,
    output logic [7:0] tx_data,
    input logic tx_ready
);
    typedef enum {
        IDLE,
        TX_WAIT,
        TX_ASCII
    } state_t;

    state_t state, state_next;
    logic [7:0] tx_data_reg, tx_data_next;
    logic [7:0] ascii_reg, ascii_next;

    assign tx_data = tx_data_reg;

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            state       <= IDLE;
            tx_data_reg <= 0;
            ascii_reg   <= 8'h30;
        end else begin
            state       <= state_next;
            tx_data_reg <= tx_data_next;
            ascii_reg   <= ascii_next;
        end
    end

    always_comb begin
        state_next   = state;
        tx_data_next = tx_data_reg;
        ascii_next   = ascii_reg;
        start        = 1'b0;
        case (state)
            IDLE: begin
                start = 1'b0;
                ascii_next = 8'h30;
                if (sw) begin
                    state_next = TX_WAIT;
                end
            end
            TX_WAIT: begin
                start = 1'b0;
                if (tick & tx_ready) begin
                    state_next = TX_ASCII;
                end
                if (sw==0) begin
                    state_next = IDLE;
                end
            end
            TX_ASCII: begin
                start = 1'b1;
                tx_data_next = ascii_reg;
                state_next = TX_WAIT;
                if (ascii_reg == 8'h7a) begin
                    ascii_next = 8'h30;
                end else begin
                    ascii_next = ascii_reg + 1;
                end
            end
        endcase
    end
endmodule