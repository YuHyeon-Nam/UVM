`timescale 1ns / 1ps

// ============================================================
// dut_top_i2c.sv  ← UVM 검증용 DUT Top
//
// [제거된 모듈]
//   - i2c_send        (FPGA 보드 SW 입력 컨트롤러)
//   - i2c_ascii_send  (로직 애널라이저용 ASCII 전송)
//   - led_control     (LED 출력 - 하드웨어 전용)
//   - clk_div         (UVM TB에서 직접 clk 공급, tick은 UVM sequence가 제어)
//
// [포트 구성]
//   - Master 제어 신호 (i2c_en, i2c_start, i2c_stop, tx_data) :
//       UVM Master Driver가 직접 구동
//   - Master 상태 신호 (tx_done, tx_ready, rx_data, rx_done) :
//       UVM Master Monitor가 샘플링
//   - Slave 출력 (outputdata) :
//       UVM Slave Monitor / Scoreboard가 샘플링
//   - SCL / SDA :
//       Master ↔ Slave 공유 버스 (pullup은 TB에서 처리)
// ============================================================

module dut_top_i2c (
    // 공통 클럭/리셋
    input  logic       clk,
    input  logic       reset,

    // ── Master 제어 포트 (UVM Master Driver → DUT) ──────────
    input  logic       i2c_en,
    input  logic       i2c_start,
    input  logic       i2c_stop,
    input  logic [7:0] tx_data,

    // ── Master 상태 포트 (DUT → UVM Master Monitor) ─────────
    output logic       tx_done,
    output logic       tx_ready,
    output logic [7:0] rx_data,
    output logic       rx_done,

    // ── Slave 출력 포트 (DUT → UVM Slave Monitor) ───────────
    output logic [7:0] outputdata,

    // ── I2C 버스 (공유, TB에서 pullup 처리) ─────────────────
    output logic       scl,
    inout  logic       sda
);

    // ── I2C Master 인스턴스 ──────────────────────────────────
    i2c_master u_i2c_master (
        .clk      (clk),
        .reset    (reset),
        .i2c_en   (i2c_en),
        .i2c_start(i2c_start),
        .i2c_stop (i2c_stop),
        .tx_data  (tx_data),
        .tx_done  (tx_done),
        .tx_ready (tx_ready),
        .rx_data  (rx_data),
        .rx_done  (rx_done),
        .scl      (scl),
        .sda      (sda)
    );

    // ── I2C Slave 인스턴스 ───────────────────────────────────
    i2c_slave u_i2c_slave (
        .clk       (clk),
        .reset     (reset),
        .scl       (scl),
        .sda       (sda),
        .outputdata(outputdata)
    );

endmodule