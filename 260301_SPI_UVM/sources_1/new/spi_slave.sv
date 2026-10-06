`timescale 1ns / 1ps

module spi_slave (
    // global signals
    input  logic       clk,
    input  logic       reset,
    // internal signals
    input  logic [7:0] tx_data,   // 전송할 데이터
    output logic       tx_ready,  // 새 tx_data 로드 가능
    output logic [7:0] rx_data,   // 수신된 데이터
    output logic       done,      // 수신 완료
    input  logic       cpol,
    input  logic       cpha,
    // external signals
    input  logic       sclk,
    input  logic       mosi,
    output logic       miso,
    input  logic       cs_n
);

    // ─── SCLK 엣지 검출 ───────────────────────────────────────────
    logic sclk_prev;
    logic sclk_rise, sclk_fall;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) sclk_prev <= 1'b0;
        else sclk_prev <= sclk;
    end

    assign sclk_rise = (~sclk_prev) & sclk;
    assign sclk_fall = sclk_prev & (~sclk);

    // ─── 샘플/시프트 엣지 결정 ───────────────────────────────────
    // CPOL=0, CPHA=0 : 상승엣지 샘플, 하강엣지 시프트
    // CPOL=0, CPHA=1 : 하강엣지 샘플, 상승엣지 시프트
    // CPOL=1, CPHA=0 : 하강엣지 샘플, 상승엣지 시프트
    // CPOL=1, CPHA=1 : 상승엣지 샘플, 하강엣지 시프트
    logic sample_edge, shift_edge;

    always_comb begin
        case ({
            cpol, cpha
        })
            2'b00: begin
                sample_edge = sclk_rise;
                shift_edge  = sclk_fall;
            end
            2'b01: begin
                sample_edge = sclk_fall;
                shift_edge  = sclk_rise;
            end
            2'b10: begin
                sample_edge = sclk_fall;
                shift_edge  = sclk_rise;
            end
            2'b11: begin
                sample_edge = sclk_rise;
                shift_edge  = sclk_fall;
            end
        endcase
    end

    // ─── 데이터 레지스터 ──────────────────────────────────────────
    logic [7:0] rx_shift_reg;
    logic [7:0] tx_shift_reg;
    logic [2:0] bit_cnt;
    logic       active;  // CS_N=0 상태
    logic       done_reg;
    logic       first_shift_done;

    assign rx_data  = rx_shift_reg;
    assign miso     = tx_shift_reg[7];  // MSB first
    assign done     = done_reg;
    assign tx_ready = ~active;
    assign active   = ~cs_n;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            rx_shift_reg     <= 8'h00;
            tx_shift_reg     <= 8'h00;
            bit_cnt          <= 3'd0;
            done_reg         <= 1'b0;
            first_shift_done <= 1'b0;
        end else begin
            done_reg <= 1'b0;  // pulse

            // CS_N 비활성 → 초기화
            if (cs_n) begin
                bit_cnt          <= 3'd0;
                tx_shift_reg     <= tx_data;  // 다음 전송 데이터 프리로드
                first_shift_done <= 1'b0;
            end else begin
                // 샘플 엣지: MOSI 수신
                if (sample_edge) begin
                    rx_shift_reg <= {rx_shift_reg[6:0], mosi};
                    if (bit_cnt == 3'd7) begin
                        done_reg <= 1'b1;
                        bit_cnt  <= 3'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 1;
                    end
                end

                // 시프트 엣지: MISO 송신
                if (shift_edge) begin
                    if (cpha && !first_shift_done) begin
                        // CPHA=1: 첫 번째 시프트 엣지는 스킵 (MSB 유지)
                        first_shift_done <= 1'b1;
                    end else begin
                        tx_shift_reg <= {tx_shift_reg[6:0], 1'b0};
                    end
                end
            end
        end
    end

endmodule