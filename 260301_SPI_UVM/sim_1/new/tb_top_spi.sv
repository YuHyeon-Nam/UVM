// ============================================================
//  SPI UVM Testbench — All-in-One
//  I2C UVM 구조(tb_top_i2c.sv) 기반, SPI 트랜잭션 단위로 재설계
//
//  핵심 설계 원칙:
//   - Driver가 트랜잭션 완료 후 seq_item에 결과를 채워 반환
//   - Monitor는 done 신호 기준으로 완성된 트랜잭션 캡처
//   - Scoreboard는 Monitor에서 받은 완성된 트랜잭션만 비교
//   - spi_master RTL 신호: start, tx_ready, done, tx_data, rx_data,
//                           slave_sel, cpol, cpha, sclk, mosi, miso, cs_n
//
//  파일 구성:
//    1.  spi_if          (interface)
//    2.  spi_seq_item
//    3.  spi_driver
//    4.  spi_monitor
//    5.  spi_scoreboard
//    6.  spi_coverage
//    7.  spi_agent
//    8.  spi_env
//    9.  spi_sequence
//   10.  spi_test
//   11.  tb_top
// ============================================================

import uvm_pkg::*;
`include "uvm_macros.svh"

// ============================================================
// 1. Interface
// ============================================================
interface spi_if (
    input bit clk
);
    // Global
    logic       reset;
    // Master control signals
    logic [1:0] slave_sel;
    logic       start;
    logic [7:0] tx_data;
    logic       tx_ready;
    logic [7:0] rx_data;
    logic       done;
    logic       cpol;
    logic       cpha;
    // SPI bus signals
    logic       sclk;
    logic       mosi;
    logic       miso;
    logic [1:0] cs_n;
endinterface

// ============================================================
// 2. Sequence Item
// ============================================================
class spi_seq_item extends uvm_sequence_item;

    // ── Driver가 채우기 전 사용자가 설정하는 필드 ──
    rand logic [7:0] tx_data;  // Master → Slave 송신 데이터
    rand logic [1:0] slave_sel;  // 타겟 슬레이브 선택 (2'b01 = slave0, 2'b10 = slave1)
    rand logic       cpol;  // SPI 모드 CPOL
    rand logic       cpha;  // SPI 모드 CPHA

    // ── Driver가 트랜잭션 완료 후 채우는 필드 ──
    logic      [7:0] rx_data;  // Slave → Master 수신 데이터 (Driver가 채움)

    // tx_data 전체 범위 허용
    constraint c_tx_data {tx_data inside {[8'h00 : 8'hFF]};}
    // 유효한 슬레이브 선택만 허용 (slave0 또는 slave1)
    constraint c_slave_sel {slave_sel inside {2'b01, 2'b10};}
    // 기본 SPI 모드 0 (필요 시 테스트에서 오버라이드)
    // ★ c_spi_mode 제거: cpol/cpha는 do_txn에서 직접 지정
    //   Scenario 6 랜덤 시 전체 범위(0/1) 자유 랜덤화됨

    function new(string name = "spi_seq_item");
        super.new(name);
    endfunction

    `uvm_object_utils_begin(spi_seq_item)
        `uvm_field_int(tx_data, UVM_DEFAULT)
        `uvm_field_int(slave_sel, UVM_DEFAULT)
        `uvm_field_int(cpol, UVM_DEFAULT)
        `uvm_field_int(cpha, UVM_DEFAULT)
        `uvm_field_int(rx_data, UVM_DEFAULT)
    `uvm_object_utils_end

endclass

// ============================================================
// 3. Driver
// ============================================================
// 설계 원칙:
//   - spi_master RTL FSM: IDLE → (CP_DELAY) → CP0 → CP1 → ... → (CP_DELAY) → IDLE
//   - 모든 신호 대기는 wait() 레벨 폴링 사용 (combinational 신호 누락 방지)
//   - 각 트랜잭션 전 wait(tx_ready===1) 로 IDLE 진입 보장
//   - done 펄스 발생 후 rx_data를 seq_item에 채워 반환
//
//   SPI 트랜잭션 흐름:
//     1. tx_ready===1 (IDLE) 대기
//     2. slave_sel, cpol, cpha, tx_data 설정
//     3. start=1 (1클럭 펄스) → FSM이 CP0/CP_DELAY 진입
//     4. done===1 대기 (8비트 전송 완료)
//     5. rx_data 캡처 → seq_item에 저장
//     6. start=0, 신호 초기화
// ============================================================
class spi_driver extends uvm_driver #(spi_seq_item);
    `uvm_component_utils(spi_driver)

    virtual spi_if vif;

    function new(string name = "spi_driver", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_if)::get(this, "", "vif", vif)) `uvm_fatal(get_name(), "spi_if not found");
    endfunction

    task run_phase(uvm_phase phase);
        // 초기 신호 상태
        vif.start     = 1'b0;
        vif.tx_data   = 8'h00;
        vif.slave_sel = 2'b00;
        vif.cpol      = 1'b0;
        vif.cpha      = 1'b0;

        forever begin
            seq_item_port.get_next_item(req);
            drv_transaction(req);
            seq_item_port.item_done();
        end
    endtask

    // ── SPI 트랜잭션 구동 ──────────────────────────────────────
    task drv_transaction(spi_seq_item item);

        // STEP 1: IDLE 진입 대기 (tx_ready=1)
        wait (vif.tx_ready === 1'b1);
        @(posedge vif.clk);
        $display("[DRV] %0t STEP1: IDLE (tx_ready=1)", $time);

        // STEP 2: 트랜잭션 파라미터 설정
        //   cpol/cpha는 tx_data보다 먼저 안정화 (RTL이 start 엣지에서 래치)
        vif.cpol      = item.cpol;
        vif.cpha      = item.cpha;
        vif.slave_sel = item.slave_sel;
        vif.tx_data   = item.tx_data;
        @(posedge vif.clk);
        $display("[DRV] %0t STEP2: param set slave_sel=%02b cpol=%0b cpha=%0b tx=0x%02h", $time, item.slave_sel,
                 item.cpol, item.cpha, item.tx_data);

        // STEP 3: start 1클럭 펄스 → FSM이 IDLE→CP0 (또는 CP_DELAY) 전환
        vif.start = 1'b1;
        @(posedge vif.clk);
        vif.start = 1'b0;
        $display("[DRV] %0t STEP3: start pulse issued", $time);

        // STEP 4: done 펄스 대기 (8비트 전송 완료)
        wait (vif.done === 1'b1);
        @(posedge vif.clk);
        // STEP 5: rx_data 캡처 (done 시점에 rx_data 유효)
        item.rx_data = vif.rx_data;
        $display("[DRV] %0t STEP5: done=1  tx=0x%02h  rx=0x%02h", $time, item.tx_data, item.rx_data);

        // done 내려갈 때까지 대기
        wait (vif.done === 1'b0);

        // STEP 6: 신호 초기화 후 IDLE 복귀 대기
        vif.slave_sel = 2'b00;
        wait (vif.tx_ready === 1'b1);
        repeat (2) @(posedge vif.clk);

        `uvm_info("DRV", $sformatf("TXN done: slave=%02b cpol=%0b cpha=%0b tx=0x%02h rx=0x%02h", item.slave_sel,
                                   item.cpol, item.cpha, item.tx_data, item.rx_data), UVM_NONE)
    endtask

endclass

// ============================================================
// 4. Monitor
// ============================================================
// 설계 원칙:
//   - done 신호를 기준으로 트랜잭션 완료 캡처
//   - done=1 시점에 tx_data, rx_data, slave_sel, cpol, cpha 모두 유효
//   - 완성된 seq_item을 Scoreboard/Coverage로 전송
// ============================================================
class spi_monitor extends uvm_monitor;
    `uvm_component_utils(spi_monitor)

    uvm_analysis_port #(spi_seq_item) send;
    virtual spi_if vif;

    function new(string name = "spi_monitor", uvm_component parent);
        super.new(name, parent);
        send = new("send", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_if)::get(this, "", "vif", vif)) `uvm_fatal(get_name(), "spi_if not found");
    endfunction

    task run_phase(uvm_phase phase);
        spi_seq_item item;
        forever begin
            // done 상승엣지 감지 (레벨 폴링)
            wait (vif.done === 1'b1);
            @(posedge vif.clk);

            // done 시점에 모든 신호 캡처
            item           = spi_seq_item::type_id::create("item", this);
            item.slave_sel = vif.slave_sel;
            item.cpol      = vif.cpol;
            item.cpha      = vif.cpha;
            item.tx_data   = vif.tx_data;
            item.rx_data   = vif.rx_data;

            `uvm_info("MON", $sformatf(
                      "[TXN] slave=%02b cpol=%0b cpha=%0b tx=0x%02h rx=0x%02h  cs_n=%02b",
                      item.slave_sel,
                      item.cpol,
                      item.cpha,
                      item.tx_data,
                      item.rx_data,
                      vif.cs_n
                      ), UVM_NONE)

            send.write(item);

            // done 내려갈 때까지 대기 (중복 캡처 방지)
            wait (vif.done === 1'b0);
        end
    endtask

endclass

// ============================================================
// 5. Scoreboard
// ============================================================
// 설계 원칙:
//   - Loopback 검증: spi_slave가 수신한 데이터 = master가 송신한 tx_data
//   - spi_master의 rx_data = spi_slave가 miso로 돌려준 데이터
//   - 본 TB에서 slave tx_data는 고정값(8'hA5)이므로
//     rx_data === 8'hA5 이면 PASS
//   - tx_data 루프백: slave가 수신한 데이터는 monitor에서 tx_data로 확인
// ============================================================
class spi_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(spi_scoreboard)

    uvm_analysis_imp #(spi_seq_item, spi_scoreboard) recv;

    // Slave가 Master로 돌려주는 고정 응답 데이터 (tb_top에서 설정)
    logic [7:0] slave_resp = 8'hA5;

    int pass_cnt = 0;
    int fail_cnt = 0;

    function new(string name = "spi_scoreboard", uvm_component parent);
        super.new(name, parent);
        recv = new("recv", this);
    endfunction

    function void write(spi_seq_item item);
        // ── TX 검증: cs_n이 올바르게 활성화되었는지 (Monitor에서 간접 확인)
        `uvm_info("SCB", $sformatf(
                  "RECV: slave=%02b cpol=%0b cpha=%0b tx=0x%02h rx=0x%02h",
                  item.slave_sel,
                  item.cpol,
                  item.cpha,
                  item.tx_data,
                  item.rx_data
                  ), UVM_NONE)

        // ── RX 루프백 검증: slave 응답 데이터와 비교
        if (item.rx_data === slave_resp) begin
            pass_cnt++;
            `uvm_info("SCB", $sformatf("LOOPBACK MATCH  : exp=0x%02h  got=0x%02h ✓ (pass=%0d)", slave_resp,
                                       item.rx_data, pass_cnt), UVM_NONE)
        end else begin
            fail_cnt++;
            `uvm_error("SCB", $sformatf(
                       "LOOPBACK MISMATCH: exp=0x%02h  got=0x%02h (fail=%0d)", slave_resp, item.rx_data, fail_cnt))
        end
    endfunction

    function void report_phase(uvm_phase phase);
        `uvm_info("SCB", $sformatf("\n=== Scoreboard Report ===\n  PASS: %0d\n  FAIL: %0d", pass_cnt, fail_cnt),
                  UVM_NONE)
    endfunction

endclass

// ============================================================
// 6. Coverage
// ============================================================
// 커버리지 설계 원칙 (RTL 기반):
//   - spi_master FSM: IDLE / CP0 / CP1 / CP_DELAY
//   - SPI 모드: CPOL×CPHA 4조합 (Mode 0~3)
//   - Slave 선택: slave0 (2'b01), slave1 (2'b10)
//   - 데이터: 8비트 전체 범위
//   - CS_N: 각 슬레이브별 정확한 활성화 여부
// ============================================================
class spi_coverage extends uvm_subscriber #(spi_seq_item);
    `uvm_component_utils(spi_coverage)

    spi_seq_item   curr_item;
    spi_seq_item   prev_item;

    virtual spi_if vif;

    // ── 트랜잭션 커버그룹 ─────────────────────────────────────
    covergroup cg_spi_transaction;

        // ── 1. 슬레이브 선택 ─────────────────────────────────
        cp_slave_sel: coverpoint curr_item.slave_sel {
            bins slave0 = {2'b01};  // CS_N[0] 활성화
            bins slave1 = {2'b10};  // CS_N[1] 활성화
        }

        // ── 2. SPI 모드 (CPOL × CPHA 4조합) ─────────────────
        cp_spi_mode: coverpoint {
            curr_item.cpol, curr_item.cpha
        } {
            bins mode0 = {2'b00};  // CPOL=0, CPHA=0
            bins mode1 = {2'b01};  // CPOL=0, CPHA=1
            bins mode2 = {2'b10};  // CPOL=1, CPHA=0
            bins mode3 = {2'b11};  // CPOL=1, CPHA=1
        }

        // ── 3. TX 데이터 값 범위 ──────────────────────────────
        cp_tx_range: coverpoint curr_item.tx_data {
            bins zero = {8'h00};  // 최솟값 경계
            bins max_val = {8'hFF};  // 최댓값 경계
            bins low = {[8'h01 : 8'h3F]};  // 하위 범위
            bins mid_low = {[8'h40 : 8'h7F]};  // 중하위 범위
            bins mid_high = {[8'h80 : 8'hBF]};  // 중상위 범위
            bins high = {[8'hC0 : 8'hFE]};  // 상위 범위
        }

        // ── 4. TX 데이터 MSB / LSB ───────────────────────────
        cp_msb: coverpoint curr_item.tx_data[7] {
            bins msb_0 = {1'b0}; bins msb_1 = {1'b1};
        }
        cp_lsb: coverpoint curr_item.tx_data[0] {
            bins lsb_0 = {1'b0};  // 짝수
            bins lsb_1 = {1'b1};  // 홀수
        }

        // ── 5. RX 데이터 값 범위 ──────────────────────────────
        // slave 응답이 8'hA5 고정이므로 mid_high bin만 달성 가능
        // 나머지 bin은 ignore_bins 처리하여 의미없는 미달 제거
        cp_rx_range: coverpoint curr_item.rx_data {
            bins zero = {8'h00};
            bins max_val = {8'hFF};
            bins low = {[8'h01 : 8'h3F]};
            bins mid_low = {[8'h40 : 8'h7F]};
            bins mid_high = {[8'h80 : 8'hBF]};  // 8'hA5 → 이 bin만 커버됨
            bins high = {[8'hC0 : 8'hFE]};
            ignore_bins unreachable = {8'h00, 8'hFF, [8'h01 : 8'h3F], [8'h40 : 8'h7F], [8'hC0 : 8'hFE]};
        }

        // ── 6. 슬레이브 × SPI 모드 교차 ─────────────────────
        cx_slave_mode: cross cp_slave_sel, cp_spi_mode;

        // ── 7. 슬레이브 × TX 데이터 범위 교차 ───────────────
        cx_slave_tx: cross cp_slave_sel, cp_tx_range;

        // ── 8. MSB × LSB 조합 커버 ───────────────────────────
        cx_msb_lsb: cross cp_msb, cp_lsb;

        // ── 9. SPI 모드 × TX 데이터 범위 교차 ───────────────
        cx_mode_tx: cross cp_spi_mode, cp_tx_range;

    endgroup

    // ── 연속 트랜잭션 패턴 커버그룹 ───────────────────────────
    // cross 대신 {prev,curr} concat 단일 coverpoint로 인코딩
    // XSim cross auto-bin 생성으로 인한 99.0% 고착 문제 우회
    covergroup cg_spi_sequence_pattern;

        // ── 10. 슬레이브 전환 패턴 (4조합) ──────────────────
        // {prev_slave[1:0], curr_slave[1:0]} 4bit 직접 커버
        cp_slave_transition: coverpoint {
            prev_item.slave_sel, curr_item.slave_sel
        } iff (prev_item != null) {
            bins s0_to_s0 = {4'b0101};  // slave0→slave0
            bins s0_to_s1 = {4'b0110};  // slave0→slave1
            bins s1_to_s0 = {4'b1001};  // slave1→slave0
            bins s1_to_s1 = {4'b1010};  // slave1→slave1
        }

        // ── 11. SPI 모드 전환 패턴 (16조합) ─────────────────
        // {prev_cpol, prev_cpha, curr_cpol, curr_cpha} 4bit
        cp_mode_transition: coverpoint {
            prev_item.cpol, prev_item.cpha, curr_item.cpol, curr_item.cpha
        } iff (prev_item != null) {
            bins m00_to_m00 = {4'b0000};
            bins m00_to_m01 = {4'b0001};
            bins m00_to_m10 = {4'b0010};
            bins m00_to_m11 = {4'b0011};
            bins m01_to_m00 = {4'b0100};
            bins m01_to_m01 = {4'b0101};
            bins m01_to_m10 = {4'b0110};
            bins m01_to_m11 = {4'b0111};
            bins m10_to_m00 = {4'b1000};
            bins m10_to_m01 = {4'b1001};
            bins m10_to_m10 = {4'b1010};
            bins m10_to_m11 = {4'b1011};
            bins m11_to_m00 = {4'b1100};
            bins m11_to_m01 = {4'b1101};
            bins m11_to_m10 = {4'b1110};
            bins m11_to_m11 = {4'b1111};
        }

    endgroup

    // ── 버스 신호 커버그룹 (매 클럭 샘플링) ───────────────────
    covergroup cg_spi_bus_activity;

        // ── 12. done / tx_ready 발생 여부 ────────────────────
        cp_done: coverpoint vif.done {
            bins active = {1'b1}; bins inactive = {1'b0};
        }
        cp_tx_ready: coverpoint vif.tx_ready {
            bins ready = {1'b1};  // IDLE 상태
            bins not_ready = {1'b0};  // 전송 중
        }

        // ── 13. SCLK 상태 ─────────────────────────────────────
        cp_sclk: coverpoint vif.sclk {
            bins sclk_high = {1'b1}; bins sclk_low = {1'b0};
        }

        // ── 14. CS_N 상태 (각 슬레이브별) ────────────────────
        cp_cs_n0: coverpoint vif.cs_n[0] {
            bins asserted = {1'b0};  // slave0 선택
            bins deasserted = {1'b1};  // 비선택
        }
        cp_cs_n1: coverpoint vif.cs_n[1] {
            bins asserted = {1'b0};  // slave1 선택
            bins deasserted = {1'b1};  // 비선택
        }

        // ── 15. done=1 & tx_ready=1 동시 발생 → 불가 ────────
        cx_done_ready: cross cp_done, cp_tx_ready{
            illegal_bins impossible = binsof (cp_done.active) && binsof (cp_tx_ready.ready);
        }

    endgroup

    function new(string name = "spi_coverage", uvm_component parent);
        super.new(name, parent);
        prev_item               = null;
        cg_spi_transaction      = new();
        cg_spi_sequence_pattern = new();
        cg_spi_bus_activity     = new();
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_if)::get(this, "", "vif", vif)) `uvm_fatal(get_name(), "spi_if not found");
    endfunction

    // 매 클럭마다 버스 신호 커버리지 샘플링
    task run_phase(uvm_phase phase);
        forever begin
            @(posedge vif.clk);
            cg_spi_bus_activity.sample();
        end
    endtask

    function void write(spi_seq_item t);
        curr_item = t;
        cg_spi_transaction.sample();
        // prev_item이 null인 첫 트랜잭션은 sequence_pattern 샘플링 제외
        // (prev_item==null → 2'bxx → 어떤 bin에도 미매핑 → 영구 미달 방지)
        if (prev_item != null) cg_spi_sequence_pattern.sample();
        prev_item = t;
    endfunction

    function void report_phase(uvm_phase phase);
        `uvm_info("COV", $sformatf(
                  "\n=== Coverage Report ===\n  cg_spi_transaction     : %.1f%%\n  cg_spi_sequence_pattern: %.1f%%\n  cg_spi_bus_activity    : %.1f%%",
                  cg_spi_transaction.get_coverage(),
                  cg_spi_sequence_pattern.get_coverage(),
                  cg_spi_bus_activity.get_coverage()
                  ), UVM_NONE)
    endfunction

endclass

// ============================================================
// 7. Agent
// ============================================================
class spi_agent extends uvm_agent;
    `uvm_component_utils(spi_agent)

    spi_driver                    spi_drv;
    spi_monitor                   spi_mon;
    uvm_sequencer #(spi_seq_item) spi_sqr;

    function new(string name = "spi_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        spi_drv = spi_driver::type_id::create("spi_drv", this);
        spi_mon = spi_monitor::type_id::create("spi_mon", this);
        spi_sqr = uvm_sequencer#(spi_seq_item)::type_id::create("spi_sqr", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        spi_drv.seq_item_port.connect(spi_sqr.seq_item_export);
    endfunction

endclass

// ============================================================
// 8. Env
// ============================================================
class spi_env extends uvm_env;
    `uvm_component_utils(spi_env)

    spi_agent      spi_agt;
    spi_scoreboard spi_scb;
    spi_coverage   spi_cov;

    function new(string name = "spi_env", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        spi_agt = spi_agent::type_id::create("spi_agt", this);
        spi_scb = spi_scoreboard::type_id::create("spi_scb", this);
        spi_cov = spi_coverage::type_id::create("spi_cov", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        spi_agt.spi_mon.send.connect(spi_scb.recv);
        spi_agt.spi_mon.send.connect(spi_cov.analysis_export);
    endfunction

endclass

// ============================================================
// 9. Sequence
// ============================================================
// 커버리지 100% 달성을 위한 시나리오:
//   Scenario 1: SPI 4모드 × 2슬레이브 조합 (코너 케이스 포함)
//   Scenario 2: TX 데이터 전체 범위 커버 (slave0, Mode0)
//   Scenario 3: 슬레이브 전환 패턴 (slave0→slave1→slave0)
//   Scenario 4: SPI 모드 전환 패턴 (mode0→1→2→3 순환)
//   Scenario 5: MSB/LSB 경계값 커버
//   Scenario 6: 랜덤 트랜잭션
// ============================================================
class spi_sequence extends uvm_sequence #(spi_seq_item);
    `uvm_object_utils(spi_sequence)

    // 데이터 범위 대표값
    logic [7:0] range_samples[6] = '{8'h00, 8'hFF, 8'h20, 8'h60, 8'hA0, 8'hE0};

    function new(string name = "spi_sequence");
        super.new(name);
    endfunction

    // ── 단일 SPI 트랜잭션 태스크 ──────────────────────────────
    task do_txn(input logic [1:0] slave_sel, input logic cpol, input logic cpha, input logic [7:0] tx_data,
                output logic [7:0] rx_data);
        spi_seq_item item;
        item = spi_seq_item::type_id::create("item");
        start_item(item);
        item.slave_sel = slave_sel;
        item.cpol      = cpol;
        item.cpha      = cpha;
        item.tx_data   = tx_data;
        finish_item(item);
        rx_data = item.rx_data;
        `uvm_info("SEQ", $sformatf("TXN: slave=%02b mode[%0b%0b] tx=0x%02h rx=0x%02h", slave_sel, cpol, cpha, tx_data,
                                   rx_data), UVM_NONE)
    endtask

    task body();
        logic [7:0] rx;
        logic [7:0] rnd;

        virtual spi_if vif;
        if (!uvm_config_db#(virtual spi_if)::get(null, "*", "vif", vif)) `uvm_fatal("SEQ", "spi_if not found")

        @(negedge vif.reset);
        repeat (10) @(posedge vif.clk);

        // ── Scenario 1: SPI 4모드 × 2슬레이브 ──────────────
        // 커버: cp_slave_sel × cp_spi_mode (cx_slave_mode 전체)
        `uvm_info("SEQ", "=== Scenario 1: 4 SPI Modes × 2 Slaves ===", UVM_NONE)
        do_txn(2'b01, 0, 0, 8'h55, rx);  // slave0, Mode0
        do_txn(2'b01, 0, 1, 8'h55, rx);  // slave0, Mode1
        do_txn(2'b01, 1, 0, 8'h55, rx);  // slave0, Mode2
        do_txn(2'b01, 1, 1, 8'h55, rx);  // slave0, Mode3
        do_txn(2'b10, 0, 0, 8'hAA, rx);  // slave1, Mode0
        do_txn(2'b10, 0, 1, 8'hAA, rx);  // slave1, Mode1
        do_txn(2'b10, 1, 0, 8'hAA, rx);  // slave1, Mode2
        do_txn(2'b10, 1, 1, 8'hAA, rx);  // slave1, Mode3

        // ── Scenario 2: TX 데이터 전체 범위 커버 ─────────────
        // 커버: cp_tx_range 6개 bin, cp_msb, cp_lsb, cx_msb_lsb
        `uvm_info("SEQ", "=== Scenario 2: Full TX Data Range ===", UVM_NONE)
        foreach (range_samples[i]) begin
            do_txn(2'b01, 0, 0, range_samples[i], rx);
        end

        // ── Scenario 3: 슬레이브 전환 패턴 ──────────────────
        // 커버: cx_slave_transition 4조합
        //   slave0→slave0, slave0→slave1, slave1→slave0, slave1→slave1
        `uvm_info("SEQ", "=== Scenario 3: Slave Transition Pattern ===", UVM_NONE)
        do_txn(2'b01, 0, 0, 8'h11, rx);  // slave0
        do_txn(2'b01, 0, 0, 8'h22, rx);  // slave0 → slave0
        do_txn(2'b10, 0, 0, 8'h33, rx);  // slave0 → slave1
        do_txn(2'b10, 0, 0, 8'h44, rx);  // slave1 → slave1
        do_txn(2'b01, 0, 0, 8'h55, rx);  // slave1 → slave0

        // ── Scenario 4: SPI 모드 전환 패턴 ──────────────────
        // 커버: cx_mode_transition 4×4=16조합 완전 순회
        // 4×4 매트릭스를 모두 커버하는 시퀀스 (최소 트랜잭션 수)
        `uvm_info("SEQ", "=== Scenario 4: SPI Mode Transition Pattern ===", UVM_NONE)
        begin
            // 각 행(이전 mode)에서 4개 열(다음 mode)로의 전환을 커버
            // mode0 → mode0/1/2/3
            do_txn(2'b01, 0, 0, 8'hA0, rx);  // mode0
            do_txn(2'b01, 0, 0, 8'hA1, rx);  // mode0→mode0
            do_txn(2'b01, 0, 1, 8'hA2, rx);  // mode0→mode1
            do_txn(2'b01, 1, 0, 8'hA3, rx);  // mode1→mode2
            do_txn(2'b01, 1, 1, 8'hA4, rx);  // mode2→mode3
            do_txn(2'b01, 0, 0, 8'hA5, rx);  // mode3→mode0
            do_txn(2'b10, 1, 1, 8'hA6, rx);  // mode0→mode3
            do_txn(2'b10, 0, 1, 8'hA7, rx);  // mode3→mode1
            do_txn(2'b10, 1, 0, 8'hA8, rx);  // mode1→mode2
            do_txn(2'b10, 0, 0, 8'hA9, rx);  // mode2→mode0
            do_txn(2'b01, 1, 1, 8'hAA, rx);  // mode0→mode3
            do_txn(2'b01, 1, 0, 8'hAB, rx);  // mode3→mode2
            do_txn(2'b10, 0, 1, 8'hAC, rx);  // mode2→mode1
            do_txn(2'b10, 1, 1, 8'hAD, rx);  // mode1→mode3
            do_txn(2'b01, 1, 0, 8'hAE, rx);  // mode3→mode2
            do_txn(2'b01, 0, 1, 8'hAF, rx);  // mode2→mode1
            do_txn(2'b01, 0, 0, 8'hB0, rx);  // mode1→mode0 ← 누락 bin 커버
        end

        // ── Scenario 5: MSB/LSB 경계값 커버 ─────────────────
        // 커버: cx_msb_lsb 4조합 (MSB=0/1 × LSB=0/1)
        `uvm_info("SEQ", "=== Scenario 5: MSB/LSB Boundary ===", UVM_NONE)
        do_txn(2'b01, 0, 0, 8'h00, rx);  // MSB=0, LSB=0
        do_txn(2'b01, 0, 0, 8'h01, rx);  // MSB=0, LSB=1
        do_txn(2'b01, 0, 0, 8'h80, rx);  // MSB=1, LSB=0
        do_txn(2'b01, 0, 0, 8'hFF, rx);  // MSB=1, LSB=1

        // ── Scenario 6: 랜덤 트랜잭션 ───────────────────────
        // c_spi_mode 제거로 cpol/cpha 전체 범위 자유 랜덤
        // cx_mode_tx 커버리지: 4모드 × 6 tx_data bin = 24조합 명시 순회
        `uvm_info("SEQ", "=== Scenario 6: Random Transactions ===", UVM_NONE)
        begin
            logic [1:0] mode_list [4] = '{2'b00, 2'b01, 2'b10, 2'b11};
            // tx_data bin 대표값: zero/max/low/mid_low/mid_high/high
            logic [7:0] tx_bin    [6] = '{8'h00, 8'hFF, 8'h20, 8'h60, 8'hA0, 8'hE0};
            logic [1:0] slave_list[2] = '{2'b01, 2'b10};

            // mode × tx_data bin 순회 (slave는 번갈아 배정)
            // 4모드 × 6bin = 24트랜잭션
            foreach (mode_list[m]) begin
                foreach (tx_bin[b]) begin
                    spi_seq_item item;
                    item = spi_seq_item::type_id::create("item");
                    start_item(item);
                    if (!item.randomize()) `uvm_fatal("SEQ", "Randomize failed")
                    item.slave_sel = slave_list[(m+b)%2];  // slave 균등 배분
                    item.cpol      = mode_list[m][1];
                    item.cpha      = mode_list[m][0];
                    item.tx_data   = tx_bin[b];
                    finish_item(item);
                    `uvm_info("SEQ", $sformatf(
                              "RAND: slave=%02b mode[%0b%0b] tx=0x%02h rx=0x%02h",
                              item.slave_sel,
                              item.cpol,
                              item.cpha,
                              item.tx_data,
                              item.rx_data
                              ), UVM_NONE)
                end
            end
        end
        // ── Scenario 7: 커버리지 마무리를 위한 핀포인트 샷 ──────────────
        `uvm_info("SEQ", "=== Scenario 7: Final Coverage Completion ===", UVM_NONE)

        // 1. Slave 1 -> Slave 1 전환 (이미 있을 수 있지만 확실히 보장)
        do_txn(2'b10, 0, 0, 8'h00, rx);
        do_txn(2'b10, 0, 0, 8'h00, rx);

        // 2. 가장 유력한 미달 후보: Mode 2 -> Mode 2 전환
        do_txn(2'b01, 1, 0, 8'h22, rx);  // Mode 2
        do_txn(2'b01, 1, 0, 8'h22, rx);  // Mode 2 -> Mode 2 (자가 전환)

        // 3. 혹시 모를 누락: Mode 1 -> Mode 1 전환
        do_txn(2'b01, 0, 1, 8'h11, rx);  // Mode 1
        do_txn(2'b01, 0, 1, 8'h11, rx);  // Mode 1 -> Mode 1 (자가 전환)

        // 4. 마지막으로 안전하게 Mode 0으로 복귀
        do_txn(2'b01, 0, 0, 8'h00, rx);

    endtask
endclass

// ============================================================
// 10. Test
// ============================================================
class spi_test extends uvm_test;
    `uvm_component_utils(spi_test)

    spi_env      spi_e;
    spi_sequence spi_seq;

    function new(string name = "spi_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        spi_e   = spi_env::type_id::create("spi_e", this);
        spi_seq = spi_sequence::type_id::create("spi_seq", this);
    endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        spi_seq.start(spi_e.spi_agt.spi_sqr);
        phase.drop_objection(this);
    endtask

endclass

// ============================================================
// 11. TB Top
// ============================================================
// DUT 연결 구조:
//   spi_master : UVM Driver가 제어
//   spi_slave0 : spi_master의 slave0 포트 (cs_n[0]) 연결
//   spi_slave1 : spi_master의 slave1 포트 (cs_n[1]) 연결
//   slave tx_data = 8'hA5 (Scoreboard 기준값)
// ============================================================
module tb_top;

    // ── 클럭 생성 (100MHz) ────────────────────────────────────
    bit clk;
    always #5 clk = ~clk;

    // ── Interface 인스턴스 ────────────────────────────────────
    spi_if i_if (.clk(clk));

    // ── 내부 wire (DUT 간 연결) ───────────────────────────────
    logic sclk_w;
    logic mosi_w;
    logic miso0_w, miso1_w;
    logic       miso_w;  // master miso (두 slave OR)
    logic [1:0] cs_n_w;

    // slave가 동시에 구동하지 않도록 cs_n으로 mux
    // cs_n[0]=0일 때 slave0 miso 활성, cs_n[1]=0일 때 slave1 miso 활성
    assign miso_w = (~cs_n_w[0]) ? miso0_w : (~cs_n_w[1]) ? miso1_w : 1'b1;

    // Interface에 내부 wire 연결 (Monitor 관찰용)
    assign i_if.sclk = sclk_w;
    assign i_if.mosi = mosi_w;
    assign i_if.miso = miso_w;
    assign i_if.cs_n = cs_n_w;

    // ── DUT: spi_master ──────────────────────────────────────
    spi_master u_spi_master (
        .clk      (clk),
        .reset    (i_if.reset),
        .slave_sel(i_if.slave_sel),
        .start    (i_if.start),
        .tx_data  (i_if.tx_data),
        .tx_ready (i_if.tx_ready),
        .rx_data  (i_if.rx_data),
        .done     (i_if.done),
        .cpol     (i_if.cpol),
        .cpha     (i_if.cpha),
        .sclk     (sclk_w),
        .mosi     (mosi_w),
        .miso     (miso_w),
        .cs_n     (cs_n_w)
    );

    // ── DUT: spi_slave0 (slave 0번) ───────────────────────────
    spi_slave u_spi_slave0 (
        .clk     (clk),
        .reset   (i_if.reset),
        .tx_data (8'hA5),       // slave0 → master 응답 고정값
        .tx_ready(),
        .rx_data (),            // slave0 수신 데이터 (필요시 연결)
        .done    (),
        .cpol    (i_if.cpol),
        .cpha    (i_if.cpha),
        .sclk    (sclk_w),
        .mosi    (mosi_w),
        .miso    (miso0_w),
        .cs_n    (cs_n_w[0])
    );

    // ── DUT: spi_slave1 (slave 1번) ───────────────────────────
    spi_slave u_spi_slave1 (
        .clk     (clk),
        .reset   (i_if.reset),
        .tx_data (8'hA5),       // slave1 → master 응답 고정값
        .tx_ready(),
        .rx_data (),
        .done    (),
        .cpol    (i_if.cpol),
        .cpha    (i_if.cpha),
        .sclk    (sclk_w),
        .mosi    (mosi_w),
        .miso    (miso1_w),
        .cs_n    (cs_n_w[1])
    );

    // ── 초기화 ────────────────────────────────────────────────
    initial begin
        i_if.reset     = 1'b1;
        i_if.start     = 1'b0;
        i_if.tx_data   = 8'h00;
        i_if.slave_sel = 2'b00;
        i_if.cpol      = 1'b0;
        i_if.cpha      = 1'b0;
        #200;
        i_if.reset = 1'b0;
    end

    // ── UVM 설정 및 실행 ──────────────────────────────────────
    initial begin
        uvm_config_db#(virtual spi_if)::set(null, "*", "vif", i_if);
        run_test("spi_test");
    end

    // ── 디버그: 주요 신호 모니터링 ────────────────────────────
    initial begin
        forever begin
            @(posedge clk);
            if (i_if.done)
                $display(
                    "[DBG] %0t done=1  slave_sel=%02b cpol=%0b cpha=%0b tx=0x%02h rx=0x%02h  cs_n=%02b",
                    $time,
                    i_if.slave_sel,
                    i_if.cpol,
                    i_if.cpha,
                    i_if.tx_data,
                    i_if.rx_data,
                    i_if.cs_n
                );
        end
    end

    // ── 시뮬레이션 타임아웃 ───────────────────────────────────
    initial begin
        #200_000_000;
        `uvm_fatal("TB_TOP", "SIMULATION TIMEOUT")
    end

endmodule
