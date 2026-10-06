// ============================================================
//  I2C UVM Testbench — All-in-One  (v3)
//  APB UVM 구조 기반, I2C 트랜잭션 단위로 재설계
//
//  핵심 설계 원칙:
//   - Driver가 트랜잭션 완료 후 seq_item에 결과를 채워 반환
//   - Monitor는 tx_done/rx_done 신호 대신 트랜잭션 완료(DRV done) 기준으로 동작
//   - Scoreboard는 Monitor에서 받은 완성된 트랜잭션만 비교
//   - i2c_start 포트는 사용하지 않음 (Repeated Start 미사용)
//
//  파일 구성:
//    1.  i2c_if          (interface)
//    2.  i2c_seq_item
//    3.  i2c_driver
//    4.  i2c_monitor
//    5.  i2c_scoreboard
//    6.  i2c_coverage
//    7.  i2c_agent
//    8.  i2c_env
//    9.  i2c_sequence
//   10.  i2c_test
//   11.  tb_top
// ============================================================

import uvm_pkg::*;
`include "uvm_macros.svh"

// ============================================================
// 1. Interface
// ============================================================
interface i2c_if (input bit clk);
    logic       reset;
    logic       i2c_en;
    logic       i2c_start;
    logic       i2c_stop;
    logic [7:0] tx_data;
    logic       tx_done;
    logic       tx_ready;
    logic [7:0] rx_data;
    logic       rx_done;
    logic [7:0] outputdata;
    logic       scl;
endinterface

// ============================================================
// 2. Sequence Item
// ============================================================
class i2c_seq_item extends uvm_sequence_item;

    rand logic       we;          // 1=Write, 0=Read
    rand logic [7:0] wdata;       // Write 데이터 (we=1일 때)
         logic [7:0] rdata;       // Read 결과 (Driver가 채움)
         logic       ack_ok;      // Slave ACK 수신 여부

    // wdata 전체 범위 허용
    constraint c_wdata { wdata inside {[8'h00:8'hFF]}; }

    function new(string name = "i2c_seq_item");
        super.new(name);
    endfunction

    `uvm_object_utils_begin(i2c_seq_item)
        `uvm_field_int(we,     UVM_DEFAULT)
        `uvm_field_int(wdata,  UVM_DEFAULT)
        `uvm_field_int(rdata,  UVM_DEFAULT)
        `uvm_field_int(ack_ok, UVM_DEFAULT)
    `uvm_object_utils_end

endclass

// ============================================================
// 3. Driver
// ============================================================
// 설계 원칙:
//   - 모든 신호 대기는 wait() 레벨 폴링 사용 (combinational 신호 누락 방지)
//   - 각 트랜잭션 전 wait(tx_ready===1) 로 IDLE 진입 보장
//   - i2c_master FSM 제어 흐름:
//       WRITE: IDLE→START→DATA(addr)→ACK→HOLD→DATA(data)→ACK→HOLD→STOP→STOP_AFTER→IDLE
//       READ : IDLE→START→DATA(addr)→ACK→HOLD→READ→R_ACK→HOLD→STOP→STOP_AFTER→IDLE
// ============================================================
class i2c_driver extends uvm_driver #(i2c_seq_item);
    `uvm_component_utils(i2c_driver)

    virtual i2c_if vif;

    localparam logic [7:0] SLAVE_ADDR_W = 8'hA0;  // Write 주소
    localparam logic [7:0] SLAVE_ADDR_R = 8'hA1;  // Read  주소

    function new(string name = "i2c_driver", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db #(virtual i2c_if)::get(this, "", "vif", vif))
            `uvm_fatal(get_name(), "i2c_if not found");
    endfunction

    task run_phase(uvm_phase phase);
        vif.i2c_en    = 0;
        vif.i2c_start = 0;
        vif.i2c_stop  = 0;
        vif.tx_data   = 8'h00;
        forever begin
            seq_item_port.get_next_item(req);
            if (req.we) drv_write(req);
            else        drv_read(req);
            seq_item_port.item_done();
        end
    endtask

    //       tx_data는 반드시 i2c_en=1 이전에 안정화되어야 함
    //       → tx_data는 blocking(=) 할당, i2c_en은 그 다음 클럭에 인가
    // ── WRITE 트랜잭션 ────────────────────────────────────────
    // ★ HOLD Write 분기: tx_ready=1 & i2c_en=1 → DATA_1 재진입
    //   i2c_en=0 반드시 1클럭 먼저, 그 다음 i2c_stop=1 인가
    // ── WRITE 트랜잭션 ────────────────────────────────────────
    // ★ 핵심 설계: i2c_state_reg는 ACK_2에서 래치됨
    //   HOLD에서 i2c_stop=1을 줘도 이미 i2c_state_reg=I2C_DATA라서 STOP 불가
    //   → 데이터 ACK_2 직전(DATA_1~ACK_1 구간)에 i2c_stop=1이 있어야 함
    //   → tx_done 발생과 동시에(또는 직전에) i2c_stop=1 인가
    // ── WRITE 트랜잭션 ────────────────────────────────────────
    // 타이밍 규칙:
    //   tx_data  → tx_done 시점에 다음 데이터 세팅
    //   i2c_stop → tx_ready=1 시점에 세팅
    //   i2c_en   → IDLE/HOLD에서 계속 1 유지 (끄면 START/DATA 진입 불가)
    task drv_write(i2c_seq_item item);
        // 1. IDLE 진입 대기 (tx_ready=1)
        wait (vif.tx_ready === 1'b1);
        @(posedge vif.clk);
        $display("[DRV_W] %0t STEP1: IDLE", $time);

        // 2. 주소 전송 시작
        //    IDLE: i2c_en=1 → START 진입, tx_data 래치
        vif.tx_data  = SLAVE_ADDR_W;
        vif.i2c_en   = 1'b1;
        vif.i2c_stop = 1'b0;
        @(posedge vif.clk);
        $display("[DRV_W] %0t STEP2: addr=0xA0 en=1", $time);

        // 3. 주소 ACK 대기
        //    tx_done 시점에 tx_data를 다음 데이터(wdata)로 교체
        wait (vif.tx_done === 1'b1);
        vif.tx_data = item.wdata;   // tx_done에 맞춰 다음 데이터 세팅
        $display("[DRV_W] %0t STEP3: addr tx_done=1, next_data=0x%02h", $time, item.wdata);
        @(posedge vif.clk);
        wait (vif.tx_done === 1'b0);

        // 4. HOLD 진입 → tx_ready=1 대기
        //    tx_ready=1 시점에 i2c_stop=1 세팅 → STOP 예약
        //    i2c_en=1 유지 + i2c_stop=1 → HOLD에서 DATA_1 진입 후 ACK_2에서 I2C_STOP 래치
        wait (vif.tx_ready === 1'b1);
        vif.i2c_en   = 1'b1;
        vif.i2c_stop = 1'b1;   // tx_ready에 맞춰 STOP 예약
        @(posedge vif.clk);
        $display("[DRV_W] %0t STEP4: HOLD tx_ready=1, en=1 stop=1", $time);

        // 5. 데이터 ACK 대기
        wait (vif.tx_done === 1'b1);
        $display("[DRV_W] %0t STEP5: data tx_done=1 outputdata=0x%02h", $time, vif.outputdata);
        vif.i2c_en   = 1'b0;
        vif.i2c_stop = 1'b0;
        @(posedge vif.clk);
        wait (vif.tx_done === 1'b0);

        // 6. IDLE 복귀 대기
        wait (vif.tx_ready === 1'b0);
        wait (vif.tx_ready === 1'b1);
        $display("[DRV_W] %0t STEP6: IDLE outputdata=0x%02h", $time, vif.outputdata);
        repeat (2) @(posedge vif.clk);

        `uvm_info("DRV", $sformatf(
            "WRITE done: data=0x%02h  outputdata=0x%02h",
            item.wdata, vif.outputdata), UVM_NONE)
    endtask

    // ── READ 트랜잭션 ─────────────────────────────────────────
    // ★ READ: 주소(0xA1) ACK_2에서 {0,0}=I2C_DATA, rw_mode=1
    //   HOLD에서 rw_mode=1 → 조건 없이 READ_1 진입
    //   READ 후 HOLD → i2c_stop=1 필요 (다음 ACK 없으므로 직접 인가)
    // ── READ 트랜잭션 ─────────────────────────────────────────
    // 타이밍 규칙:
    //   i2c_en=1 유지: IDLE에서 START 진입 조건
    //   i2c_stop → tx_ready=1(HOLD Write분기 재진입) 시점에 세팅
    //   RTL 수정: HOLD READ분기에서 rw_mode_next=0 클리어
    //             → R_ACK_4→HOLD 재진입 시 Write분기로 전환
    task drv_read(i2c_seq_item item);
        // 1. IDLE 진입 대기
        wait (vif.tx_ready === 1'b1);
        @(posedge vif.clk);
        $display("[DRV_R] %0t STEP1: IDLE", $time);

        // 2. Read 주소 전송 (0xA1)
        //    i2c_stop=0: ACK_2에서 I2C_DATA 래치 → HOLD→READ_1
        vif.tx_data  = SLAVE_ADDR_R;
        vif.i2c_en   = 1'b1;
        vif.i2c_stop = 1'b0;
        @(posedge vif.clk);
        $display("[DRV_R] %0t STEP2: addr=0xA1 en=1 stop=0", $time);

        // 3. 주소 ACK 대기 (tx_done 시점)
        //    ★ tx_done 시점에 i2c_stop=1 미리 세팅
        //    → READ_1~R_ACK_4 진행 중 i2c_stop=1이 유지됨
        //    → R_ACK_4에서 n_state=HOLD 결정 시 i2c_stop=1이 이미 보임
        //    → HOLD 진입 시 i2c_stop=1 → STOP 전환
        wait (vif.tx_done === 1'b1);
        vif.i2c_stop = 1'b1;   // ★ tx_done 시점에 STOP 예약
        $display("[DRV_R] %0t STEP3: addr tx_done=1, stop=1 예약", $time);
        @(posedge vif.clk);
        wait (vif.tx_done === 1'b0);

        // 4. 데이터 수신 대기
        wait (vif.rx_done === 1'b1);
        $display("[DRV_R] %0t STEP4: rx_done=1 rx=0x%02h", $time, vif.rx_data);
        @(posedge vif.clk);
        item.rdata   = vif.rx_data;
        vif.i2c_stop = 1'b0;
        vif.i2c_en   = 1'b0;

        // 5. IDLE 복귀 대기 (HOLD→STOP→STOP_AFTER→IDLE)
        wait (vif.rx_done  === 1'b0);
        wait (vif.tx_ready === 1'b0);
        wait (vif.tx_ready === 1'b1);
        $display("[DRV_R] %0t STEP5: IDLE outputdata=0x%02h", $time, vif.outputdata);
        repeat (2) @(posedge vif.clk);

        `uvm_info("DRV", $sformatf(
            "READ  done: rx=0x%02h  outputdata=0x%02h",
            item.rdata, vif.outputdata), UVM_NONE)
    endtask

endclass

// ============================================================
// 4. Monitor
// ============================================================
// 설계 원칙:
//   - tx_done 신호를 트랜잭션 단위로 관찰
//   - 트랜잭션 타입(write/read)은 tx_data 값으로 구분
//     (0xA0 첫 번째 tx_done = WRITE 주소, 이후 = WRITE 데이터)
//     (0xA1 첫 번째 tx_done = READ  주소, 이후 = rx_done으로 처리)
//   - 상태머신으로 addr_phase / data_phase 추적
// ============================================================
class i2c_monitor extends uvm_monitor;
    `uvm_component_utils(i2c_monitor)

    uvm_analysis_port #(i2c_seq_item) send;
    virtual i2c_if vif;

    // 트랜잭션 추적용 상태
    typedef enum { MON_IDLE, MON_ADDR, MON_DATA } mon_state_t;
    mon_state_t mon_state;
    logic [7:0] captured_addr;

    function new(string name = "i2c_monitor", uvm_component parent);
        super.new(name, parent);
        send = new("send", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db #(virtual i2c_if)::get(this, "", "vif", vif))
            `uvm_fatal(get_name(), "i2c_if not found");
    endfunction

    task run_phase(uvm_phase phase);
        i2c_seq_item item;
        mon_state = MON_IDLE;
        forever begin
            // tx_done 감지 (레벨 폴링)
            wait (vif.tx_done === 1'b1);
            @(posedge vif.clk);

            case (mon_state)
                MON_IDLE, MON_ADDR: begin
                    // 첫 번째 tx_done = 주소 페이즈
                    captured_addr = vif.tx_data;
                    `uvm_info("MON", $sformatf(
                        "[ADDR] addr=0x%02h  outputdata=0x%02h",
                        captured_addr, vif.outputdata), UVM_NONE)

                    if (captured_addr == 8'hA1) begin
                        // READ 트랜잭션: rx_done 대기
                        wait (vif.tx_done === 1'b0);
                        wait (vif.rx_done === 1'b1);
                        @(posedge vif.clk);
                        item            = i2c_seq_item::type_id::create("item", this);
                        item.we         = 1'b0;
                        item.wdata      = 8'h00;
                        item.rdata      = vif.rx_data;
                        `uvm_info("MON", $sformatf(
                            "[READ ] rx=0x%02h  outputdata=0x%02h",
                            item.rdata, vif.outputdata), UVM_NONE)
                        send.write(item);
                        wait (vif.rx_done === 1'b0);
                        mon_state = MON_IDLE;
                    end else begin
                        // WRITE 트랜잭션: 다음 tx_done(데이터) 대기
                        mon_state = MON_DATA;
                    end
                end

                MON_DATA: begin
                    // 두 번째 tx_done = 데이터 페이즈
                    item        = i2c_seq_item::type_id::create("item", this);
                    item.we     = 1'b1;
                    item.wdata  = vif.tx_data;
                    item.rdata  = 8'h00;
                    `uvm_info("MON", $sformatf(
                        "[WRITE] data=0x%02h  outputdata=0x%02h",
                        item.wdata, vif.outputdata), UVM_NONE)
                    send.write(item);
                    mon_state = MON_IDLE;
                end
            endcase

            wait (vif.tx_done === 1'b0);  // tx_done 내려갈 때까지 대기 (중복 방지)
        end
    endtask

endclass

// ============================================================
// 5. Scoreboard
// ============================================================
class i2c_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(i2c_scoreboard)

    uvm_analysis_imp #(i2c_seq_item, i2c_scoreboard) recv;

    logic [7:0] write_queue[$];  // Write 데이터 큐 (Read 검증용)

    function new(string name = "i2c_scoreboard", uvm_component parent);
        super.new(name, parent);
        recv = new("recv", this);
    endfunction

    function void write(i2c_seq_item item);
        if (item.we) begin
            // ★ slave_ram은 단일 레지스터 → 마지막 Write 값만 보유
            // Write→Write 패턴 시 queue를 비우고 최신 값만 유지
            write_queue = {};
            write_queue.push_back(item.wdata);
            `uvm_info("SCB", $sformatf(
                "WRITE queued: 0x%02h  (queue=%0d)", item.wdata, write_queue.size()), UVM_NONE)
        end else begin
            if (write_queue.size() == 0) begin
                `uvm_info("SCB", "READ without prior WRITE (slave_ram 값 유지)", UVM_NONE)
            end else begin
                logic [7:0] exp;
                exp = write_queue[0];  // pop 안 함: 연속 Read 지원
                if (item.rdata === exp)
                    `uvm_info("SCB", $sformatf(
                        "LOOPBACK MATCH  : exp=0x%02h  got=0x%02h ✓", exp, item.rdata), UVM_NONE)
                else
                    `uvm_error("SCB", $sformatf(
                        "LOOPBACK MISMATCH: exp=0x%02h  got=0x%02h", exp, item.rdata))
            end
        end
    endfunction

endclass

// ============================================================
// 6. Coverage
// ============================================================
// 커버리지 설계 원칙 (RTL 기반):
//   - i2c_master FSM: IDLE/START/DATA/ACK/HOLD/READ/R_ACK/STOP
//   - i2c_slave  FSM: IDLE/START/ADDR/ACK/RX_SHIFT/RX_ACK/TX_SHIFT/TX_ACK
//   - 슬레이브 주소: 7'h50 고정 (0xA0=Write, 0xA1=Read)
//   - 데이터: 8비트 전체 범위
// ============================================================
class i2c_coverage extends uvm_subscriber #(i2c_seq_item);
    `uvm_component_utils(i2c_coverage)

    i2c_seq_item curr_item;
    i2c_seq_item prev_item;  // 이전 트랜잭션 (연속 패턴 추적용)

    // ── Master FSM 상태 관찰용 ───────────────────────────────
    virtual i2c_if vif;

    covergroup cg_i2c_transaction;

        // ── 1. 트랜잭션 방향 ─────────────────────────────────
        cp_direction: coverpoint curr_item.we {
            bins write_op = {1'b1};  // Master Write
            bins read_op  = {1'b0};  // Master Read
        }

        // ── 2. 슬레이브 주소: 7'h50 고정 (Write=0xA0, Read=0xA1)
        //    cp_direction과 동일 신호이므로 별도 coverpoint 불필요

        // ── 3. Write 데이터 값 범위 ───────────────────────────
        cp_wdata_range: coverpoint curr_item.wdata {
            bins zero       = {8'h00};          // 최솟값 경계
            bins max_val    = {8'hFF};          // 최댓값 경계
            bins low        = {[8'h01:8'h3F]}; // 하위 범위
            bins mid_low    = {[8'h40:8'h7F]}; // 중하위 범위
            bins mid_high   = {[8'h80:8'hBF]}; // 중상위 범위
            bins high       = {[8'hC0:8'hFE]}; // 상위 범위
        }

        // ── 4. 데이터 MSB / LSB 비트 패턴 ────────────────────
        cp_msb: coverpoint curr_item.wdata[7] {
            bins msb_0 = {1'b0};  // MSB=0 (0x00~0x7F)
            bins msb_1 = {1'b1};  // MSB=1 (0x80~0xFF)
        }
        cp_lsb: coverpoint curr_item.wdata[0] {
            bins lsb_0 = {1'b0};  // 짝수
            bins lsb_1 = {1'b1};  // 홀수
        }

        // ── 5. Read 수신 데이터 값 범위 ──────────────────────
        // Read 트랜잭션(we=0)에서만 rdata가 의미있음
        // Write 트랜잭션에서는 rdata=0x00 → ignore
        cp_rdata_range: coverpoint curr_item.rdata iff (curr_item.we == 0) {
            bins zero       = {8'h00};
            bins max_val    = {8'hFF};
            bins low        = {[8'h01:8'h3F]};
            bins mid_low    = {[8'h40:8'h7F]};
            bins mid_high   = {[8'h80:8'hBF]};
            bins high       = {[8'hC0:8'hFE]};
        }

        // ── 6. cp_read_occur: cp_direction과 동일 → 제거

        // ── 7. Write 방향에서의 데이터 범위 커버 ────────────
        // Read 트랜잭션에서 wdata는 의미없는 필드이므로
        // Write 방향만 wdata range와 cross
        cx_dir_wdata: cross cp_direction, cp_wdata_range {
            ignore_bins read_wdata = binsof(cp_direction.read_op);
        }

        // ── 8. MSB × LSB 조합 커버 ───────────────────────────
        cx_msb_lsb: cross cp_msb, cp_lsb;

        // ── 9. 방향 × MSB 조합 ───────────────────────────────
        cx_dir_msb: cross cp_direction, cp_msb {
            ignore_bins read_msb = binsof(cp_direction.read_op);
        }

    endgroup

    // ── 연속 트랜잭션 패턴 커버그룹 ─────────────────────────
    covergroup cg_i2c_sequence_pattern;

        // ── 10. 이전→현재 트랜잭션 패턴 ─────────────────────
        // Write→Read: 일반적인 루프백
        // Write→Write: 연속 Write
        // Read→Write: Read 후 새로운 Write
        // Read→Read: 연속 Read (동일 slave_ram 읽기)
        cp_prev_dir: coverpoint (prev_item != null ? prev_item.we : 1'bx) {
            bins prev_write = {1'b1};
            bins prev_read  = {1'b0};
        }
        cp_curr_dir: coverpoint curr_item.we {
            bins curr_write = {1'b1};
            bins curr_read  = {1'b0};
        }
        cx_seq_pattern: cross cp_prev_dir, cp_curr_dir;

    endgroup

    // ── Master FSM 상태 커버그룹 (인터페이스 신호 기반) ──────
    // clocked covergroup은 new()에서만 인스턴스화 가능하므로
    // non-clocked로 선언 후 run_phase에서 매 클럭 sample() 호출
    covergroup cg_i2c_bus_activity;

        // ── 11. tx_done / rx_done 발생 여부 ──────────────────
        cp_tx_done: coverpoint vif.tx_done {
            bins active   = {1'b1};
            bins inactive = {1'b0};
        }
        cp_rx_done: coverpoint vif.rx_done {
            bins active   = {1'b1};
            bins inactive = {1'b0};
        }

        // ── 12. tx_ready 상태 (IDLE/HOLD 여부) ───────────────
        cp_tx_ready: coverpoint vif.tx_ready {
            bins ready    = {1'b1};  // IDLE 또는 HOLD Write분기
            bins not_ready= {1'b0};  // 전송 중
        }

        // ── 13. SCL 상태 ──────────────────────────────────────
        cp_scl: coverpoint vif.scl {
            bins scl_high = {1'b1};
            bins scl_low  = {1'b0};
        }

        // ── 14. tx_done × tx_ready 동시 상태 ─────────────────
        // tx_done=1 & tx_ready=1은 동시 발생 불가 → illegal_bins 처리
        cx_done_ready: cross cp_tx_done, cp_tx_ready {
            illegal_bins impossible = binsof(cp_tx_done.active) &&
                                      binsof(cp_tx_ready.ready);
        }

    endgroup

    function new(string name = "i2c_coverage", uvm_component parent);
        super.new(name, parent);
        prev_item = null;
        // ★ embedded covergroup은 반드시 new() 안에서 인스턴스화
        cg_i2c_transaction      = new();
        cg_i2c_sequence_pattern = new();
        cg_i2c_bus_activity     = new();
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db #(virtual i2c_if)::get(this, "", "vif", vif))
            `uvm_fatal(get_name(), "i2c_if not found");
    endfunction

    // 매 클럭마다 버스 신호 커버리지 샘플링
    task run_phase(uvm_phase phase);
        forever begin
            @(posedge vif.clk);
            cg_i2c_bus_activity.sample();
        end
    endtask

    function void write(i2c_seq_item t);
        curr_item = t;
        cg_i2c_transaction.sample();
        cg_i2c_sequence_pattern.sample();
        prev_item = t;  // 현재를 이전으로 저장
    endfunction

    // 시뮬 종료 시 커버리지 리포트 출력
    function void report_phase(uvm_phase phase);
        `uvm_info("COV", $sformatf(
            "\n=== Coverage Report ===\n  cg_i2c_transaction     : %.1f%%\n  cg_i2c_sequence_pattern: %.1f%%\n  cg_i2c_bus_activity    : %.1f%%",
            cg_i2c_transaction.get_coverage(),
            cg_i2c_sequence_pattern.get_coverage(),
            cg_i2c_bus_activity.get_coverage()
        ), UVM_NONE)
    endfunction

endclass

// ============================================================
// 7. Agent
// ============================================================
class i2c_agent extends uvm_agent;
    `uvm_component_utils(i2c_agent)

    i2c_driver                    i2c_drv;
    i2c_monitor                   i2c_mon;
    uvm_sequencer #(i2c_seq_item) i2c_sqr;

    function new(string name = "i2c_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        i2c_drv = i2c_driver::type_id::create("i2c_drv", this);
        i2c_mon = i2c_monitor::type_id::create("i2c_mon", this);
        i2c_sqr = uvm_sequencer #(i2c_seq_item)::type_id::create("i2c_sqr", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        i2c_drv.seq_item_port.connect(i2c_sqr.seq_item_export);
    endfunction

endclass

// ============================================================
// 8. Env
// ============================================================
class i2c_env extends uvm_env;
    `uvm_component_utils(i2c_env)

    i2c_agent      i2c_agt;
    i2c_scoreboard i2c_scb;
    i2c_coverage   i2c_cov;

    function new(string name = "i2c_env", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        i2c_agt = i2c_agent::type_id::create("i2c_agt", this);
        i2c_scb = i2c_scoreboard::type_id::create("i2c_scb", this);
        i2c_cov = i2c_coverage::type_id::create("i2c_cov", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        i2c_agt.i2c_mon.send.connect(i2c_scb.recv);
        i2c_agt.i2c_mon.send.connect(i2c_cov.analysis_export);
    endfunction

endclass

// ============================================================
// 9. Sequence
// ============================================================
// ============================================================
// 9. Sequence
// ============================================================
// 커버리지 100% 달성을 위한 시나리오:
//   Scenario 1: 코너 케이스 Write→Read (0x00, 0xFF, 0x40, 0xC0)
//   Scenario 2: 랜덤 Write→Read 루프백 (데이터 범위 커버)
//   Scenario 3: 연속 Write→Write (Write→Write 패턴 커버)
//   Scenario 4: 연속 Read→Read (Read→Read 패턴 커버)
//   Scenario 5: MSB/LSB 경계 데이터 (비트 패턴 커버)
// ============================================================
// ============================================================
// 9. Sequence
// ============================================================
class i2c_sequence extends uvm_sequence #(i2c_seq_item);
    `uvm_object_utils(i2c_sequence)

    // 각 데이터 범위 대표값
    logic [7:0] corner_cases[4]  = '{8'h00, 8'hFF, 8'h40, 8'hC0};
    logic [7:0] range_samples[6] = '{8'h00, 8'hFF, 8'h20, 8'h60, 8'hA0, 8'hE0};

    function new(string name = "i2c_sequence");
        super.new(name);
    endfunction

    task do_write(logic [7:0] data);
        i2c_seq_item item;
        item = i2c_seq_item::type_id::create("item");
        start_item(item);
        item.we    = 1'b1;
        item.wdata = data;
        finish_item(item);
        `uvm_info("SEQ", $sformatf("WRITE: 0x%02h", data), UVM_NONE)
    endtask

    task do_read(output logic [7:0] rdata);
        i2c_seq_item item;
        item = i2c_seq_item::type_id::create("item");
        start_item(item);
        item.we = 1'b0;
        finish_item(item);
        rdata = item.rdata;
        `uvm_info("SEQ", $sformatf("READ : 0x%02h", item.rdata), UVM_NONE)
    endtask

    task body();
        logic [7:0] rnd, rdata;
        int unsigned num_iter = 10;

        virtual i2c_if vif;
        if (!uvm_config_db #(virtual i2c_if)::get(null, "*", "vif", vif))
            `uvm_fatal("SEQ", "i2c_if not found")

        @(negedge vif.reset);
        repeat (10) @(posedge vif.clk);

        // ── Scenario 1: 코너 케이스 Write→Read ──────────────
        // 커버: zero, max, mid_low, mid_high, Write→Read 패턴
        `uvm_info("SEQ", "=== Scenario 1: Corner Case Write-Read ===", UVM_NONE)
        foreach (corner_cases[i]) begin
            do_write(corner_cases[i]);
            do_read(rdata);
        end

        // ── Scenario 2: 전체 데이터 범위 Write→Read ─────────
        // 커버: 6개 bin 전체, MSB/LSB 4조합
        `uvm_info("SEQ", "=== Scenario 2: Full Range Write-Read ===", UVM_NONE)
        foreach (range_samples[i]) begin
            do_write(range_samples[i]);
            do_read(rdata);
        end

        // ── Scenario 3: Write→Write 패턴 커버 ───────────────
        // Write→Write: SCB queue에 쌓이지만 이후 Read로 소진
        // cx_seq_pattern(Write→Write) 달성
        `uvm_info("SEQ", "=== Scenario 3: Write-Write-Read ===", UVM_NONE)
        do_write(8'h11);   // Write
        do_write(8'h22);   // Write→Write 패턴
        do_read(rdata);    // 마지막 Write(0x22)만 읽힘 → SCB: exp=0x11, got=0x22 MISMATCH 발생 가능
        // ★ SCB queue 소진을 위해 추가 Read
        do_read(rdata);    // exp=0x22

        // ── Scenario 4: Read→Write 패턴 + Read→Read 패턴 ───
        // cx_seq_pattern 나머지 bin 달성
        `uvm_info("SEQ", "=== Scenario 4: Read-Write / Read-Read ===", UVM_NONE)
        do_write(8'h33);   // Write (준비)
        do_read(rdata);    // Read
        do_read(rdata);    // Read→Read 패턴 (slave_ram=0x33 유지)
        do_write(8'h44);   // Read→Write 패턴
        do_read(rdata);    // Write→Read

        // ── Scenario 5: MSB/LSB 경계값 ──────────────────────
        `uvm_info("SEQ", "=== Scenario 5: MSB/LSB Boundary ===", UVM_NONE)
        do_write(8'h01); do_read(rdata);  // MSB=0, LSB=1
        do_write(8'h80); do_read(rdata);  // MSB=1, LSB=0

        // ── Scenario 6: 랜덤 Write→Read ─────────────────────
        `uvm_info("SEQ", "=== Scenario 6: Random Write-Read ===", UVM_NONE)
        repeat (num_iter) begin
            rnd = $urandom_range(0, 255);
            do_write(rnd);
            do_read(rdata);
        end

    endtask
endclass

// ============================================================
// 10. Test
// ============================================================
class i2c_test extends uvm_test;
    `uvm_component_utils(i2c_test)

    i2c_env      i2c_e;
    i2c_sequence i2c_seq;

    function new(string name = "i2c_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        i2c_e   = i2c_env::type_id::create("i2c_e", this);
        i2c_seq = i2c_sequence::type_id::create("i2c_seq", this);
    endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        i2c_seq.start(i2c_e.i2c_agt.i2c_sqr);
        phase.drop_objection(this);
    endtask

endclass

// ============================================================
// 11. TB Top
// ============================================================
module tb_top;

    bit clk;
    always #5 clk = ~clk;

    wire sda;
    pullup (sda);

    i2c_if i_if (.clk(clk));

    dut_top_i2c DUT (
        .clk       (clk),
        .reset     (i_if.reset),
        .i2c_en    (i_if.i2c_en),
        .i2c_start (i_if.i2c_start),
        .i2c_stop  (i_if.i2c_stop),
        .tx_data   (i_if.tx_data),
        .tx_done   (i_if.tx_done),
        .tx_ready  (i_if.tx_ready),
        .rx_data   (i_if.rx_data),
        .rx_done   (i_if.rx_done),
        .outputdata(i_if.outputdata),
        .scl       (i_if.scl),
        .sda       (sda)
    );

    initial begin
        i_if.reset    = 1;
        i_if.i2c_en   = 0;
        i_if.i2c_start= 0;
        i_if.i2c_stop = 0;
        i_if.tx_data  = 8'h00;
        #200;
        i_if.reset = 0;
    end

    initial begin
        uvm_config_db #(virtual i2c_if)::set(null, "*", "vif", i_if);
        run_test("i2c_test");
    end

    // 디버그: FSM 상태 및 주요 신호 모니터링
    initial begin
        forever begin
            @(posedge clk);
            if (i_if.tx_done)
                $display("[DBG] %0t tx_done=1 tx_data=0x%02h tx_ready=%0b rx_done=%0b outputdata=0x%02h",
                    $time, i_if.tx_data, i_if.tx_ready, i_if.rx_done, i_if.outputdata);
            if (i_if.rx_done)
                $display("[DBG] %0t rx_done=1 rx_data=0x%02h outputdata=0x%02h",
                    $time, i_if.rx_data, i_if.outputdata);
        end
    end

    initial begin
        #50_000_000;
        `uvm_fatal("TB_TOP", "SIMULATION TIMEOUT")
    end

endmodule