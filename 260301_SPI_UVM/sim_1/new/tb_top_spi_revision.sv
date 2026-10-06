import uvm_pkg::*;
`include "uvm_macros.svh"

interface spi_ctrl_if (
    input bit clk
);

    logic       reset;
    logic [1:0] slave_sel;
    logic       start;
    logic [7:0] tx_data;
    logic       tx_ready;
    logic [7:0] rx_data;
    logic       done;
    logic       cpol;
    logic       cpha;

endinterface

interface spi_bus_if (
    input bit clk
);
    logic       sclk;
    logic       mosi;
    logic       miso;
    logic [1:0] cs_n;

    //assertion 
    logic       reset;  //disable iff 용 (전역 신호)
    logic       cpol;  // 버스 파형 규격
    logic       cpha;  // 버스 파형 규격

    // ── 타이밍 상수 ─────────────────────────
    localparam int HALF_PERIOD = 50;  // sclk half period (clk 사이클)
    localparam int EDGES_PER_XFER = 16;  // 8비트 = 16엣지

    // ══════════════════════════════════════════
    //  SVA 프로토콜 체커
    // ══════════════════════════════════════════

    // 1. CS 배타성 — 두 슬레이브 동시 선택 금지
    property p_cs_exclusive;
        @(posedge clk) disable iff (reset) $onehot0(
            ~cs_n
        );
    endproperty
    a_cs_exclusive :
    assert property (p_cs_exclusive)
    else $error("[SVA] CS_EXCLUSIVE 위반: cs_n=%b (두 슬레이브 동시 선택)", cs_n);

    // 2. CS 비활성 시 sclk는 idle 레벨(cpol)
    // property p_sclk_idle;
    //     @(posedge clk) disable iff (reset) (cs_n == 2'b11) |-> (sclk == cpol);
    // endproperty

    property p_sclk_idle;
    @(posedge clk) disable iff (reset)
        (cs_n == 2'b11) && $stable(cpol) |-> (sclk == cpol);
        //                 ↑ cpol이 방금 바뀐 사이클은 검사 제외
    endproperty
    
    a_sclk_idle :
    assert property (p_sclk_idle)
    else $error("[SVA] SCLK_IDLE 위반: cs_n 비활성인데 sclk=%b (cpol=%b)", sclk, cpol);

    // 3. sclk 최소 펄스 폭 — 한번 바뀌면 HALF_PERIOD 동안 유지
    property p_sclk_min_width;
        @(posedge clk) disable iff (reset) $changed(
            sclk
        ) |=> $stable(
            sclk
        ) [* (HALF_PERIOD - 1)];
    endproperty
    a_sclk_min_width :
    assert property (p_sclk_min_width)
    else $error("[SVA] SCLK_WIDTH 위반: half period가 %0d clk 미만", HALF_PERIOD);

    // 4. t_CSS — CS assert 후 첫 sclk 엣지까지 최소 시간
    property p_cs_setup;
        @(posedge clk) disable iff (reset) ($fell(
            cs_n[0]
        ) || $fell(
            cs_n[1]
        )) |=> $stable(
            sclk
        ) [* (HALF_PERIOD - 1)];
    endproperty
    a_cs_setup :
    assert property (p_cs_setup)
    else $error("[SVA] CS_SETUP(t_CSS) 위반: CS assert 직후 sclk가 너무 일찍 변함");

    // 5. t_CSH — 마지막 sclk 엣지 후 CS 해제까지 최소 시간
    property p_cs_hold;
        @(posedge clk) disable iff (reset) ($changed(
            sclk
        ) && (cs_n != 2'b11)) |=> (cs_n != 2'b11) [* (HALF_PERIOD - 1)];
    endproperty
    a_cs_hold :
    assert property (p_cs_hold)
    else $error("[SVA] CS_HOLD(t_CSH) 위반: sclk 엣지 후 CS가 너무 일찍 풀림");

    // ── 엣지 카운터 (assertion 6번용) ──────────
    logic sclk_d;
    int   edge_cnt;
    always @(posedge clk) begin
        if (reset) begin
            sclk_d   <= cpol;
            edge_cnt <= 0;
        end else begin
            sclk_d <= sclk;
            if (cs_n == 2'b11) edge_cnt <= 0;
            else if (sclk !== sclk_d) edge_cnt <= edge_cnt + 1;
        end
    end

    // 6. 트랜잭션당 엣지 개수 — 잘리거나 늘어나면 안 됨
    property p_edge_count;
        @(posedge clk) disable iff (reset) $rose(
            cs_n == 2'b11
        ) |-> (edge_cnt == EDGES_PER_XFER);
    endproperty
    a_edge_count :
    assert property (p_edge_count)
    else $error("[SVA] EDGE_COUNT 위반: 엣지 %0d개 (기대 %0d개)", edge_cnt, EDGES_PER_XFER);

    // 7. 전송 중 CS 값이 바뀌면 안 됨 (슬레이브 전환 금지)
    //    허용: 활성 → 같은 값 유지, 또는 활성 → 완전 해제(2'b11)
    //    금지: 활성 → 다른 활성 (01 ↔ 10)
    property p_cs_stable_during_xfer;
        @(posedge clk) disable iff (reset) (cs_n != 2'b11) |=> ($stable(
            cs_n
        ) || (cs_n == 2'b11));
    endproperty
    a_cs_stable_during_xfer :
    assert property (p_cs_stable_during_xfer)
    else $error("[SVA] CS_STABLE 위반: 전송 중 CS가 %b 로 변경됨", cs_n);

endinterface

typedef enum bit [1:0] {
    EVIL_NONE = 2'd0,
    EVIL_SEL_CHANGE = 2'd1,  // 전송 중 slave_sel 변경  ← 실제 버그 정조준
    EVIL_SEL_BOTH = 2'd2  // slave_sel = 2'b11 (둘 다 선택)
} evil_mode_e;

class spi_cfg extends uvm_object;
    `uvm_object_utils(spi_cfg)
    rand bit cpol;
    rand bit cpha;
    rand bit [7:0] slave0_resp;
    rand bit [7:0] slave1_resp;
    evil_mode_e evil_mode = EVIL_NONE;

    constraint c_diff {slave0_resp != slave1_resp;}

    function new(string name = "spi_cfg");
        super.new(name);
    endfunction
endclass


class spi_seq_item extends uvm_sequence_item;
    rand bit [7:0] tx_data;
    rand bit [1:0] slave_sel;

    bit [7:0] rx_data;

    constraint c_slave {slave_sel inside {2'b01, 2'b10};}

    `uvm_object_utils_begin(spi_seq_item)
        `uvm_field_int(tx_data, UVM_DEFAULT)
        `uvm_field_int(slave_sel, UVM_DEFAULT)
        `uvm_field_int(rx_data, UVM_DEFAULT)
    `uvm_object_utils_end

    function new(string name = "spi_seq_item");
        super.new(name);

    endfunction  //new()
endclass  //spi_seq_item    extends superClass

class spi_bus_item extends uvm_sequence_item;
    bit [7:0] mosi_byte;
    bit [7:0] miso_byte;
    bit [1:0] cs_active;

    `uvm_object_utils(spi_bus_item)

    function new(string name = "spi_bus_item");
        super.new(name);

    endfunction  //new()
endclass  //spi_bus_item extends uvm_sequence_item

class spi_base_seq extends uvm_sequence #(spi_seq_item);
    `uvm_object_utils(spi_base_seq)
    rand int unsigned num_txn;
    constraint c_num {soft num_txn inside {[20 : 50]};}  //[4:8]에서 트랜잭션 수 늘리기

    function new(string name = "spi_base_seq");
        super.new(name);

    endfunction  //new()

    task body();
        spi_seq_item item;
        repeat (num_txn) begin
            item = spi_seq_item::type_id::create("item");
            start_item(item);
            if (!item.randomize()) `uvm_fatal("SEQ", "randomize failed")
            finish_item(item);
        end
    endtask  //body
endclass  //spi_base_seq extends superClass

class spi_boundary_seq extends uvm_sequence #(spi_seq_item);
    `uvm_object_utils(spi_boundary_seq)

    function new(string name = "spi_boundary_seq");
        super.new(name);
    endfunction

    task body();
        // 6개 bin의 대표값 (zero / max / others[4]를 고루 겨냥)
        bit [7:0] vals[] = '{8'h00, 8'hFF, 8'h20, 8'h60, 8'hA0, 8'hE0};
        bit [1:0] slaves[] = '{2'b01, 2'b10};
        spi_seq_item item;

        foreach (vals[i]) begin
            foreach (slaves[j]) begin
                item = spi_seq_item::type_id::create("item");
                start_item(item);
                item.tx_data   = vals[i];  // ★ randomize 안 씀 — directed니까
                item.slave_sel = slaves[j];
                finish_item(item);
            end
        end
    endtask
endclass

class spi_driver extends uvm_driver #(spi_seq_item);
    `uvm_component_utils(spi_driver)
    virtual spi_ctrl_if vif;
    spi_cfg cfg;

    function new(string name, uvm_component parent);
        super.new(name, parent);

    endfunction  //new()

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_ctrl_if)::get(this, "", "ctrl_vif", vif))
            `uvm_fatal(get_name(), "ctrl_vif not found")
        if (!uvm_config_db#(spi_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal(get_name(), "cfg not found")
    endfunction

    // task reset_phase(uvm_phase phase);
    //     phase.raise_objection(this);
    //     vif.start = 1'b0;
    //     vif.tx_data = 8'h00;
    //     vif.slave_sel = 2'b00;
    //     vif.cpol = cfg.cpol;
    //     vif.cpha = cfg.cpha;
    //     @(negedge vif.reset);
    //     repeat (10) @(posedge vif.clk);
    //     phase.drop_objection(this);
    // endtask

    task run_phase(uvm_phase phase);

        // 초기값 세팅
        vif.start     = 1'b0;
        vif.tx_data   = 8'h00;
        vif.slave_sel = 2'b00;
        vif.cpol      = cfg.cpol;
        vif.cpha      = cfg.cpha;

        // ★ 리셋 해제를 여기서 기다림
        wait (vif.reset === 1'b0);  //@(negedge vif.reset) 대신 level로 활용.
        repeat (10) @(posedge vif.clk);

        forever begin
            seq_item_port.get_next_item(req);
            drive_one(req);
            seq_item_port.item_done();
        end
    endtask



    //task drive_one(spi_seq_item item);
    virtual task drive_one(spi_seq_item item);  //virtual 추가 
        wait (vif.tx_ready === 1'b1);
        @(posedge vif.clk);
        vif.cpol      = cfg.cpol;  // coverage 채우기 위해 이동
        vif.cpha      = cfg.cpha;
        vif.slave_sel = item.slave_sel;
        vif.tx_data   = item.tx_data;
        @(posedge vif.clk);
        vif.start = 1'b1;
        @(posedge vif.clk);
        vif.start = 1'b0;

        wait (vif.done === 1'b1);
        @(posedge vif.clk);
        //item.rx_data = vif.rx_data; //ctrl_monitor 대신 사용

        wait (vif.done === 1'b0);
        vif.slave_sel = 2'b00;
        wait (vif.tx_ready === 1'b1);
        repeat (2) @(posedge vif.clk);

        //ctrl_monitor 대신 사용
        //`uvm_info("DRV", $sformatf("mode[%0b%0b] slave=%02b tx=0x%02h rx=0x%02h (expect 0x%02h)", cfg.cpol, cfg.cpha,
        //                           item.slave_sel, item.tx_data, item.rx_data,
        //                           (item.slave_sel == 2'b01) ? cfg.slave0_resp : cfg.slave1_resp), UVM_NONE)
    endtask
endclass

class spi_evil_driver extends spi_driver;
    `uvm_component_utils(spi_evil_driver)

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual task drive_one(spi_seq_item item);
        fork
            begin
                case (cfg.evil_mode)
                    EVIL_SEL_CHANGE: begin
                        wait (vif.tx_ready === 1'b0);  // 전송 시작 대기
                        repeat (250) @(posedge vif.clk);  // 2~3비트 지난 시점
                        `uvm_info("EVIL", $sformatf(
                                  "전송 중 slave_sel 변경 주입: %02b -> %02b", vif.slave_sel, ~vif.slave_sel),
                                  UVM_NONE)
                        vif.slave_sel = ~vif.slave_sel;  // ★ 위반 주입
                    end
                    EVIL_SEL_BOTH: begin
                        wait (vif.tx_ready === 1'b0);
                        repeat (250) @(posedge vif.clk);
                        `uvm_info("EVIL", "slave_sel=2'b11 주입 (둘 다 선택)", UVM_NONE)
                        vif.slave_sel = 2'b11;  // ★ 위반 주입
                    end
                    default: ;  // EVIL_NONE
                endcase
            end
        join_none

        super.drive_one(item);  // 정상 동작은 그대로 수행
        disable fork;  // 주입 스레드 정리
    endtask
endclass

class spi_ctrl_monitor extends uvm_monitor;
    `uvm_component_utils(spi_ctrl_monitor)
    virtual spi_ctrl_if vif;
    uvm_analysis_port #(spi_seq_item) ap;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction  //new()

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_ctrl_if)::get(this, "", "ctrl_vif", vif))
            `uvm_fatal(get_name(), "ctrl_vif not found")
    endfunction

    task run_phase(uvm_phase phase);
        spi_seq_item item;
        wait (vif.reset == 1'b0);  //reset 대기

        forever begin
            wait (vif.done === 1'b1);
            @(posedge vif.clk);

            item = spi_seq_item::type_id::create("item");
            item.tx_data = vif.tx_data;
            item.rx_data = vif.rx_data;
            item.slave_sel = vif.slave_sel;

            `uvm_info("CTRL_MON", $sformatf(
                      "observed slave=%02b tx=0x%02h rx=0x%02h", item.slave_sel, item.tx_data, item.rx_data), UVM_HIGH)
            ap.write(item);
            wait (vif.done === 1'b0);  // 중복 캡처 방지
        end
    endtask  //run_phase

endclass  //spi_ctrl_monitor extends uvm_monitor

class spi_ctrl_agent extends uvm_agent;
    `uvm_component_utils(spi_ctrl_agent)
    spi_driver drv;
    uvm_sequencer #(spi_seq_item) sqr;
    spi_ctrl_monitor mon;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        drv = spi_driver::type_id::create("drv", this);
        sqr = uvm_sequencer#(spi_seq_item)::type_id::create("sqr", this);
        mon = spi_ctrl_monitor::type_id::create("mon", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        drv.seq_item_port.connect(sqr.seq_item_export);  // no port
    endfunction  //new()
endclass  //spi_ctrl_agent extends uvm_agent

class spi_slave_driver extends uvm_component;
    `uvm_component_utils(spi_slave_driver)
    virtual spi_bus_if vif;
    spi_cfg cfg;
    uvm_analysis_port #(spi_bus_item) ap;  //ap 참조 추가

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);  //ap 참조 추가
    endfunction  //new()
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_bus_if)::get(this, "", "bus_vif", vif))
            `uvm_fatal(get_name(), "bus_vif not found")
        if (!uvm_config_db#(spi_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal(get_name(), "cfg not found")
    endfunction

    task run_phase(uvm_phase phase);
        bit          [7:0] sr;
        bit                shift_on_rise;
        spi_bus_item       rsp;  //ap
        vif.miso = 1'b1;
        //        shift_on_rise = (cfg.cpol ^ cfg.cpha); //coverage하면서 이동

        wait (vif.cs_n === 2'b11);
        forever begin
            wait (vif.cs_n !== 2'b11);
            shift_on_rise = (cfg.cpol ^ cfg.cpha);

            //응답값 결정 — 경계값이 섞이도록 가중 랜덤
            //sr = $urandom_range(0, 255); 대신 사용
            randcase
                1: sr = 8'h00;
                1: sr = 8'hFF;
                6: sr = $urandom_range(0, 255);
            endcase

            // ★ "내가 이걸 보낸다"고 알림
            rsp = spi_bus_item::type_id::create("rsp");
            rsp.miso_byte = sr;
            rsp.cs_active = ~vif.cs_n;
            ap.write(rsp);

            // sr = (vif.cs_n[0] == 1'b0) ? cfg.slave0_resp : cfg.slave1_resp;
            // cfg로 덮어쓰던 줄은 삭제
            vif.miso = sr[7];

            if (cfg.cpha) begin
                if (shift_on_rise) @(posedge vif.sclk);
                else @(negedge vif.sclk);
            end

            for (int i = 0; i < 7; i++) begin
                if (shift_on_rise) @(posedge vif.sclk);
                else @(negedge vif.sclk);
                sr = {sr[6:0], 1'b0};
                vif.miso = sr[7];
            end

            wait (vif.cs_n === 2'b11);
            vif.miso = 1'b1;
        end
        //cfg
        // forever begin
        //     wait (vif.cs_n !== 2'b11);
        //     //새 응답값으로
        //     sr = $urandom_range(0, 255);
        //     rsp = spi_bus_item::type_id::create("rsp");
        //     rsp.miso_byte = sr;
        //     sr = (vif.cs_n[0] == 1'b0) ? cfg.slave0_resp : cfg.slave1_resp;
        //     //cfg -> coverage 활용하면서 삭제
        //     vif.miso = sr[7];

        //     if (cfg.cpha) begin
        //         if (shift_on_rise) @(posedge vif.sclk);
        //         else @(negedge vif.sclk);
        //     end

        //     for (int i = 0; i < 7; i++) begin
        //         if (shift_on_rise) @(posedge vif.sclk);
        //         else @(negedge vif.sclk);
        //         sr = {sr[6:0], 1'b0};
        //         vif.miso = sr[7];
        //     end
        //     wait (vif.cs_n === 2'b11);
        //     vif.miso = 1'b1;
        // end
    endtask  //run_phase
endclass  //spi_slave_driver extends superClass

class spi_bus_monitor extends uvm_monitor;
    `uvm_component_utils(spi_bus_monitor)
    virtual spi_bus_if vif;
    spi_cfg cfg;
    uvm_analysis_port #(spi_bus_item) ap;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction  //new()

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_bus_if)::get(this, "", "bus_vif", vif))
            `uvm_fatal(get_name(), "bus_monitor_vif not found")
        if (!uvm_config_db#(spi_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal(get_name(), "bus_monitor cfg not found")
    endfunction

    task run_phase(uvm_phase phase);
        spi_bus_item item;
        bit [7:0] mosi_sr, miso_sr;
        bit sample_on_rise;

        //sample_on_rise = ~(cfg.cpol ^ cfg.cpha); //forever 안으로

        wait (vif.cs_n === 2'b11);  // ★ 유효한 idle 상태가 될 때까지 대기

        forever begin
            wait (vif.cs_n !== 2'b11);
            sample_on_rise = ~(cfg.cpol ^ cfg.cpha);  //이동
            item = spi_bus_item::type_id::create("item");
            item.cs_active = ~vif.cs_n;
            mosi_sr = '0;
            miso_sr = '0;
            for (int i = 0; i < 8; i++) begin
                if (sample_on_rise) @(posedge vif.sclk);
                else @(negedge vif.sclk);
                mosi_sr = {mosi_sr[6:0], vif.mosi};
                miso_sr = {miso_sr[6:0], vif.miso};
            end
            item.mosi_byte = mosi_sr;
            item.miso_byte = miso_sr;
            `uvm_info("BUS_MON", $sformatf(
                      "decoded mosi=0x%02h miso=0x%02h cs=%02b", item.mosi_byte, item.miso_byte, item.cs_active),
                      UVM_MEDIUM)
            ap.write(item);
            wait (vif.cs_n === 2'b11);
        end
    endtask  //run_phase
endclass  //spi_bus_monitor extends uvm_monitor


`uvm_analysis_imp_decl(_ctrl)
`uvm_analysis_imp_decl(_bus)
`uvm_analysis_imp_decl(_rsp)  // slave response stream - scoreboard로 가는 판정

class spi_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(spi_scoreboard)
    uvm_analysis_imp_ctrl #(spi_seq_item, spi_scoreboard) ctrl_imp;
    uvm_analysis_imp_bus #(spi_bus_item, spi_scoreboard) bus_imp;
    uvm_analysis_imp_rsp #(spi_bus_item, spi_scoreboard) rsp_imp;

    spi_cfg cfg;
    int pass_cnt, fail_cnt;
    spi_seq_item ctrl_q[$];
    spi_bus_item bus_q[$];
    spi_bus_item rsp_q[$];

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ctrl_imp = new("ctrl_imp", this);
        bus_imp  = new("bus_imp", this);
        rsp_imp  = new("rsp_imp", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(spi_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal(get_name(), "cfg not found")
    endfunction

    function void write_ctrl(spi_seq_item t);
        ctrl_q.push_back(t);
        try_compare();
    endfunction

    function void write_bus(spi_bus_item t);
        bus_q.push_back(t);
        try_compare();
    endfunction

    function void write_rsp(spi_bus_item t);
        rsp_q.push_back(t);
        try_compare();
    endfunction  // ap


    function void try_compare();
        spi_seq_item c;
        spi_bus_item b;
        spi_bus_item r;
        //bit [7:0] exp_resp; //randcase활용과 함께 안쓰임
        if (ctrl_q.size() == 0 || bus_q.size() == 0 || rsp_q.size() == 0) return;  //rsp_q ap
        c = ctrl_q.pop_front();
        b = bus_q.pop_front();
        r = rsp_q.pop_front();  //ap
        //exp_resp = (c.slave_sel == 2'b01) ? cfg.slave0_resp : cfg.slave1_resp; //기대값을 cfg에서 계산해내는 변수
        //이건 **"설정 파일에 뭐라고 적혀 있는지"**를 기대값으로 쓰는 것인데
        //정확히는 **"slave가 실제로 무엇을 보냈는지"**여야 합니다.
        //지금은 둘이 항상 같아서 문제가 안 드러날 뿐 -> "slave_driver"를 개선(ap)추가

        do_check("TX_SERIALIZE", c.tx_data, b.mosi_byte);  // ① 시퀸스가 시킨값, 핀에서 복원한값
        do_check("RX_DESERIALIZE", b.miso_byte, c.rx_data);  // ② 핀에서 복원한 값, DUT가 복원한 값
        //do_check("END_TO_END", exp_resp, c.rx_data);  // ③ cfg -> 실제 보낸 값
        //변경 : cfg 대신 slave_driver가 알려준 miso_byte를 그대로 사용
        do_check("END_TO_END", r.miso_byte, c.rx_data);  //slave가 보낸것, DUT가 복원한 값 비교.

    endfunction

    function void do_check(string tag, bit [7:0] exp, bit [7:0] got);
        if (exp === got) begin
            pass_cnt++;
            `uvm_info("SCB", $sformatf("%s MATCH exp=0x%02h got=0x%02h", tag, exp, got), UVM_HIGH)
        end else begin
            fail_cnt++;
            `uvm_error("SCB", $sformatf("%s MISMATCH exp=0x%02h got=0x%02h", tag, exp, got))
        end
    endfunction

    function void report_phase(uvm_phase phase);
        //ap - queue 동기화 확인
        if (ctrl_q.size() != 0 || bus_q.size() != 0 || rsp_q.size() != 0)
            `uvm_warning("SCB", $sformatf(
                         "큐 잔여: ctrl=%0d bus=%0d rsp=%0d", ctrl_q.size(), bus_q.size(), rsp_q.size()))
        `uvm_info("SCB", $sformatf("\n=== Scoreboard ===\n  PASS: %0d\n  FAIL: %0d", pass_cnt, fail_cnt), UVM_NONE)
    endfunction
endclass  //spi_scoreboard extends uvm_scoreboard

class spi_coverage extends uvm_subscriber #(spi_seq_item);
    `uvm_component_utils(spi_coverage)

    spi_cfg cfg;
    spi_seq_item curr_item;

    //coverage gruoup 내 예약어 : item, option, type_option, bins, binsof
    covergroup cg_spi_txn;
        cp_mode: coverpoint {
            cfg.cpol, cfg.cpha
        } {
            bins mode0 = {2'b00}; bins mode1 = {2'b01}; bins mode2 = {2'b10}; bins mode3 = {2'b11};
        }
        cp_slave: coverpoint curr_item.slave_sel {
            bins s0 = {2'b01}; bins s1 = {2'b10}; illegal_bins invalid = {2'b00, 2'b11};
        }
        cp_tx: coverpoint curr_item.tx_data {
            bins zero = {8'h00}; bins max = {8'hFF}; bins others[4] = {[8'h01 : 8'hFE]};
        }
        cp_rx: coverpoint curr_item.rx_data {
            bins zero = {8'h00}; bins max = {8'hFF}; bins others[4] = {[8'h01 : 8'hFE]};
        }
        cx_mode_slave: cross cp_mode, cp_slave;
        cx_mode_tx: cross cp_mode, cp_tx;
    endgroup

    function new(string name, uvm_component parent);
        super.new(name, parent);
        cg_spi_txn = new();  //covergruop은 생성자에서 new() 필요
    endfunction  //new()

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(spi_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal(get_name(), "cfg not found")
    endfunction

    function void write(spi_seq_item t);
        curr_item = t;  //멤버에 저장
        cg_spi_txn.sample();  // ★ 샘플 → covergroup이 curr_item.xxx를 읽음
    endfunction

    function void report_phase(uvm_phase phase);
        `uvm_info("COV", $sformatf(
                  "\n=== Coverage ===\n  total         : %.1f%%\n  cp_mode       : %.1f%%\n  cp_slave      : %.1f%%\n  cp_tx         : %.1f%%\n  cp_rx         : %.1f%%\n  cx_mode_slave : %.1f%%\n  cx_mode_tx    : %.1f%%",
                  cg_spi_txn.get_coverage(),
                  cg_spi_txn.cp_mode.get_coverage(),
                  cg_spi_txn.cp_slave.get_coverage(),
                  cg_spi_txn.cp_tx.get_coverage(),
                  cg_spi_txn.cp_rx.get_coverage(),
                  cg_spi_txn.cx_mode_slave.get_coverage(),
                  cg_spi_txn.cx_mode_tx.get_coverage()
                  ), UVM_NONE)
    endfunction

endclass  //spi_coverage extends uvm_subsriber #(spi_seq_item)
//write()에서 받은 트랜잭션을 멤버 변수 item에 대입한 뒤 sample()을 호출
//covergroup은 sample 시점에 curr_item.slave_sel 등을 읽어간다
//uvm_subscriber #(T)를 쓰면 analysis_export가 자동으로 생기고, write(T t) 구현만 하면 됨

class spi_bus_coverage extends uvm_subscriber #(spi_bus_item);
    `uvm_component_utils(spi_bus_coverage)

    virtual spi_bus_if vif;

    // ── 매 클럭 샘플링용 ─────────────────────
    bit s_sclk, s_cs0, s_cs1;

    covergroup cg_bus_signal;
        cp_sclk: coverpoint s_sclk {bins low = {0}; bins high = {1};}
        cp_cs0: coverpoint s_cs0 {bins asserted = {0}; bins idle = {1};}
        cp_cs1: coverpoint s_cs1 {bins asserted = {0}; bins idle = {1};}
        cx_cs: cross cp_cs0, cp_cs1{
            // ★ SVA와 이중 안전망 — 둘 다 asserted면 불법
            illegal_bins both = binsof (cp_cs0.asserted) && binsof (cp_cs1.asserted);
        }
    endgroup

    // ── 트랜잭션 전환 패턴용 ─────────────────
    bit [1:0] s_prev_cs,  s_curr_cs;
    bit       s_has_prev;

    covergroup cg_bus_txn;
        cp_transition: coverpoint {
            s_prev_cs, s_curr_cs
        } iff (s_has_prev) {
            bins s0_to_s0 = {4'b0101}; bins s0_to_s1 = {4'b0110}; bins s1_to_s0 = {4'b1001}; bins s1_to_s1 = {4'b1010};
        }
    endgroup

    function new(string name, uvm_component parent);
        super.new(name, parent);
        cg_bus_signal = new();
        cg_bus_txn    = new();
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual spi_bus_if)::get(this, "", "bus_vif", vif))
            `uvm_fatal(get_name(), "bus_vif not found")
    endfunction

    // 버스 신호는 매 클럭 샘플링
    task run_phase(uvm_phase phase);
        wait (vif.cs_n === 2'b11);  // X값 회피
        forever begin
            @(posedge vif.clk);
            s_sclk = vif.sclk;
            s_cs0  = vif.cs_n[0];
            s_cs1  = vif.cs_n[1];
            cg_bus_signal.sample();
        end
    endtask

    // 트랜잭션은 bus_mon이 준 것으로
    function void write(spi_bus_item t);
        s_curr_cs = t.cs_active;
        if (s_has_prev) cg_bus_txn.sample();
        s_prev_cs  = s_curr_cs;
        s_has_prev = 1'b1;
    endfunction

    function void report_phase(uvm_phase phase);
        `uvm_info("BUS_COV", $sformatf(
                  "\n=== Bus Coverage ===\n  cg_bus_signal : %.1f%%\n  cg_bus_txn    : %.1f%%",
                  cg_bus_signal.get_coverage(),
                  cg_bus_txn.get_coverage()
                  ), UVM_NONE)
    endfunction
endclass

class spi_env extends uvm_env;
    `uvm_component_utils(spi_env)
    spi_cfg cfg;
    spi_ctrl_agent ctrl_agt;
    spi_slave_driver slave_drv;
    spi_bus_monitor bus_mon;
    spi_scoreboard scb;
    spi_coverage cov;
    spi_bus_coverage bus_cov;


    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(spi_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal(get_name(), "cfg not found")
        scb = spi_scoreboard::type_id::create("scb", this);
        ctrl_agt = spi_ctrl_agent::type_id::create("ctrl_agt", this);
        slave_drv = spi_slave_driver::type_id::create("slave_drv", this);
        bus_mon = spi_bus_monitor::type_id::create("bus_mon", this);
        cov = spi_coverage::type_id::create("cov", this);
        bus_cov = spi_bus_coverage::type_id::create("bus_cov", this);
    endfunction

    //connect_phase
    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        ctrl_agt.mon.ap.connect(scb.ctrl_imp);  //결과 판정용
        bus_mon.ap.connect(scb.bus_imp);
        slave_drv.ap.connect(scb.rsp_imp);  //ap
        ctrl_agt.mon.ap.connect(cov.analysis_export);  //coverage - scoreboard와 동일한 ap에 연결
                                                       //테스트 완결성 집계용
        bus_mon.ap.connect(bus_cov.analysis_export);
    endfunction
endclass

//ctrl_mon.ap는 이미 scb.ctrl_imp에 연결돼 있지만, analysis port는 1:N fan-out이라 여러 곳에 동시 연결

// //spi base test가 모든걸 다하는 구조. (부모에서 다하고 특정 조건만 자식에서 할당)
// //-> base test는 틀만 갖고 run_phase와 같은건 자식이 상속받고 진행.
// class spi_base_test extends uvm_test;
//     `uvm_component_utils(spi_base_test)
//     spi_env env;
//     spi_cfg cfg;

//     function new(string name, uvm_component parent);
//         super.new(name, parent);
//     endfunction

//     function void build_phase(uvm_phase phase);
//         super.build_phase(phase);
//         cfg = spi_cfg::type_id::create("cfg");
//         if (!cfg.randomize())
//             `uvm_fatal(get_name(),
//                        "cfg randomize failed");  //randomize가 한번만 됨. -> mode 한개만 관찰 가능
//         //if(!cfg.randomize() with {cpol == 0; cpha ==0;}) `uvm_fatal(get_name(), "cfg randomize failed"); //00
//         //if(!cfg.randomize() with {cpol == 0; cpha ==1;}) `uvm_fatal(get_name(), "cfg randomize failed"); //01
//         //if(!cfg.randomize() with {cpol == 1; cpha ==0;}) `uvm_fatal(get_name(), "cfg randomize failed"); //10
//         //if(!cfg.randomize() with {cpol == 1; cpha == 1;}) `uvm_fatal(get_name(), "cfg randomize failed");  //11
//         uvm_config_db#(spi_cfg)::set(this, "*", "cfg", cfg);
//         env = spi_env::type_id::create("env", this);
//     endfunction

//     task run_phase(uvm_phase phase);
//         spi_base_seq seq;
//         spi_boundary_seq bseq;
//         phase.raise_objection(this);

//         for (int m = 0; m < 4; m++) begin
//             cfg.cpol = m[1];  // 00, 01, 10, 11 순회
//             cfg.cpha = m[0];
//             `uvm_info(get_name(), $sformatf("=== Mode %0d (cpol=%0b cpha=%0b) ===", m, cfg.cpol, cfg.cpha), UVM_NONE)

//             seq = spi_base_seq::type_id::create($sformatf("seq_m%0d", m));
//             if (!seq.randomize()) `uvm_fatal(get_name(), "seq randomize failed");
//             seq.start(env.ctrl_agt.sqr);

//             bseq = spi_boundary_seq::type_id::create($sformatf("bseq_m%0d", m));
//             bseq.start(env.ctrl_agt.sqr);
//         end

//         // seq = spi_base_seq::type_id::create("seq");
//         // if (!seq.randomize()) `uvm_fatal(get_name(), "seq randomize failed");
//         // seq.start(env.ctrl_agt.sqr);

//         phase.drop_objection(this);
//     endtask  //run_phase
// endclass

class spi_base_test extends uvm_test;
    `uvm_component_utils(spi_base_test)
    spi_env env;
    spi_cfg cfg;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        cfg = spi_cfg::type_id::create("cfg");
        if (!cfg.randomize()) `uvm_fatal(get_name(), "cfg randomize failed");
        uvm_config_db#(spi_cfg)::set(this, "*", "cfg", cfg);
        env = spi_env::type_id::create("env", this);
    endfunction

    // ── 공통 헬퍼: 각 test가 조립해서 쓸 부품들 ─────────────

    virtual task run_random(int unsigned n, string tag = "rnd");
        spi_base_seq seq;
        seq = spi_base_seq::type_id::create(tag);
        if (!seq.randomize() with {num_txn == n;})
            `uvm_fatal(get_name(), "seq randomize failed");
        seq.start(env.ctrl_agt.sqr);
    endtask

    virtual task run_boundary(string tag = "bnd");
        spi_boundary_seq bseq;
        bseq = spi_boundary_seq::type_id::create(tag);
        bseq.start(env.ctrl_agt.sqr);
    endtask

    virtual task set_mode(int m);
        cfg.cpol = m[1];
        cfg.cpha = m[0];
        `uvm_info(get_name(), $sformatf("=== Mode %0d (cpol=%0b cpha=%0b) ===",
                  m, cfg.cpol, cfg.cpha), UVM_NONE)
    endtask

    // ── run_phase는 비워둠 — 시나리오는 자식이 정의 ──────────
    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        `uvm_warning(get_name(), "spi_base_test는 뼈대입니다. 하위 test를 지정하세요.")
        phase.drop_objection(this);
    endtask
endclass

//테스트 라이브러리
// ── 빠른 확인용 — 코드 고칠 때마다 돌리는 것 ────────────────
class spi_smoke_test extends spi_base_test;
    `uvm_component_utils(spi_smoke_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        set_mode(0);
        run_random(5);           // 트랜잭션 5개만
        phase.drop_objection(this);
    endtask
endclass

// ── 커버리지 달성용 — 지금 base가 하던 것 ────────────────────
class spi_mode_test extends spi_base_test;
    `uvm_component_utils(spi_mode_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        for (int m = 0; m < 4; m++) begin
            set_mode(m);
            run_random(30, $sformatf("rnd_m%0d", m));
            run_boundary($sformatf("bnd_m%0d", m));
        end
        phase.drop_objection(this);
    endtask
endclass

// ── 다중 시드 버그 사냥용 — 모드도 랜덤 ──────────────────────
class spi_random_test extends spi_base_test;
    `uvm_component_utils(spi_random_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        repeat (8) begin                       // 8개 구간
            set_mode($urandom_range(0, 3));    // ★ 모드를 매번 랜덤하게
            run_random($urandom_range(10, 30));
        end
        phase.drop_objection(this);
    endtask
endclass

// ── 경계값 집중 ──────────────────────────────────────────────
class spi_boundary_test extends spi_base_test;
    `uvm_component_utils(spi_boundary_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        for (int m = 0; m < 4; m++) begin
            set_mode(m);
            run_boundary($sformatf("bnd_m%0d", m));
        end
        phase.drop_objection(this);
    endtask
endclass

class spi_negative_test extends spi_base_test;
    `uvm_component_utils(spi_negative_test)

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        // ★ 이 한 줄이 전부 — env/agent 코드는 손대지 않음
        spi_driver::type_id::set_type_override(spi_evil_driver::get_type());
        super.build_phase(phase);
        // cfg.evil_mode = EVIL_SEL_CHANGE;  // 시나리오 선택
        cfg.evil_mode = EVIL_SEL_BOTH;  // 시나리오 선택
        // cfg.evil_mode = EVIL_SEL_NONE;  // 시나리오 선택
    endfunction

    //run_phase 추가
    task run_phase(uvm_phase phase);        // ★ 추가 — 짧게
        phase.raise_objection(this);
        set_mode(0);
        run_random(10);                     // 10번만 주입해도 충분
        phase.drop_objection(this);
    endtask
endclass

module tb_top;
    bit clk;
    always #5 clk = ~clk;

    spi_ctrl_if ctrl_if (.clk(clk));
    spi_bus_if bus_if (.clk(clk));

    spi_master u_master (
        .clk      (clk),
        .reset    (ctrl_if.reset),
        .slave_sel(ctrl_if.slave_sel),
        .start    (ctrl_if.start),
        .tx_data  (ctrl_if.tx_data),
        .tx_ready (ctrl_if.tx_ready),
        .rx_data  (ctrl_if.rx_data),
        .done     (ctrl_if.done),
        .cpol     (ctrl_if.cpol),
        .cpha     (ctrl_if.cpha),
        .sclk     (bus_if.sclk),
        .mosi     (bus_if.mosi),
        .miso     (bus_if.miso),        // ← UVM slave driver가 구동할 것
        .cs_n     (bus_if.cs_n)
    );

    // assertion 참조용 신호 연결
    assign bus_if.reset = ctrl_if.reset;
    assign bus_if.cpol  = ctrl_if.cpol;
    assign bus_if.cpha  = ctrl_if.cpha;

    initial begin
        ctrl_if.reset = 1'd1;
        #200 ctrl_if.reset = 1'd0;
    end
    initial begin
        uvm_config_db#(virtual spi_ctrl_if)::set(null, "*", "ctrl_vif", ctrl_if);
        uvm_config_db#(virtual spi_bus_if)::set(null, "*", "bus_vif", bus_if);
        run_test("spi_mode_test");    // ← 기본값. +UVM_TESTNAME이 있으면 그게 우선
    end

    initial begin
        #200_000_000;
        `uvm_fatal("TB_TOP", "SIMULATION TIMEOUT");
    end

endmodule

