`timescale 1ns / 1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

interface counter_if (
    input logic clk
);
    logic       reset_n;
    logic       enable;
    logic [3:0] count;
    //driver clocing block
    clocking drv_cb @(posedge clk);
        default input #1step output #0;
        output reset_n;
        output enable;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input reset_n;
        input enable;
        input count;
    endclocking

    modport drv_mp(clocking drv_cb, input clk);
    modport mon_mp(clocking mon_cb, input clk);

endinterface  //counter_if

class counter_seq_item extends uvm_sequence_item;
    rand bit       reset_n;
    rand bit       enable;
    rand int       cycles;
    logic    [3:0] count;

    constraint cycles_c {cycles inside {[1 : 20]};}

    `uvm_object_utils_begin(counter_seq_item)
        `uvm_field_int(reset_n, UVM_ALL_ON)
        `uvm_field_int(enable, UVM_ALL_ON)
        `uvm_field_int(cycles, UVM_ALL_ON)
        `uvm_field_int(count, UVM_ALL_ON)
    `uvm_object_utils_end

    function new(string name = "counter_seq_item");
        super.new(name);
    endfunction  //new()

    function string convert2string();
        return $sformatf("reset_n=%0b, enable = %0b, cycles = %0b, count = %0h", reset_n, enable, cycles, count);

    endfunction
endclass  //counter_seq_item extends uvm_sequence_item

class counter_reset_seq extends uvm_sequence #(counter_seq_item);
    `uvm_object_utils(counter_reset_seq)

    function new(string name = "counter_reset_seq");
        super.new(name);
    endfunction  //new()

    virtual task body();
        counter_seq_item item;
        item = counter_seq_item::type_id::create("item");
        start_item(item);
        item.reset_n = 0;
        item.enable  = 0;
        item.cycles  = 2;
        finish_item(item);
        `uvm_info(get_type_name(), "reset done!", UVM_MEDIUM)
    endtask  //
endclass  //counter_reset_seq extends uvm_sequence #(counter_seq_item)

class counter_count_seq extends uvm_sequence #(counter_seq_item);
    `uvm_object_utils(counter_count_seq)

    int num_transactions;

    function new(string name = "counter_count_seq");
        super.new(name);
    endfunction  //new()

    virtual task body();
        counter_seq_item item;
        for (int i = 0; i < num_transactions; i++) begin
            item = counter_seq_item::type_id::create($sformatf("item_%0d", i));
            //sequencer에게 보내는 작업.
            start_item(item);
            if (!item.randomize() with {
                    reset_n == 1;
                    enable == 1;
                    cycles inside {[1 : 5]};
                })
                `uvm_fatal(get_type_name(), "randomize faild!");
            finish_item(item);
            `uvm_info(get_type_name(), $sformatf("[%0d/%0d] %s", i + 1, num_transactions, item.convert2string()),
                      UVM_MEDIUM)
        end
    endtask  //
endclass  //counter_reset_seq extends uvm_sequence #(counter_seq_item)


class counter_master_seq extends uvm_sequence #(counter_seq_item);
    `uvm_object_utils(counter_master_seq)

    function new(string name = "counter_master_seq");
        super.new(name);
    endfunction  //new()

    virtual task body();
        counter_reset_seq reset_seq;
        counter_count_seq count_seq;

        `uvm_info(get_type_name(), "===senario 1 : Reset ===", UVM_MEDIUM)
        reset_seq = counter_reset_seq::type_id::create("reset_seq");
        reset_seq.start(m_sequencer);  // m_sequencer는 내부에 미리 정의 되어있음.

        `uvm_info(get_type_name(), "===senario 2 : Count ===", UVM_MEDIUM)
        count_seq = counter_count_seq::type_id::create("count_seq");
        count_seq.num_transactions = 5;
        count_seq.start(m_sequencer);

        `uvm_info(get_type_name(), "===Master Sequence Done! ===", UVM_MEDIUM)
    endtask  //
endclass  //counter_reset_seq extends uvm_sequence #(counter_seq_item)

//component
//이름만 바꿔준것과 동일
class counter_sequencer extends uvm_sequencer #(counter_seq_item);
    `uvm_component_utils(counter_sequencer)

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()
endclass  //counter_sequencer extends uvm_sequencer #(counter_seq_item)

//TLM 통신을 해서 item 형태가 동일하다.
//sequencer item에 대한 이름이 들어간다.
class counter_driver extends uvm_driver #(counter_seq_item);
    `uvm_component_utils(counter_driver)

    virtual counter_if vif;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual counter_if)::get(this, "", "vif", vif)) `uvm_fatal(get_type_name(), "vif not found")
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
    endfunction

    virtual task run_phase(uvm_phase phase);
        counter_seq_item item;
        forever begin
            seq_item_port.get_next_item(item);
            drive_item(item);
            seq_item_port.item_done();
        end
    endtask  //

    virtual task drive_item(counter_seq_item item);
        vif.drv_cb.reset_n <= item.reset_n;
        vif.drv_cb.enable  <= item.enable;
        repeat (item.cycles) @(vif.drv_cb);  //@(posedge vif.clk);
    endtask  //drive_item
endclass  //counter_sequencer extends uvm_sequencer #(counter_seq_item)

class counter_monitor extends uvm_monitor;
    `uvm_component_utils(counter_monitor)

    virtual counter_if vif;
    counter_seq_item item;
    int expected_count;

    uvm_analysis_port #(counter_seq_item) ap;  //analysis port

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual counter_if)::get(this, "", "vif", vif)) `uvm_fatal(get_type_name(), "vif not found")
        expected_count = 0;
        ap = new("ap", this);  // ap instance 생성
        item = counter_seq_item::type_id::create("item");
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
    endfunction

    virtual task run_phase(uvm_phase phase);
        forever begin
            @(vif.mon_cb);  //@(posedge vif.clk);
            item.reset_n = vif.mon_cb.reset_n;
            item.enable  = vif.mon_cb.enable;
            item.count   = vif.mon_cb.count;
            `uvm_info(get_type_name(), $sformatf("mon_send %s", item.convert2string()), UVM_MEDIUM);
            ap.write(item);

            // if (!vif.mon_cb.reset_n) begin
            //     expected_count = 0;
            // end else if (vif.mon_cb.enable) begin
            //     if (vif.mon_cb.count !== expected_count) begin
            //         `uvm_error(get_type_name(), $sformatf(
            //                    "Mismatced! expect = %0d, count = %0d", expected_count, vif.mon_cb.count));
            //     end else begin
            //         `uvm_info(get_type_name(), $sformatf(
            //                   "Matched! expect = %0d, count = %0d", expected_count, vif.mon_cb.count), UVM_MEDIUM);
            //     end
            //     expected_count = (expected_count + 1) % 16;
            // end else begin
            //     //no count
            //     if (vif.mon_cb.count !== expected_count) begin
            //         `uvm_error(get_type_name(), $sformatf(
            //                    "Mismatced! expect = %0d, count = %0d", expected_count, vif.mon_cb.count));
            //     end else begin
            //         `uvm_info(get_type_name(), $sformatf(
            //                   "Matched! expect = %0d, count = %0d", expected_count, vif.mon_cb.count), UVM_MEDIUM);
            //     end
            // end
        end
    endtask  //
endclass  //counter_sequencer extends uvm_sequencer #(counter_seq_item)

class counter_agent extends uvm_agent;
    `uvm_component_utils(counter_agent)

    counter_sequencer sqr;
    counter_driver drv;
    counter_monitor mon;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        sqr = counter_sequencer::type_id::create("sqr", this);
        drv = counter_driver::type_id::create("drv", this);
        mon = counter_monitor::type_id::create("mon", this);
    endfunction

    //driver - sequencer TLM connect
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction

    virtual task run_phase(uvm_phase phase);

    endtask  //
endclass  //counter_sequencer extends uvm_sequencer #(counter_seq_item)

class counter_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(counter_scoreboard)

    //class 이름
    uvm_analysis_imp #(counter_seq_item, counter_scoreboard) ap_imp;  //ap implemenation

    logic [3:0] expect_count;
    int match_count;
    int error_count;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        ap_imp = new("ap_imp", this);
        expect_count = 4'hx;  //count 초기값이 x로 되어있음. 맞춤.
        match_count = 0;
        error_count = 0;
    endfunction

    function logic [3:0] predict(logic reset_n, logic enable, logic [3:0] count);
        if (!reset_n) return 4'h0;
        else if (enable) return count + 1;
        else return count;

    endfunction

    virtual function void write(counter_seq_item item);
        if (expect_count !== item.count) begin
            `uvm_error(get_type_name(),
                       $sformatf("Mismatched ! expect_count =%0d, vif.count = %0d, (reset_n = %0b, enable=%0b)",
                                 expect_count, item.count, item.reset_n, item.enable))
            error_count++;
        end else begin
            `uvm_info(get_type_name(), $sformatf(
                      "MATCH expect_count =%0d, vif.count = %0d, (reset_n = %0b, enable=%0b)",
                      expect_count,
                      item.count,
                      item.reset_n,
                      item.enable
                      ), UVM_MEDIUM)
            match_count++;
        end
        expect_count = predict(item.reset_n, item.enable, expect_count);
    endfunction

    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info(get_type_name(), "=====Scoreboard Summary =====", UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("Total transaction : %0d", match_count + error_count), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("Matches : %0d", match_count), UVM_LOW)
        `uvm_info(get_type_name(), $sformatf("Errors  : %0d", error_count), UVM_LOW)

        if (error_count > 0)
            `uvm_error(get_type_name(), $sformatf("Test Failed - %0d mismatches detected", error_count))
        else `uvm_info(get_type_name(), $sformatf("Test Successed - all transactions matched", match_count), UVM_LOW)

    endfunction
endclass  //counter_scoreboard

class counter_subscriber extends uvm_subscriber #(counter_seq_item);
    `uvm_component_utils(counter_subscriber)

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()

    virtual function void write(counter_seq_item item);
        `uvm_info(get_type_name(), $sformatf("subscriber item %s", item.convert2string()), UVM_LOW)
    endfunction

endclass  //counter_subscriber


class counter_env extends uvm_env;
    `uvm_component_utils(counter_env)

    counter_agent agent;
    counter_scoreboard scb;
    counter_subscriber ssb;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        agent = counter_agent::type_id::create("agent", this);
        scb   = counter_scoreboard::type_id::create("scb", this);
        ssb   = counter_subscriber::type_id::create("ssb", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        //모니터가 부르고 스코어보드가 받음.
        agent.mon.ap.connect(scb.ap_imp);
        //모니터의 포트와 스코어보드 포트 임플리멘테이션과 연결
        //caller.connect(callee)
        agent.mon.ap.connect(ssb.analysis_export);
        //자동으로 불려짐. ssb에 port가 없지만 여기서 자동으로 불려짐.

    endfunction

    virtual task run_phase(uvm_phase phase);

    endtask  //
endclass  //counter_sequencer extends uvm_sequencer #(counter_seq_item)

class counter_test extends uvm_test;
    `uvm_component_utils(counter_test)

    counter_env env;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction  //new()

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = counter_env::type_id::create("env", this);  //instance name, parent
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);

    endfunction

    virtual task run_phase(uvm_phase phase);
        counter_master_seq seq;
        phase.raise_objection(this);
        seq = counter_master_seq::type_id::create("seq");  //seqeunce is not componnent -> no parent
        seq.start(env.agent.sqr);
        #100;
        uvm_top.print_topology();
        phase.drop_objection(this);
    endtask  //
endclass  //counter_sequencer extends uvm_sequencer #(counter_seq_item)


module tb_counter ();
    bit clk;
    counter_if vif (clk);

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end


    counter dut (
        .clk(clk),
        .reset_n(vif.reset_n),
        .enable(vif.enable),
        .count(vif.count)
    );

    initial begin
        uvm_config_db#(virtual counter_if)::set(null, "*", "vif", vif);
        run_test("counter_test");
    end

endmodule
