`timescale 1ns / 1ps

module tb_spi ();
    logic       clk;
    logic       reset;
    logic       start;
    logic [7:0] tx_data;
    logic       tx_ready;
    logic [7:0] rx_data;
    logic       done;
    logic       sclk;
    logic       cpol;
    logic       cpha;
    // logic       mosi;
    // logic       miso;
    logic       loop_wire;
    logic       cs_n;

    // 슬레이브 내부 신호
    logic [7:0] slave_tx_data;
    logic       slave_tx_ready;
    logic [7:0] slave_rx_data;
    logic       slave_done;

    spi_master dut (
        .clk(clk),
        .reset(reset),
        .start(start),
        .tx_data(tx_data),
        .tx_ready(tx_ready),
        .rx_data(rx_data),
        .done(done),
        .cpol(cpol),
        .cpha(cpha),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso),
        .cs_n(cs_n)
    );

    spi_slave dut_slave (
        .clk(clk),
        .reset(reset),
        .tx_data(slave_tx_data),
        .tx_ready(slave_tx_ready),
        .rx_data(slave_rx_data),
        .done(slave_done),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso),
        .cs_n(cs_n),
        .cpol(cpol),
        .cpha(cpha)
    );

    always #5 clk = ~clk;
    initial begin
        clk   = 0;
        reset = 1;
        #10;
        reset = 0;
    end

    task spi_mode(bit pol, bit pha);
        @(posedge clk);
        cpol = pol;
        cpha = pha;
        @(posedge clk);
    endtask  //spi_mode

    task spi_transfer(logic [7:0] master_send, logic [7:0] slave_send);
        // 슬레이브 송신 데이터 세팅 (CS_N=1 구간에 프리로드됨)
        slave_tx_data = slave_send;
        @(posedge clk);
        wait (tx_ready);
        start   = 1;
        tx_data = master_send;
        @(posedge clk);
        start = 0;

        // 마스터 수신 완료 대기
        wait (done);
        @(posedge clk);

        // 결과 출력
        $display(
            "Mode(CPOL=%0d,CPHA=%0d) | Master sent: 0x%02h  Slave received: 0x%02h | Slave sent: 0x%02h  Master received: 0x%02h",
            cpol, cpha, master_send, slave_rx_data, slave_send, rx_data);
    endtask

    task spi_write(logic [7:0] data);
        @(posedge clk);
        wait (tx_ready);
        start   = 1;
        tx_data = data;
        @(posedge clk);
        start = 0;
        wait (done);
        @(posedge clk);
    endtask

    // initial begin
    //     repeat (5) @(posedge clk);
    //     spi_mode(0, 0);
    //     spi_write(8'haa);
    //     @(posedge clk);
    //     @(posedge clk);
    //     spi_mode(0, 1);
    //     spi_write(8'h55);
    //     @(posedge clk);
    //     @(posedge clk);
    //     spi_mode(1, 0);
    //     spi_write(8'h0f);
    //     @(posedge clk);
    //     @(posedge clk);
    //     spi_mode(1, 1);
    //     spi_write(8'hf0);
    //     @(posedge clk);
    //     @(posedge clk);
    //     $finish;
    // end

    initial begin
        slave_tx_data = 8'h00;
        repeat (5) @(posedge clk);

        spi_mode(0, 0);
        spi_transfer(8'hAA, 8'h11);
        repeat (2) @(posedge clk);

        spi_mode(0, 1);
        spi_transfer(8'h55, 8'h22);
        repeat (2) @(posedge clk);

        spi_mode(1, 0);
        spi_transfer(8'h0F, 8'h33);
        repeat (2) @(posedge clk);

        spi_mode(1, 1);
        spi_transfer(8'hF0, 8'h44);
        repeat (2) @(posedge clk);

        $finish;
    end
endmodule

