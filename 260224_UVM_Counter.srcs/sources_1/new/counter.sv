`timescale 1ns / 1ps

module counter (
    input  logic       clk,
    input  logic       reset_n,
    input  logic       enable,
    output logic [3:0] count
);

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
        end else begin
            if (enable) begin
                count <= count + 1;
            end
        end
    end
endmodule
