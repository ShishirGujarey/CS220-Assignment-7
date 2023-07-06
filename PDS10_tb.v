`include "PDS10.v"

module PDS10_tb;

reg clk;
wire [31:0] out;

decode uut (clk,out);

initial begin
clk = 0;
forever #10 clk = ~clk;
end

initial begin
$dumpfile("PDS10_tb.vcd");
$dumpvars(0, PDS10_tb);
#80000 $finish;
end

endmodule