/*
* Simulation for adder module.
*/
`include "adder.v"

module addersim(clk, out);
	input		clk;
	output[31:0]	out;

	wire[31:0]	input1 = 32'b1;
	wire[31:0]	input2 = 32'b1;
	wire		addsubsel = 1'b0;


	adder adder_inst(.input1(input1), .input2(input2), .clk(clk), .out(out));
endmodule
