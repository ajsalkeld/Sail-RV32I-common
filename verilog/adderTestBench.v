/*
* Testbench for the adder.v module.
*/
`include   "adder.v"

module adderTestbench;

	reg oneinput, twoinput, clock, adderout;

	adder adder_inst(.input1(oneinput), .input2(twoinput), .clk(clock), .out(adderout));

	initial begin
		// Conservative max clock speed 12 MHz so use 100 ns
		#1 clk = 0; oneinput = 32'b1; twoinput = 32'b1;

		#1 clk = 1; oneinput = 32'b1; twoinput = 32'b1;

		#1 clk = 0; oneinput = 32'b1; twoinput = 32'b1;
	end
	initial begin
		$dumpfile("adderTestbench.vcd");
		$dumpvars;
	end
endmodule
		
