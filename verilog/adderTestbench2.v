module top();
	reg[31:0] input1, input2;
	wire[31:0] adder_out;
	
	adder adder_inst(
		.input1(input1),
		.input2(input2),
		.out(adder_out)
	);
//simulation
initial begin
	$dumpfile ("adsim.vcd");
	$dumpvars;

	input1 = 32'b0;
	input2 = 32'b0;

	#5

	input1 = 32'b11;
	input2 = 32'b101;

	#5

	input1 = 32'b1000;
	input2 = 32'b1;

	$finish;
end

endmodule
