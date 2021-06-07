`include "alu_control.v"
//`include "alu.v"
`timescale 1s/100ms

module top();
	reg clk = 0;

	reg[31:0] A, B;
	wire[31:0] ALUOut;
	wire Branch_Enable;

	//alu_control interface
	reg[3:0] FuncCode;
	reg[6:0] Opcode;

	//alu aluctl interface - changed bit length
	wire[6:0] AluCtl_wire;

	ALUControl aluCtrl_inst(
		.FuncCode(FuncCode),
		.ALUCtl(AluCtl_wire),
		.Opcode(Opcode)
	);

	alu alu_inst(
		.ALUctl(AluCtl_wire),
		.A(A),
		.B(B),
		.ALUOut(ALUOut),
		.Branch_Enable(Branch_Enable)
	);

//simulation
always
 #0.5 clk = ~clk;

initial begin
	$dumpfile ("simulationtest2.vcd");
 	$dumpvars;

 	//reg[31:0] A, B;
 	//reg[3:0] FuncCode; //bit 32 + bit 14:12
	//reg[6:0] Opcode; //bits 6:0

 	A = 32'b0;
 	B = 32'b0;
 	FuncCode = 4'b0;
 	Opcode = 7'b0;

 	#5

 	//simulate AND instruction
 	A = 32'b00001111;
 	B = 32'b01010101;
 	FuncCode = 4'b0111;
 	Opcode = 7'b0110011;

 	#5

 	//simulate OR instruction
 	A = 32'b00001111;
 	B = 32'b01010101;
 	FuncCode = 4'b0110;
 	Opcode = 7'b0110011;

 	#5

 	//simulate ADD instruction
 	A = 32'd10000;
 	B = 32'd0111;
 	FuncCode = 4'b0000;
 	Opcode = 7'b0110011;
	
 	#5

 	//simulate SUB instruction
 	A = 32'd10000;
 	B = 32'd0111;
 	FuncCode = 4'b1000;
 	Opcode = 7'b0110011;

 	#5

 	//simulate SLT instuction
 	A = 32'b0;
 	B = 32'b10;
 	FuncCode = 4'b0010;
 	Opcode = 7'b0110011;

 	#5

 	//simulate SRL instruction
 	A = 32'b10000;
 	B = 32'b10;
 	FuncCode = 4'b0101;
 	Opcode = 7'b0110011;

 	#5

 	//simulate SRA instruction
 	A = 32'b1000;
 	B = 32'b1;
 	FuncCode = 4'b1101;
 	Opcode = 7'b0110011;

 	#5

 	//simulate SLL instruction
 	A = 32'b10;
 	B = 32'b10;
 	FuncCode = 4'b0001;
 	Opcode = 7'b0110011;

 	#5

 	//simulate XOR instruction
 	A = 32'b01010101;
 	B = 32'b11111111;
 	FuncCode = 4'b0100;
 	Opcode = 7'b0110011;

	#5

	//simulate SLL: result should be 100000 = h20
	A = 32'b10;
	B = 32'b10000;
	FuncCode = 4'b0001;
	Opcode = 7'b0110011;

	#5

	//simulate SLL: result should be 32 bits 1000...0
	A = 32'b1;
	B = 32'b11111;
	FuncCode = 4'b0001;
	Opcode = 7'b0110011;

	#5

	// SLT
	A = 32'b10;
	B = 32'b0;
	FuncCode = 4'b0010;
	Opcode = 7'b0110011;

	#5

	//simulate CSRRS instruction (copy of OR with 1 in opcode msb)
        A = 32'b00001111;
        B = 32'b01010101;
	FuncCode = 4'b0110;
	Opcode = 7'b1110011;

	#5
	//simulate SRA instruction - testing negative 2's comp numbers
        A = 32'hFFFFFFFF;
	B = 32'b10;
	FuncCode = 4'b1101;
	Opcode = 7'b0110011;

	#5
	//simulate BLT branch instructions
	A = 32'b1;
	B = 32'b10;
	FuncCode = 4'b0100;
	Opcode = 7'b1100011;

 	#50

 	$finish;
end

endmodule
