/*
	Authored 2018-2019, Ryan Voo.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"



/*
 *	Description:
 *
 *		This module implements the ALU control unit
 */



module ALUControl(FuncCode, ALUCtl, Opcode);
	input [3:0]		FuncCode; // First 3 bits funct3, last bit is 2nd bit of funct7
	input [6:0]		Opcode;
	output reg [6:0]	ALUCtl;

	/*
	 *	The `initial` statement below uses Yosys's support for nonzero
	 *	initial values:
	 *
	 *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
	 *
	 *	Rather than using this simulation construct (`initial`),
	 *	the design should instead use a reset signal going to
	 *	modules in the design and to thereby set the values.
	 */
	initial begin
		ALUCtl = 7'b0;
	end

	always @(*) begin
		case (Opcode)
			/*
			 *	LUI, U-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_LUI:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_AUIPC;

			/*
			 *	AUIPC, U-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_AUIPC:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_AUIPC;

			/*
			 *	JAL, UJ-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_JAL:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;

			/*
			 *	JALR, I-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_JALR:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;

			/*
			 *	Branch, SB-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_BRANCH:
				case (FuncCode[2:0])
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_LUI: // also AUIPC, JAL, JALR
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BEQ; //BEQ conditions
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BNE; //BNE conditions
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BLT; //BLT conditions
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BGE; //BGE conditions
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BLTU; //BLTU conditions
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_BGEU:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BGEU; //BGEU conditions
					default:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			/*
			 *	Loads, I-Type, all to same code 7'b0000010
			 */
			`kRV32I_INSTRUCTION_OPCODE_LOAD:
				case (FuncCode[2:0])
					`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_LB:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_LB; //LB
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_LH:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_LH; //LH
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_LW:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_LW; //LW
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_LBU:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_LBU; //LBU
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_LHU:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_LHU; //LHU
					default:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			/*
			 *	Stores, S-Type, all to same code 7'b0000010
			 */
			`kRV32I_INSTRUCTION_OPCODE_STORE:
				case (FuncCode[2:0])
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SB:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SB; //SB
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SH:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SB; //SH
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SW:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SB; //SW
					default:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			/*
			 *	Immediate operations, I-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_IMMOP:
				case (FuncCode[2:0]) // funct3
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_ADDI:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ADDI; //7'b0000010 ADDI
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SLTI:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLTI; // 7'b0000111; SLTI
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SLTIU:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLTIU; // 7'b0000111 SLTIU
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_XORI:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_XORI; // 7'b0001000; XORI
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_ORI:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ORI; // 7'b0000001; ORI
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_ANDI:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ANDI; // 7'b0000000; ANDI
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SLLI:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLLI; // 7'b0000101; SLLI
					3'b101:
						case (FuncCode[3]) // second bit funct7
							1'b0:
							  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SRLI; // 7'b0000011; SRLI
							1'b1:
							  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SRAI; // 7'b0000100; SRAI
							default:
								ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
						endcase
					default:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			/*
			 *	ADD SUB & logic shifts, R-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_ALUOP:
				case (FuncCode[2:0]) // funct3
					3'b000:
						case(FuncCode[3]) // testing funct7, bit 2 -- ADD, SUB identical but for this bit 
							1'b0:
							  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ADD; // 7'b0000010; ADD
							1'b1:
							  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SUB; // 7'b0000110; //SUB
							default:
								ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
						endcase
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SLL:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLL; //7'b0000101; SLL
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SLT:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLT; // 7'b0000111; SLT
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_SLTU:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLTU; //7'b0000111; SLTU
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_XOR:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_XOR; //7'b0001000; //XOR
					3'b101:
						case(FuncCode[3]) // testing second bit funct7
							1'b0:
							  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SRL; // 7'b0000011 SRL
							1'b1:
							  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SRA; //7'b0000100; SRA untested
							default:
								ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
						endcase
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_OR:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_OR; //7'b0000001; OR
					`kRV32I_INSTRUCTION_FUNCCODE_2to0_AND:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_AND; //7'b0000000; AND
					default:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			`kRV32I_INSTRUCTION_OPCODE_CSRR: //7'b1110011
				case (FuncCode[1:0]) //use lower 2 bits of funct3 to determine operation since immediate function is handled the same
					2'b01:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_CSRRW;// 7'b0001001; CSRRW/I
					2'b10:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_CSRRS; // 7'b0001010; CSRRS/I
					2'b11:
					  ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_CSRRC; // 7'b0001011; CSRRC/I
					default:
						ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			default:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
		endcase
	end
endmodule
