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
 *		This module implements the ALU for the RV32I.
 */



/*
 *	Not all instructions are fed to the ALU. As a result, the ALUctl
 *	field is only unique across the instructions that are actually
 *	fed to the ALU.
 */
module alu(ALUctl, A, B, ALUOut, Branch_Enable, clk);
   input clk;
   
   input [6:0]		ALUctl;
   input [31:0] 	A;
   input [31:0] 	B;
   output reg [31:0] 	ALUOut;
   output reg		Branch_Enable;

   reg [15:0] 		add_C;
   reg [15:0] 		add_A;
   reg [15:0] 		add_B;
   reg [15:0] 		add_D;
   wire [31:0] 		add_O;

   reg 			add_addsubtop; // 0 to add, 1 to sub
   reg 			add_addsubbot;
 			
   // Default values for simulation purposes
   reg                  CE = 1;
   reg                  DEF = 0;

   // Use DSP for addition, subtraction
   SB_MAC16 i_sbmac16_addsub
     (
      .A(add_A),
      .B(add_B),
      .C(add_C),
      .D(add_D),
      .O(add_O),
      .CLK(clk),
      .CE(CE),
      .AHOLD(DEF),
      .BHOLD(DEF),
      .CHOLD(DEF),
      .DHOLD(DEF),
      .IRSTTOP(DEF),
      .IRSTBOT(DEF),
      .ORSTTOP(DEF),
      .ORSTBOT(DEF),
      .ADDSUBTOP(add_addsubtop),
      .ADDSUBBOT(add_addsubbot),
      .OHOLDTOP(DEF),
      .OHOLDBOT(DEF),
      .OLOADTOP(DEF),
      .OLOADBOT(DEF),
      .CI(DEF),
      .ACCUMCI(DEF),
      .SIGNEXTIN(DEF)
      );

   // add_sub_32_bypassed_unsigned [24:0] = 001_0010000_1010000_0000_0000
   // Read configuration settings [24:0] from left to right while filling the
   // instance parameters.
   defparam i_sbmac16_addsub.B_SIGNED = 1'b0;
   defparam i_sbmac16_addsub.A_SIGNED = 1'b0;
   defparam i_sbmac16_addsub.MODE_8x8 = 1'b1;

   defparam i_sbmac16_addsub.BOTADDSUB_CARRYSELECT = 2'b00;
   defparam i_sbmac16_addsub.BOTADDSUB_UPPERINPUT = 1'b1;
   defparam i_sbmac16_addsub.BOTADDSUB_LOWERINPUT = 2'b00;
   defparam i_sbmac16_addsub.BOTOUTPUT_SELECT = 2'b00;

   defparam i_sbmac16_addsub.TOPADDSUB_CARRYSELECT = 2'b10;
   defparam i_sbmac16_addsub.TOPADDSUB_UPPERINPUT = 1'b1;
   defparam i_sbmac16_addsub.TOPADDSUB_LOWERINPUT = 2'b00;
   defparam i_sbmac16_addsub.TOPOUTPUT_SELECT = 2'b00;

   defparam i_sbmac16_addsub.PIPELINE_16x16_MULT_REG2 = 1'b0;
   defparam i_sbmac16_addsub.PIPELINE_16x16_MULT_REG1 = 1'b0;
   defparam i_sbmac16_addsub.BOT_8x8_MULT_REG = 1'b0;
   defparam i_sbmac16_addsub.TOP_8x8_MULT_REG = 1'b0;
   defparam i_sbmac16_addsub.D_REG = 1'b0;
   defparam i_sbmac16_addsub.B_REG = 1'b0;
   defparam i_sbmac16_addsub.A_REG = 1'b0;
   defparam i_sbmac16_addsub.C_REG = 1'b0;

   
   /*
    *	This uses Yosys's support for nonzero initial values:
    *
    *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
    *
    *	Rather than using this simulation construct (`initial`),
    *	the design should instead use a reset signal going to
    *	modules in the design.
    */
   initial begin
      ALUOut = 32'b0;
      Branch_Enable = 1'b0;
   end

   always @(ALUctl, A, B, add_O) begin
      // i_sbmac16_addsub: add_C+add_A -> top, add_B+add_D -> bottom
      add_A <= A [31:16];
      add_C <= B [31:16];
      add_D <= A [15:0];
      add_B <= B [15:0];
      add_addsubtop <= 0;
      add_addsubbot <= 0;

      
      case (ALUctl[3:0])
	/*
	 *	AND (the fields also match ANDI and LUI)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_AND:	ALUOut = A & B;

	/*
	 *	OR (the fields also match ORI)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_OR:	ALUOut = A | B;

	/*
	 *	ADD (the fields also match AUIPC, all loads (inc. LW), all stores, and ADDI)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_ADD:       begin
	                                                    add_addsubtop <= 0;
	                                                    add_addsubbot <= 0;
	                                                    add_A <= A [31:16];
                                                            add_C <= B [31:16];
                                                            add_D <= A [15:0];
	                                                    add_B <= B [15:0];
	     
	                                                    ALUOut = add_O;
	                                                end
	/*
	 *	SUBTRACT (the fields also matches all branches)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB:       begin
	                                                    add_addsubtop <= 1;
	                                                    add_addsubbot <= 1;
	                                                    add_A <= A [31:16];
                                                            add_C <= B [31:16];
                                                            add_D <= A [15:0];
	                                                    add_B <= B [15:0];
	     
	                                                    ALUOut = add_O;
	                                                end

	/*
	 *	SLT (the fields also matches all the other SLT variants)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLT:	ALUOut = $signed(A) < $signed(B) ? 32'b1 : 32'b0;

	/*
	 *	SRL (the fields also matches the other SRL variants)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRL:	ALUOut = A >> B[4:0];

	/*
	 *	SRA (the fields also matches the other SRA variants)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRA:	ALUOut = $signed(A) >>> B[4:0];

	/*
	 *	SLL (the fields also match the other SLL variants)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLL:	ALUOut = A << B[4:0];

	/*
	 *	XOR (the fields also match other XOR variants)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_XOR:	ALUOut = A ^ B;

	/*
	 *	CSRRW  only
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRW:	ALUOut = A;

	/*
	 *	CSRRS only
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRS:	ALUOut = A | B;

	/*
	 *	CSRRC only
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRC:	ALUOut = (~A) & B;

	/*
	 *	Should never happen.
	 */
	default:					ALUOut = 0;
      endcase
   end

   always @(ALUctl, ALUOut, A, B) begin
      case (ALUctl[6:4])
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:	Branch_Enable = (ALUOut == 0);
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE:	Branch_Enable = !(ALUOut == 0);
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT:	Branch_Enable = ($signed(A) < $signed(B));
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:	Branch_Enable = ($signed(A) >= $signed(B));
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:	Branch_Enable = ($unsigned(A) < $unsigned(B));
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:	Branch_Enable = ($unsigned(A) >= $unsigned(B));

	default:					Branch_Enable = 1'b0;
      endcase
   end
endmodule
