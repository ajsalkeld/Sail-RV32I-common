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
`include "sb_mac16_only.v" //simulation testing purposes

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
   //reg			sub_addsubbot = 1;
   //reg			sub_addsubbot = 1; 

   reg [15:0]		add_C2;
   reg [15:0]		add_A2;
   reg [15:0]		add_B2;
   reg [15:0]		add_D2;
   wire [31:0]		add_O2;

   //wire [31:0]		sub_O2; 
   
   reg			xor_addsubtop = 0; 
   reg			xor_addsubbot = 0;
 

   // Default values for simulation purposes
   reg			CE = 1;
   reg			DEF = 0;


   reg [15:0]		mult1_A;
   reg [15:0]		mult1_B;
   reg [15:0]		mult_C = 16'b0;
   reg [15:0]		mult_D = 16'b0;
   wire [31:0]		mult1_O;
   reg [15:0]		mult2_A;
   reg [15:0]		mult2_B;
   wire [31:0]		mult2_O;
   reg [15:0]		mult3_A;
   reg [15:0]		mult3_B;
   wire [31:0]		mult3_O;

   reg [31:0]		sll_two_power_n;
   //reg [31:0]		bit_rev_A;
   //reg [31:0]		bit_rev_srlout;
   /*
   // for branch enable cases
   wire			add_signextout;
   reg			branch_addsubtop;
   reg			branch_addsubbot;
   reg [15:0]		branch_A;
   reg [15:0]		branch_B;
   reg [15:0]		branch_C;
   reg [15:0]		branch_D;   
   wire [31:0]		br_O;
   //wire			br_co;
   //wire			br_accumco;
*/
  // reg			br_sel; // 1 if ALUOut is 0.

   /*
   // Discarding higher bits of B for SRL, SRA, and SLL operations.
   reg	[4:0]		B_lowerfive;
   reg	[32:0]		srl_out;
   reg	[32:0]		sra_out;
   reg	[32:0]		sll_out;
   */
   
   // Use DSP for addition, subtraction
   SB_MAC16 i_sbmac16_addsub
     (
      .A(add_A),
      .B(add_B),
      .C(add_C),
      .D(add_D),
      .O(add_O),
      .CLK(clk),
      .ADDSUBTOP(add_addsubtop),
      .ADDSUBBOT(add_addsubbot),
      .OLOADTOP(DEF),
      .OLOADBOT(DEF)/*,
      .SIGNEXTOUT(add_signextout)*/
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

   // DSP for branch enable block
  /*
   SB_MAC16 branch_sbmac16_inst
   (
	   .A(branch_A),
	   .B(branch_B),
	   .C(branch_C),
	   .D(branch_D),
	   .O(br_O),
	   .CLK(clk),
	   .ADDSUBTOP(branch_addsubtop),
	   .ADDSUBBOT(branch_addsubbot),
	   .SIGNEXTOUT(add_signextout),
	   //.CE(CE),
	   //.IRSTTOP(DEF),
	   //.IRSTBOT(DEF),
	   //.ORSTTOP(DEF),
	   //.ORSTBOT(DEF),
	   //.AHOLD(DEF),
	   //.BHOLD(DEF),
	   //.CHOLD(DEF),
	   //.DHOLD(DEF),
	   .OHOLDTOP(DEF),
	   .OHOLDBOT(DEF),
	   .OLOADTOP(DEF),
	   .OLOADBOT(DEF)
	   //.CI(DEF),
	   //.ACCUMCI(DEF),
	   //.ACCUMCO(br_accumco),
	   //.CO(br_co)
	   //.SIGNEXTIN(DEF)
   );

   defparam branch_sbmac16_inst.B_SIGNED = 1'b0;
   defparam branch_sbmac16_inst.A_SIGNED = 1'b0;
   defparam branch_sbmac16_inst.MODE_8x8 = 1'b1;
   defparam branch_sbmac16_inst.BOTADDSUB_CARRYSELECT = 2'b00;
   defparam branch_sbmac16_inst.BOTADDSUB_UPPERINPUT = 1'b1;
   defparam branch_sbmac16_inst.BOTADDSUB_LOWERINPUT = 2'b00;
   defparam branch_sbmac16_inst.BOTOUTPUT_SELECT = 2'b00;

   defparam branch_sbmac16_inst.TOPADDSUB_CARRYSELECT = 2'b10;
   defparam branch_sbmac16_inst.TOPADDSUB_UPPERINPUT = 1'b1;
   defparam branch_sbmac16_inst.TOPADDSUB_LOWERINPUT = 2'b00;
   defparam branch_sbmac16_inst.TOPOUTPUT_SELECT = 2'b00;

   defparam branch_sbmac16_inst.PIPELINE_16x16_MULT_REG2 = 1'b0;
   defparam branch_sbmac16_inst.PIPELINE_16x16_MULT_REG1 = 1'b0;
   defparam branch_sbmac16_inst.BOT_8x8_MULT_REG = 1'b0;
   defparam branch_sbmac16_inst.TOP_8x8_MULT_REG = 1'b0;
   defparam branch_sbmac16_inst.D_REG = 1'b0;
   defparam branch_sbmac16_inst.B_REG = 1'b0;
   defparam branch_sbmac16_inst.A_REG = 1'b0;
   defparam branch_sbmac16_inst.C_REG = 1'b0;  
*/

// 16x16 multiplier, for SLL - low bits
SB_MAC16 sb_mac16_mult1
(
   .A(mult1_A),
   .B(mult1_B),
   .C(mult_C),
   .D(mult_D),
   .O(mult1_O),
   .CLK(clk),
   .CE(DEF),
   .IRSTTOP(DEF),
   .IRSTBOT(DEF),
   .ORSTTOP(DEF),
   .ORSTBOT(DEF),
   .AHOLD(DEF),
   .BHOLD(DEF),
   .CHOLD(DEF),
   .DHOLD(DEF),
   .OHOLDTOP(DEF),
   .OHOLDBOT(DEF),
   .OLOADTOP(DEF),
   .OLOADBOT(DEF),
   .ADDSUBTOP(DEF),
   .ADDSUBBOT(DEF),
   .CI(DEF),
   .ACCUMCI(DEF),
   .ACCUMCO(),
   .SIGNEXTIN(DEF),
   .SIGNEXTOUT()
);

defparam sb_mac16_mult1.PIPELINE_16x16_MULT_REG1 = 0;
defparam sb_mac16_mult1.PIPELINE_16x16_MULT_REG2 = 0;
defparam sb_mac16_mult1.TOPOUTPUT_SELECT = 2'b11;
defparam sb_mac16_mult1.BOTOUTPUT_SELECT = 2'b11;


// SLL 2
SB_MAC16 sb_mac16_mult2
 (
 .A(mult2_A),
 .B(mult2_B),
 .C(mult_C),
 .D(mult_D),
 .O(mult2_O),
 .CLK(clk),
 .CE(DEF),
 .IRSTTOP(DEF),
 .IRSTBOT(DEF),
 .ORSTTOP(DEF),
 .ORSTBOT(DEF),
 .AHOLD(DEF),
 .BHOLD(DEF),
 .CHOLD(DEF),
 .DHOLD(DEF),
 .OHOLDTOP(DEF),
 .OHOLDBOT(DEF),
 .OLOADTOP(DEF),
 .OLOADBOT(DEF),
 .ADDSUBTOP(DEF),
 .ADDSUBBOT(DEF),
 .CI(DEF),
 .ACCUMCI(DEF),
 .ACCUMCO(),
 .SIGNEXTIN(DEF),
 .SIGNEXTOUT()
);

defparam sb_mac16_mult2.PIPELINE_16x16_MULT_REG1 = 0;
defparam sb_mac16_mult2.PIPELINE_16x16_MULT_REG2 = 0;
defparam sb_mac16_mult2.TOPOUTPUT_SELECT = 2'b11;
defparam sb_mac16_mult2.BOTOUTPUT_SELECT = 2'b11;


SB_MAC16 sb_mac16_mult3
(
.A(mult3_A),
.B(mult3_B),
.C(mult_C),
.D(mult_D),
.O(mult3_O),
.CLK(clk),
.CE(DEF),
.IRSTTOP(DEF),
.IRSTBOT(DEF),
.ORSTTOP(DEF),
.ORSTBOT(DEF),
.AHOLD(DEF),
.BHOLD(DEF),
.CHOLD(DEF),
.DHOLD(DEF),
.OHOLDTOP(DEF),
.OHOLDBOT(DEF),
.OLOADTOP(DEF),
.OLOADBOT(DEF),
.ADDSUBTOP(DEF),
.ADDSUBBOT(DEF),
.CI(DEF),
.ACCUMCI(DEF),
.ACCUMCO(),
.SIGNEXTIN(DEF),
.SIGNEXTOUT()
);

defparam sb_mac16_mult3.PIPELINE_16x16_MULT_REG1 = 0;
defparam sb_mac16_mult3.PIPELINE_16x16_MULT_REG2 = 0;
defparam sb_mac16_mult3.TOPOUTPUT_SELECT = 2'b11;
defparam sb_mac16_mult3.BOTOUTPUT_SELECT = 2'b11;


// Two 16-bit adders (32-bit adder at the moment - topaddsub_carryselect
SB_MAC16 xor_sb_mac16_inst
(
   .A(add_A2),
   .B(add_B2),
   .C(add_C2),
   .D(add_D2),
   .O(add_O2),
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
   .ADDSUBTOP(xor_addsubtop),
   .ADDSUBBOT(xor_addsubbot),
   .OHOLDTOP(DEF),
   .OHOLDBOT(DEF),
   .OLOADTOP(DEF),
   .OLOADBOT(DEF),
   .CI(DEF),
   .ACCUMCI(DEF),
   .SIGNEXTIN(DEF)
);

defparam xor_sb_mac16_inst.B_SIGNED = 1'b0;
defparam xor_sb_mac16_inst.A_SIGNED = 1'b0;
defparam xor_sb_mac16_inst.MODE_8x8 = 1'b1;

defparam xor_sb_mac16_inst.BOTADDSUB_CARRYSELECT = 2'b00;
defparam xor_sb_mac16_inst.BOTADDSUB_UPPERINPUT = 1'b1;
defparam xor_sb_mac16_inst.BOTADDSUB_LOWERINPUT = 2'b00;
defparam xor_sb_mac16_inst.BOTOUTPUT_SELECT = 2'b00;

defparam xor_sb_mac16_inst.TOPADDSUB_CARRYSELECT = 2'b00;
defparam xor_sb_mac16_inst.TOPADDSUB_UPPERINPUT = 1'b1;
defparam xor_sb_mac16_inst.TOPADDSUB_LOWERINPUT = 2'b00;
defparam xor_sb_mac16_inst.TOPOUTPUT_SELECT = 2'b00;

defparam xor_sb_mac16_inst.PIPELINE_16x16_MULT_REG2 = 1'b0;
defparam xor_sb_mac16_inst.PIPELINE_16x16_MULT_REG1 = 1'b0;
defparam xor_sb_mac16_inst.BOT_8x8_MULT_REG = 1'b0;
defparam xor_sb_mac16_inst.TOP_8x8_MULT_REG = 1'b0;
defparam xor_sb_mac16_inst.D_REG = 1'b0;
defparam xor_sb_mac16_inst.B_REG = 1'b0;
defparam xor_sb_mac16_inst.A_REG = 1'b0;
defparam xor_sb_mac16_inst.C_REG = 1'b0;

/*
*	This uses Yosys's support for nonzero initial values:
*
*		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
*
*	Rather than using this simulation construct (`initial`),
*	the design should instead use a reset signal going to
*	modules in the design.
*/

integer i;
//integer N; //for shift right operations
initial begin
ALUOut = 32'b0;
Branch_Enable = 1'b0;
end

/*
always @(A, B_lowerfive) begin
srl_out = A >> B_lowerfive;
   	sra_out = A >>> B_lowerfive;
   	sll_out = A << B_lowerfive;
   end
   */

  /*
   always @(ALUctl) begin
	   if (ALUctl[3:0] == `kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB) begin
		   add_addsubtop <= 1;
		   add_addsubbot <= 1;
	   end
	   else begin
		   add_addsubtop <= 0;
		   add_addsubbot <= 0;
	   end
   end
*/

// Removed bit_rev_A, add_O2, and all the multiplier outputs to see the
// difference - none
   always @(clk, ALUctl, A, B, add_O, mult1_O, mult2_O, mult3_O) begin
	   // i_sbmac16_addsub: add_C+add_A -> top, add_B+add_D -> bottom
      add_addsubtop <= 0;
      add_addsubbot <= 0;
	   
      add_C <= B [31:16];
      add_A <= A [31:16];
      add_B <= B [15:0];
      add_D <= A [15:0];


      add_C2 <= add_O2 [15:0];
      add_A2 <= mult1_O [31:16];
      add_B2 <= mult3_O [15:0];
      add_D2 <= mult2_O [15:0];

      // For SLL
      sll_two_power_n = 2 ** {B[4:0]};
      
      mult1_A <= A [15:0];
      mult1_B <= sll_two_power_n [15:0];
      mult2_A <= A [15:0];
      mult2_B <= sll_two_power_n [31:16];
      //mult2_B [15:8] <= 8'b0;
      mult3_A <= A [31:16];      //A [31:16];
      //mult3_A [15:0] <= 8'b0;
      mult3_B <= sll_two_power_n [15:0];
	
      // SRL
      /*for (i=0; i<=31; i=i+1) bit_rev_A [i] <= A [31-i];
	*/
      //B_lowerfive <= B[4:0];
      /*
      add_D2 <= {1'b0, A[23], 1'b0, A[22], 1'b0, A[21], 1'b0, A[20], 1'b0, A[19], 1'b0, A[18], 1'b0, A[17], 1'b0, A[16]};
      add_A2 <= {1'b0, A[31], 1'b0, A[30], 1'b0, A[29], 1'b0, A[28], 1'b0, A[27], 1'b0, A[26], 1'b0, A[25], 1'b0, A[24]};
      add_B2 <= {1'b0, B[23], 1'b0, B[22], 1'b0, B[21], 1'b0, B[20], 1'b0, B[19], 1'b0, B[18], 1'b0, B[17], 1'b0, B[16]};
      add_C2 <= {1'b0, B[31], 1'b0, B[30], 1'b0, B[29], 1'b0, B[28], 1'b0, B[27], 1'b0, B[26], 1'b0, B[25], 1'b0, B[24]};
	*/
    
      xor_addsubtop <= 0;
      xor_addsubbot <= 0;
 

      /* Introduces a delay
      xor_add_D <= {1'b0, A[7], 1'b0, A[6], 1'b0, A[5], 1'b0, A[4], 1'b0, A[3], 1'b0, A[2], 1'b0, A[1], 1'b0, A[0]};
      xor_add_A <= {1'b0, A[15], 1'b0, A[14], 1'b0, A[13], 1'b0, A[12], 1'b0, A[11], 1'b0, A[10], 1'b0, A[9], 1'b0, A[8]};
      xor_add_B <= {1'b0, B[7], 1'b0, B[6], 1'b0, B[5], 1'b0, B[4], 1'b0, B[3], 1'b0, B[2], 1'b0, B[1], 1'b0, B[0]};
      xor_add_C <= {1'b0, B[15], 1'b0, B[14], 1'b0, B[13], 1'b0, B[12], 1'b0, B[11], 1'b0, B[10], 1'b0, B[9], 1'b0, B[8]};
	*/

      /*
      // SLL multiplier inputs
      mult1_A <= A[15:0];
      mult1_B [4:0] <= B[4:0]; // testing purposes: should actually be two_power_n [15:0] where n = B[4:0]
	*/
      
       /*case (ALUctl[2:0])
	       3'b000:
		       case (ALUctl[3])
			       1'b0:	ALUOut = A & B;		// AND
			       1'b1:	ALUOut = A >>> B;	// OR
		       endcase
	       3'b001:
		       case (ALUctl[3])
			       1'b0:	ALUOut = A | B;		// OR / CSRRS
			       1'b1:	ALUOut = A << B;
		       endcase
	       3'b010:	begin
		       ALUOut = add_O;
		       case (ALUctl[3])
			       1'b0:	begin
				       add_addsubtop <= 0;	// ADD
				       add_addsubbot <= 0;
			       end
			       1'b1:	begin
				       add_addsubtop <= 1;	// SUB
				       add_addsubbot <= 1;
			       end
		       endcase
	       end
	       3'b011:
		       case (ALUctl[3])
			       1'b0:	ALUOut = A >> B;	// SRL
			       1'b1:	ALUOut = $signed(A) < $signed(B) ? 32'b1 : 32'b0; // SLT
		       endcase
	       3'b100:			ALUOut = A ^ B;		// XOR
	       3'b101:			ALUOut = A;		// CSRRW
	       3'b110:			ALUOut = (~A) & B;	// CSRRC
	       default:			ALUOut = 0;		// not assigned to anything
       endcase
	*/

      case (ALUctl[3:0])
	/*
	 *	AND (the fields also match ANDI and LUI)
	 */
	
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_AND:	
	begin
		for (i=0; i<=31; i=i+1) ALUOut[i] = A[i] ? B[i] : 1'b0;
	end

	/*begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= 16'b0;
		add_B <= 16'b0;
		add_A <= A [31:16];	// & B [31:16];
		add_D <= A [15:0];	// & B [15:0];
		ALUOut <= add_O & B;
	end*/
		
	/*begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= 16'b0;
		add_B <= 16'b0;
		add_A <= A [31:16] & B [31:16];
		add_D <= A [15:0] & B [15:0];
		ALUOut = add_O;	//A & B;
	end*/
	/*begin
			add_addsubtop <= 0;
			add_addsubbot <= 0;
			add_D <= {1'b0, A[7], 1'b0, A[6], 1'b0, A[5], 1'b0, A[4], 1'b0, A[3], 1'b0, A[2], 1'b0, A[1], 1'b0,
				B
			       		A[0]};
			add_A <= {1'b0, A[15], 1'b0, A[14], 1'b0, A[13], 1'b0, A[12], 1'b0, A[11], 1'b0, A[10], 1'b0, A[9], 
					1'b0, A[8]};
			add_B <= {1'b0, B[7], 1'b0, B[6], 1'b0, B[5], 1'b0, B[4], 1'b0, B[3], 1'b0, B[2], 1'b0, B[1], 1'b0,
			       		B[0]};
			add_C <= {1'b0, B[15], 1'b0, B[14], 1'b0, B[13], 1'b0, B[12], 1'b0, B[11], 1'b0, B[10], 1'b0, B[9], 
					1'b0, B[8]};
			ALUOut [15:0] <= {add_O[31], add_O[29], add_O[27], add_O[25], add_O[23], add_O[21], add_O[19], 
					add_O[17], add_O[15], add_O[13], add_O[11], add_O[9], add_O[7], add_O[5], 
					add_O[3], add_O[1]};
			ALUOut [31:16] <= {add_O2[31], add_O2[29], add_O2[27], add_O2[25], add_O2[23], add_O2[21], add_O2[19], 
					add_O2[17], add_O2[15], add_O2[13], add_O2[11], add_O2[9], add_O2[7], add_O2[5], 
					add_O2[3], add_O2[1]};
		end*/
				
	/*
	 *	OR (the fields also match ORI)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_OR:	
		for (i=0; i<=31; i=i+1) ALUOut[i] = A[i] ? 1'b1 : B[i];
	/*begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= 32'b0;
		add_B <= 32'b0;
		add_A <= A [31:16];
		add_D <= A [15:0];
		for (i=0; i<=31; i=i+1) ALUOut[i] = add_O ? 1'b1 : B[i];
	end*/
		//ALUOut = A | B;

	/*
	 *	ADD (the fields also match AUIPC, all loads (inc. LW), all stores, and ADDI)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_ADD:	begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= B [31:16];
		add_B <= B [15:0];
		add_A <= A [31:16];
		add_D <= A [15:0];
		ALUOut = add_O;
	end

	/*
	 *	SUBTRACT (the fields also matches all branches)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB:	begin
		add_addsubtop <= 1;
		add_addsubbot <= 1;
		add_C <= B [31:16];
		add_B <= B [15:0];
		add_A <= A [31:16];
		add_D <= A [15:0];
		ALUOut = add_O;
	end

	/*
 	*	SLT (the fields also matches all the other SLT variants)
 	*/
       `kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLT:	/*ALUOut = $signed(A) < $signed(B) ? 32'b1 : 32'b0;*/
       begin
		add_addsubtop <= 1;
 		add_addsubbot <= 1;
		add_C <= B [31:16];
		add_B <= B [15:0];
		add_A <= A [31:16];
		add_D <= A [15:0];
		ALUOut = add_O[31] & 1'b1;
		//ALUOut = add_O[31] ? 32'b1 : 32'b0;
		/*xor_addsubbot <= 0;
		xor_addsubtop <= 0;
		add_C <= 16'b0;
		add_A <= 16'b0;
		add_B <= 16'b0;
		add_D <= add_O[31];
		ALUOut <= add_O2;*/
	end		

		//ALUOut = $signed(A) < $signed(B) ? 32'b1 : 32'b0;

	/*
 	*	SRL (the fields also matches the other SRL variants)
 	*/
       `kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRL:	ALUOut = A >> B [4:0];
	/*begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= 16'b0;
		add_B <= 16'b0;
		add_A <= A [31:16+B[4:0]];
		add_D <= A [16+B[4:0]:B[4:0]];
		ALUOut <= add_O;
	end*/

		//ALUOut = A >> B [4:0];	
	
	/*begin
		for (i=0; i<=31; i=i+1) bit_rev_A [i] <= A [31-i];
		
		mult1_A <= bit_rev_A [15:0];
		mult2_A <= bit_rev_A [15:0];
		mult3_A <= bit_rev_A [31:16];

		bit_rev_srlout [31:16] <= add_O2 [31:16];
		bit_rev_srlout [15:0] <= mult1_O [15:0];

		add_C <= 16'b0;
		add_B <= 16'b0;
		for (i=0; i<=15; i=i+1) 
			begin
				add_D [i] <= bit_rev_srlout [31-i];
				add_A [i] <= bit_rev_srlout [15-i];
			end
		//for (i=0; i<=31; i=i+1) ALUOut [i] <= bit_rev_srlout [31-i];
		ALUOut <= add_O;
	end*/
	
	/*
	 *	SRA (the fields also matches the other SRA variants)
 	*/
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRA:	ALUOut = $signed(A) >>> B [4:0];
	/*begin
	mult1_A <= bit_rev_A [15:0];
	mult2_A <= bit_rev_A [15:0];
	mult3_A <= bit_rev_A [31:16];

	bit_rev_srlout [31:16] <= add_O2 [31:16];
	bit_rev_srlout [15:0] <= mult1_O [15:0];
	
	for (i=0; i<=31; i=i+1) ALUOut [i] <= bit_rev_srlout [31-i];

	N = B [4:0];
	for (i=0; i<=N-1; i=i+1) ALUOut [31-i] <= A[31];
	//ALUOut [31:31-N] <= { N {A[31]}};
	end*/

	/*
	 *	SLL (the fields also match the other SLL variants)
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLL:	begin
		// discarding top 16 bit output! - check if this is correct
		// also check if can use just 1 adder as 2 16-bit adders, save
		// luts - not possible
		mult1_A <= A [15:0];
		mult2_A <= A [15:0];
		mult3_A <= A [31:16];

		add_C <= 16'b0;
		add_B <= 16'b0;
		add_A <= add_O2 [31:16];	// Changing to 31:16 did not help
		add_D <= mult1_O [15:0];
		ALUOut <= add_O;
	end
		
		//ALUOut = A << B[4:0];

	/*
	 *	XOR (the fields also match other XOR variants)
	 */

	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_XOR:	//ALUOut = {A[31:0] ? (~B[31:0]) : B[31:0]};
		for (i=0; i<=31; i=i+1) ALUOut[i] = A[i] ? (~B[i]) : B[i];
	/*begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= 32'b0;
		add_B <= 32'b0;
		add_A <= A [31:16];
		add_D <= A [15:0];
		for (i=0; i<=31; i=i+1) ALUOut[i] = A[i] ? add_O : B[i];	
	end*/
		/*begin
		add_addsubtop <= 1;
		add_addsubbot <= 1;
		// Output = A-C, D-B
		add_C <= A [31:16];
		add_B <= A [15:0];
		add_A <= 16'b1111111111111111;
		add_D <= 16'b1111111111111111;
		for (i=0; i<=31; i=i+1) ALUOut[i] = A[i] ? add_O[i] : B[i];
	end*/
	/*ALUOut = A ^ B;*/	
	/*begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= 16'b0;
		add_B <= 16'b0;
		add_A <= A [31:16] ^ B [31:16];
		add_D <= A [15:0] ^ B [15:0];
		ALUOut <= add_O;
	end*/
	/*begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_D <= {1'b0, A[7], 1'b0, A[6], 1'b0, A[5], 1'b0, A[4], 1'b0, A[3], 1'b0, A[2], 1'b0, A[1], 1'b0, A[0]};
		add_A <= {1'b0, A[15], 1'b0, A[14], 1'b0, A[13], 1'b0, A[12], 1'b0, A[11], 1'b0, A[10], 1'b0, A[9], 1'b0, A[8]};
		add_B <= {1'b0, B[7], 1'b0, B[6], 1'b0, B[5], 1'b0, B[4], 1'b0, B[3], 1'b0, B[2], 1'b0, B[1], 1'b0, B[0]};
		add_C <= {1'b0, B[15], 1'b0, B[14], 1'b0, B[13], 1'b0, B[12], 1'b0, B[11], 1'b0, B[10], 1'b0, B[9], 1'b0, B[8]};
		ALUOut [15:0] <= {add_O[30], add_O[28], add_O[26], add_O[24], add_O[22], add_O[20], add_O[18], 
			add_O[16], add_O[14], add_O[12], add_O[10], add_O[8], add_O[6], add_O[4], add_O[2],  
					add_O[0]};
		// Top half - using second DSP block
		ALUOut [31:16] <= {add_O2[30], add_O2[28], add_O2[26], add_O2[24], add_O2[22], add_O2[20], add_O2[18],
					add_O2[16], add_O2[14], add_O2[12], add_O2[10], add_O2[8], add_O2[6], add_O2[4], 
					add_O2[2], add_O2[0]};
	end*/
	
        /*
	 *	CSRRW  only
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRW:	/*ALUOut = A;*/	begin
		add_addsubtop <= 0;
		add_addsubbot <= 0;
		add_C <= 16'b0;
		add_B <= 16'b0;
		add_A <= A [31:16];
		add_D <= A [15:0];
		ALUOut <= add_O;
	end
		//ALUOut = A;

	/*
	 *	CSRRS only - keep this commented out - reduces luts by ~29
	 */
	//`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRS:	ALUOut = A | B;

	/*
	 *	CSRRC only
	 */
	`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRC:	/*ALUOut = (~A) & B;*/	/*begin
		add_addsubtop <= 1;
		add_addsubbot <= 1;
		add_C <= A [31:16];
		add_B <= A [15:0];
		add_A <= A [31:16] & B [31:16];
		add_D <= A [15:0] & B [15:0];
		ALUOut <= add_O;
	end*/
	/*	begin
		ALUOut <= B;
		for (i=0; i<=31; i=i+1) begin
			if (A[i]) ALUOut[i] <= 1'b0;
		end
	end*/
		begin
		add_addsubtop <= 1;
		add_addsubbot <= 1;
		add_C <= A [31:16];
		add_B <= A [15:0];
		add_A <= 16'b1111111111111111;
		add_D <= 16'b1111111111111111;
		ALUOut = add_O & B;
	end
		//ALUOut = (~A) & B;

	/*
	 *	Should never happen.
	 */
	default:					ALUOut = 0;
      endcase
   end

   /*always @(ALUOut) begin
	   br_sel = (ALUOut == 0);
   end*/

   always @(ALUctl, ALUOut, A, B/*, br_O*/) begin
/*	   branch_addsubtop <= 1;
           branch_addsubbot <= 1;
	   branch_C <= B [31:16];
	   branch_B <= B [15:0];
	   branch_A <= A [31:16];
	   branch_D <= A [15:0];
*/
	   case (ALUctl[6:4])
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:	Branch_Enable =	(ALUOut == 0);
	/*begin
		if (A[31] == B[31])	Branch_Enable = br_O[31] ^ 1'b1;
		else			Branch_Enable = A[31];
	end*/
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE:	Branch_Enable = !(ALUOut == 0);
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT:	//Branch_Enable = add_signextout;
		//Branch_Enable <= br_accumco;
		//Branch_Enable <= br_O[31] & 1'b1;
		Branch_Enable = ($signed(A) < $signed(B));
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:	//Branch_Enable = add_signextout ? 1'b0 : 1'b1;
		//Branch_Enable <= br_co;
		//Branch_Enable <= br_O[31] ^ 1'b1;
		Branch_Enable = ($signed(A) >= $signed(B));
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:	//Branch_Enable <= add_signextout;
		Branch_Enable = ($unsigned(A) < $unsigned(B));
	`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:	//Branch_Enable <= add_signextout ? 1'b0 : 1'b1;
		Branch_Enable = ($unsigned(A) >= $unsigned(B));
	default:					Branch_Enable = 1'b0;
      endcase
   end
endmodule
