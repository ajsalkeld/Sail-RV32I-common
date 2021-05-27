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



/*
 *	Description:
 *
 *		This module implements an adder for use by the branch unit
 *		and program counter increment among other things.
 */



module adder(input1, input2, out);
	input [31:0]	input1;
	input [31:0]	input2;
	output [31:0]	out;

	// Used combinational (=) not sequential (<=) assignments - change if
	// this causes issues.
	reg[15:0]	add_C = input1[31:16];
	reg[15:0]	add_A = input2[31:16];
	reg[15:0]	add_B = input1[15:0];
	reg[15:0]	add_D = input2[15:0];

	reg		addsubtop = 0;
	reg		addsubbot = 0;

	SB_MAC16	sb_mac16_inst (
		.A(add_A),
		.B(add_B),
		.C(add_C),
		.D(add_D),
		.O(out),
		//.CLK(clk),	//add back in if needed - unregistered inputs
		//and outputs so should not need clock input to DSP.
		.ADDSUBTOP(addsubtop),
		.ADDSUBBOT(addsubbot)
	);

	defparam sb_mac16_inst.B_SIGNED = 1'b0;
	defparam sb_mac16_inst.A_SIGNED = 1'b0;
	defparam sb_mac16_inst.MODE_8x8 = 1'b1;
	
	defparam sb_mac16_inst.BOTADDSUB_CARRYSELECT = 2'b00;
	defparam sb_mac16_inst.BOTADDSUB_UPPERINPUT = 1'b1;
	defparam sb_mac16_inst.BOTADDSUB_LOWERINPUT = 2'b00;
	defparam sb_mac16_inst.BOTOUTPUT_SELECT = 2'b00;

	defparam sb_mac16_inst.TOPADDSUB_CARRYSELECT = 2'b10;
	defparam sb_mac16_inst.TOPADDSUB_UPPERINPUT = 1'b1;
	defparam sb_mac16_inst.TOPADDSUB_LOWERINPUT = 2'b00;
	defparam sb_mac16_inst.TOPOUTPUT_SELECT = 2'b00;
	
	defparam sb_mac16_inst.PIPELINE_16x16_MULT_REG2 = 1'b0;
	defparam sb_mac16_inst.PIPELINE_16x16_MULT_REG1 = 1'b0;
	defparam sb_mac16_inst.BOT_8x8_MULT_REG = 1'b0;
	defparam sb_mac16_inst.TOP_8x8_MULT_REG = 1'b0;
	defparam sb_mac16_inst.D_REG = 1'b0;
	defparam sb_mac16_inst.B_REG = 1'b0;
	defparam sb_mac16_inst.A_REG = 1'b0;
	defparam sb_mac16_inst.C_REG = 1'b0;

	//assign		out = input1 + input2;
endmodule
