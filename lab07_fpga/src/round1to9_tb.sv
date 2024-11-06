// Victoria Parizot
// vparizot@hmc.edu
// 10/31/2024
// Lab 7 Test Bench for rounds 1 - 9 AES encryption

module round1to9_tb();
	logic [3:0] round;
	logic [127:0] currKey, in, outOfRound, rKeyOut, rkeyexpected, outOfRoundExpected;
	logic clk;

	round1to9 dut( .clk(clk), .round(round), .currKey(currKey), .in(in), .outOfRound(outOfRound), .rKeyOut(rKeyOut));
	always
		begin
		clk = 0; #5;
		clk = 1; #5;

		end
	initial
		begin
		currKey = 128'h2b7e151628aed2a6abf7158809cf4f3c;
		round = 3'd1;
		//in = 128'h3242f6a8885a308d313198a2e0370734;
		//rkeyexpected = 128'ha0fafe1788542cb123a339392a6c7605;
		in = 128'h193de3bea0f4e22b9ac68d2ae9f84808;
		rkeyexpected = 128'ha0fafe1788542cb123a339392a6c7605;
		outOfRoundExpected = 128'ha49c7ff2689f352b6b5bea43026a5049;
		
		
		#500;
		
		//key = 128'ha0fafe1788542cb123a339392a6c7605;
		//round = 3'd2;
		//rkeyexpected = 128'hf2c295f27a96b9435935807a7359f67f;
		//#500;


		//key = 128'hf2c295f27a96b9435935807a7359f67f;
		//round = 3'd3;
		//rkeyexpected = 128'h3d80477d4716fe3e1e237e446d7a883b; /////////////////
		//#100;


		end

endmodule