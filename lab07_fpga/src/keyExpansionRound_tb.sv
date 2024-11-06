// Victoria Parizot
// vparizot@hmc.edu
// 10/31/2024
// Lab 7 Test Bench for a Key Expansion operation


module keyExpansionRound_tb();
	logic [3:0] round;
	logic [127:0] key, rkey, rkeyexpected;
	logic [31:0] Rcon;
	logic clk;

	keyExpansionRound dut( .clk(clk), .key(key), .Rcon(Rcon),  .rkey(rkey));
	always
		begin
		clk = 0; #5;
		clk = 1; #5;

		end
	initial
		begin
		//key = 128'h2b7e151628aed2a6abf7158809cf4f3c;
		//round = 3'd1;
		//rkeyexpected = 128'ha0fafe1788542cb123a339392a6c7605;
		//#500;
		
		key = 128'ha0fafe1788542cb123a339392a6c7605;
		Rcon = 32'h01000000;
		rkeyexpected = 128'hf2c295f27a96b9435935807a7359f67f;
		#500;


		key = 128'hf2c295f27a96b9435935807a7359f67f;
		round = 3'd3;
		rkeyexpected = 128'h3d80477d4716fe3e1e237e446d7a883b; /////////////////
		#100;


		end

endmodule