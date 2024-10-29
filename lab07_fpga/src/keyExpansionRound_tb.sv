module keyExpansionRound_tb();
	logic [3:0] j;
	logic [127:0] key;
	logic [31:0] rkey0, rkey1, rkey2, rkey3;

	keyExpansionRound dut( .key(key), .j(j), .rkey0(rkey0), .rkey1(rkey1), .rkey2(rkey2), .rkey3(rkey3));
	always
		begin
		clk = 5; #5;
		clk = 5; #5;

		end
	initial
		begin
		


		end

endmodule