// Victoria Parizot
// vparizot@hmc.edu
// 10/31/2024
// Lab 7 Test Bench for subword, helper function for keyExpansionRounf

module subword_tb();
	logic [31:0] word, subWordOut, subWordOutExpected;
	logic clk;
	
	subWord dut( .word(word), .clk(clk), .subWordOut(subWordOut));
	always
		begin
		clk = 1; #5;
		clk = 0; #5;
		end
	initial
		begin
		word = 32'hcf4f3c09; #10;

		subWordOutExpected = 32'h8a84eb01;
		end
		
endmodule