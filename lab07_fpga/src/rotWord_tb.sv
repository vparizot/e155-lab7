// Victoria Parizot
// vparizot@hmc.edu
// 10/31/2024
// Lab 7 Test Bench for rotWord, helper function within keyExpansionROund

module rotWord_tb();
	logic [31:0] word, rotWordOut, rotWordOutExpected;
	logic clk;
	
	rotWord dut( .word(word), .rotWordOut(rotWordOut));
	always
		begin
		clk = 1; #1;
		clk = 0; #0;
		end
	initial
		begin
		word = 32'h09cf4f3c; #10;
		rotWordOutExpected = 32'hcf4f3c09;
		end

endmodule