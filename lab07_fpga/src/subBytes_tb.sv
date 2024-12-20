// Victoria Parizot
// vparizot@hmc.edu
// 10/31/2024
// Lab 7 Test Bench for subBytes module for AES


module subBytes_tb();
	logic clk;
	logic [127:0] in, out, outExpected;

	subBytes dut( .in(in), .clk(clk), .out(out));
	always
		begin
		clk = 0; #5;
		clk = 1; #5;

		end
	initial
		begin
		in = 128'h193de3bea0f4e22b9ac68d2ae9f84808; #10;
		outExpected = 128'hd42711aee0bf98f1b8b45de51e415230;
		end

endmodule
