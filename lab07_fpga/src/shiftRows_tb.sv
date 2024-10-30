module shiftRows_tb();
	logic clk;
	logic [127:0] in, out, outExpected;

	shiftRows dut( .in(in), .out(out));
	always
		begin
		clk = 0; #5;
		clk = 1; #5;

		end
	initial
		begin
		in = 128'hd42711aee0bf98f1b8b45de51e415230; #10;
		outExpected = 128'hd4bf5d30e0b452aeb84111f11e2798e5;
		end

endmodule