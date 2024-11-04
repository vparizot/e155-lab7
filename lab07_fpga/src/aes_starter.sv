/////////////////////////////////////////////
// aes
//   Top level module with SPI interface and SPI core
/////////////////////////////////////////////

module aes(//input  logic clk,
           input  logic sck, 
           input  logic sdi,
           output logic sdo,
           input  logic load,
           output logic done);
             
    logic [127:0] key, plaintext, cyphertext;
    //logic clksig;
    HSOSC hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk));
            
    aes_spi spi(sck, sdi, sdo, done, key, plaintext, cyphertext);   
    aes_core core(clk, load, key, plaintext, done, cyphertext);

endmodule

/////////////////////////////////////////////
// aes_spi
//   SPI interface.  Shifts in key and plaintext
//   Captures ciphertext when done, then shifts it out
//   Tricky cases to properly change sdo on negedge clk
/////////////////////////////////////////////


module aes_spi(input  logic sck, 
               input  logic sdi,
               output logic sdo,
               input  logic done,
               output logic [127:0] key, plaintext,
               input  logic [127:0] cyphertext);

    logic         sdodelayed, wasdone;
    logic [127:0] cyphertextcaptured;
               
    // assert load
    // apply 256 sclks to shift in key and plaintext, starting with plaintext[127]
    // then deassert load, wait until done
    // then apply 128 sclks to shift out cyphertext, starting with cyphertext[127]
    // SPI mode is equivalent to cpol = 0, cpha = 0 since data is sampled on first edge and the first
    // edge is a rising edge (clock going from low in the idle state to high).
    always_ff @(posedge sck)
        if (!wasdone)  {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[126:0], key, sdi};
        else           {cyphertextcaptured, plaintext, key} = {cyphertextcaptured[126:0], plaintext, key, sdi}; 
    
    // sdo should change on the negative edge of sck
    always_ff @(negedge sck) begin
        wasdone = done;
        sdodelayed = cyphertextcaptured[126];
    end
    
    // when done is first asserted, shift out msb before clock edge
    assign sdo = (done & !wasdone) ? cyphertext[127] : sdodelayed;
endmodule

/////////////////////////////////////////////
// aes_core
//   top level AES encryption module
//   when load is asserted, takes the current key and plaintext
//   generates cyphertext and asserts done when complete 11 cycles later
// 
//   See FIPS-197 with Nk = 4, Nb = 4, Nr = 10
//
//   The key and message are 128-bit values packed into an array of 16 bytes as
//   shown below
//        [127:120] [95:88] [63:56] [31:24]     S0,0    S0,1    S0,2    S0,3
//        [119:112] [87:80] [55:48] [23:16]     S1,0    S1,1    S1,2    S1,3
//        [111:104] [79:72] [47:40] [15:8]      S2,0    S2,1    S2,2    S2,3
//        [103:96]  [71:64] [39:32] [7:0]       S3,0    S3,1    S3,2    S3,3
//
//   Equivalently, the values are packed into four words as given
//        [127:96]  [95:64] [63:32] [31:0]      w[0]    w[1]    w[2]    w[3]
/////////////////////////////////////////////

module aes_core(input  logic         clk, 
                input  logic         load,
                input  logic [127:0] key, 
                input  logic [127:0] plaintext, 
                output logic         done, 
                output logic [127:0] cyphertext);

    // Define internal variables
    logic [3:0] round;
    logic [10:0] counter;
    logic [31:0] Rcon;
    logic [127:0] textIn0, keyIn0, textOut0, in, out1, out2, inMC, outMC, currKey, keyOut, out3, out ;

    // First round
	addRoundKey ark1(textIn0, keyIn0, textOut0);

    // Rounds 1-9
	subBytes a1(clk, in, out1);
   	shiftRows a2(out1, out2); 
   	mixcolumns a3(inMC, outMC); // mixcolumns a3(clk, out2, out3);
   	keyExpansionRound a4(clk, currKey, Rcon, keyOut);
   	addRoundKey ark2(out3, keyOut, out);
    

    always_ff @(posedge clk) begin
	if (load) begin
        // load all new signals, and reset 
		counter <= 0; 
		textIn0 <= plaintext;
		keyIn0 <= key;
		done <= 0;
		round <= 0;
	end
	else begin 
		counter <= counter + 1; // counter used to allow for clock delays
		if (counter == 6) begin
        // after 6 posedges, the signals have stablized, so increment round
		counter <= 0; // reset round counter
		round <= round + 1; // increment round
		in <= out; // update inputs for new round based on last round
		currKey <= keyOut; // update key for new round
		end
	end
	if (counter == 0 && round == 0) 
        // First Round: start round w/ addRoundKey
		begin
		currKey <= keyIn0;
		in <= textOut0;
		end

	if (round < 9) 
        // rounds 1-9, pass through mix columns
		begin
		inMC <= out2;
		out3 <= outMC;
		end 

	else 
        //final round, skip mixcolumns 
		begin
		out3 <= out2;
		
		end 
	if (counter == 6 && round == 9) 
        // stop the process: set done & output cyphertext
		begin
		cyphertext <= out;
		done <= 1;
		end
    
    // define Rcon based on round
	if (round == 0) Rcon = 32'h01000000;
	if (round == 1) Rcon = 32'h02000000;
	if (round == 2) Rcon = 32'h04000000;
	if (round == 3) Rcon = 32'h08000000;
	if (round == 4) Rcon = 32'h10000000;
	if (round == 5) Rcon = 32'h20000000;
	if (round == 6) Rcon = 32'h40000000;
	if (round == 7) Rcon = 32'h80000000;
	if (round == 8) Rcon = 32'h1b000000;
	if (round == 9) Rcon = 32'h36000000;
	end
    
endmodule

module round1to9( input logic clk, 
		  input logic [31:0] Rcon,
		  input logic [3:0] round,
		  input logic [127:0] currKey, in,
		  output logic [127:0] outOfRound, rKeyOut);
  	
	logic [127:0] out, out1, out2, out3, keyOut, keyOutTemp;

	subBytes a1(clk, in, out1);
	shiftRows a2(clk, out1, out2); // TODO: ADD CLK
	mixcolumns a3(out2, out3); //TODO:add clk
	keyExpansionRound a4(clk, currKey, round, Rcon, keyOut);
	
	assign keyOutTemp = keyOut;
	addRoundKey a5(out3, keyOutTemp, out);

  assign outOfRound = out;
  assign rKeyOut = keyOut;
endmodule

module round10( input logic clk, 
			input logic [31:0] Rcon,
		  input logic [3:0] round,
		  input logic [127:0] currKey, in, 
		  output logic [127:0] outFinal);
		  
	logic [127:0] out1, out2, rkey,  outOfRound;

	subBytes a6(clk, in, out1);
	shiftRows a7(clk, out1, out2); // TODO: ADD CLK
	keyExpansionRound a8(clk, currKey, round, Rcon, rkey);
	addRoundKey a9(out2, rkey, outOfRound);
	assign outFinal = outOfRound;
	//assign done = 1'b1;

endmodule


/////////////////////////////////////////////
// sbox
//   Infamous AES byte substitutions with magic numbers
//   Synchronous version which is mapped to embedded block RAMs (EBR)
//   Section 5.1.1, Figure 7
/////////////////////////////////////////////
module sbox_sync(
	input		logic [7:0] a,
	input	 	logic 			clk,
	output 	logic [7:0] y);
            
  // sbox implemented as a ROM
  // This module is synchronous and will be inferred using BRAMs (Block RAMs)
  logic [7:0] sbox [0:255];

  initial   $readmemh("sbox.txt", sbox);
	
	// Synchronous version
	always_ff @(posedge clk) begin
		y <= sbox[a];
	end
endmodule


/////////////////////////////////////////////
// mixcolumns
//   Even funkier action on columns
//   Section 5.1.3, Figure 9
//   Same operation performed on each of four columns
/////////////////////////////////////////////

module mixcolumns(
		  input  logic [127:0] a,
                  output logic [127:0] y);

  mixcolumn mc0(clk, a[127:96], y[127:96]);
  mixcolumn mc1(clk, a[95:64],  y[95:64]);
  mixcolumn mc2(clk, a[63:32],  y[63:32]);
  mixcolumn mc3(clk, a[31:0],   y[31:0]);
endmodule

/////////////////////////////////////////////
// mixcolumn
//   Perform Galois field operations on bytes in a column
//   See EQ(4) from E. Ahmed et al, Lightweight Mix Columns Implementation for AES, AIC09
//   for this hardware implementation
/////////////////////////////////////////////

module mixcolumn(input clk, // added clk signal;
                 input  logic [31:0] a,
                 output logic [31:0] y);
                      
        logic [7:0] a0, a1, a2, a3, y0, y1, y2, y3, t0, t1, t2, t3, tmp;
        
        assign {a0, a1, a2, a3} = a;
        assign tmp = a0 ^ a1 ^ a2 ^ a3;
    
        galoismult gm0(a0^a1, t0);
        galoismult gm1(a1^a2, t1);
        galoismult gm2(a2^a3, t2);
        galoismult gm3(a3^a0, t3);
        
        assign y0 = a0 ^ tmp ^ t0;
        assign y1 = a1 ^ tmp ^ t1;
        assign y2 = a2 ^ tmp ^ t2;
        assign y3 = a3 ^ tmp ^ t3;
        //assign y = {y0, y1, y2, y3};    

        //NOTE: I added always_ff loop
        //always_ff @(posedge clk)
        //begin
         assign y = {y0, y1, y2, y3};   
        //end 
endmodule

/////////////////////////////////////////////
// galoismult
//   Multiply by x in GF(2^8) is a left shift
//   followed by an XOR if the result overflows
//   Uses irreducible polynomial x^8+x^4+x^3+x+1 = 00011011
/////////////////////////////////////////////

module galoismult(input  logic [7:0] a,
                  output logic [7:0] y);

    logic [7:0] ashift;
    
    assign ashift = {a[6:0], 1'b0};
    assign y = a[7] ? (ashift ^ 8'b00011011) : ashift;
endmodule

/////////////////////////////////////////////
// subBytes
//   Calls sbox_sync for entire 128 bit input in 8 bit segments 
/////////////////////////////////////////////
module subBytes(input logic clk,
		        input  logic [127:0] in,
                output logic [127:0] out);

    logic [127:0] outtemp;

    sbox_sync sboxsync1(in[7:0], clk, outtemp[7:0]);
    sbox_sync sboxsync2(in[15:8], clk, outtemp[15:8]);
    sbox_sync sboxsync3(in[23:16], clk, outtemp[23:16]);
    sbox_sync sboxsync4(in[31:24], clk, outtemp[31:24]);
    sbox_sync sboxsync5(in[39:32], clk, outtemp[39:32]);
    sbox_sync sboxsync6(in[47:40], clk, outtemp[47:40]);
    sbox_sync sboxsync7(in[55:48], clk, outtemp[55:48]);
    sbox_sync sboxsync8(in[63:56], clk, outtemp[63:56]);
    sbox_sync sboxsync9(in[71:64], clk, outtemp[71:64]);
    sbox_sync sboxsync10(in[79:72], clk, outtemp[79:72]);
    sbox_sync sboxsync11(in[87:80], clk, outtemp[87:80]);
    sbox_sync sboxsync12(in[95:88], clk, outtemp[95:88]);
    sbox_sync sboxsync13(in[103:96], clk, outtemp[103:96]);
    sbox_sync sboxsync14(in[111:104], clk, outtemp[111:104]);
    sbox_sync sboxsync15(in[119:112], clk, outtemp[119:112]);
    sbox_sync sboxsync16(in[127:120], clk, outtemp[127:120]);

    assign out = outtemp;

endmodule





//   The key and message are 128-bit values packed into an array of 16 bytes as
//   shown below
//        [127:120] [95:88] [63:56] [31:24]     S0,0    S0,1    S0,2    S0,3
//        [119:112] [87:80] [55:48] [23:16]     S1,0    S1,1    S1,2    S1,3
//        [111:104] [79:72] [47:40] [15:8]      S2,0    S2,1    S2,2    S2,3
//        [103:96]  [71:64] [39:32] [7:0]       S3,0    S3,1    S3,2    S3,3
//
//   Equivalently, the values are packed into four words as given
//        [127:96]  [95:64] [63:32] [31:0]      w[0]    w[1]    w[2]    w[3]


/////////////////////////////////////////////
// shiftRows
//   Shift row x by x words
/////////////////////////////////////////////
module shiftRows(input  logic [127:0] in,
                 output logic [127:0] out);

      // row 0 unshifted
       assign out[127:120] = in[127:120];
       assign out[95:88] = in[95:88];
       assign out[63:56] = in[63:56];
       assign out[31:24] = in[31:24];

      // row 1 shifted by 1
       assign out[119:112] = in[87:80];
       assign out[87:80] = in[55:48];
       assign out[55:48] = in[23:16];
       assign out[23:16] = in[119:112];

      // row 2 shifted by 2
       assign out[111:104] = in[47:40];
       assign out[79:72] = in[15:8];
       assign out[47:40] = in[111:104];
       assign out[15:8] = in[79:72];

      // row 3 shifted by 3
       assign out[103:96] = in[7:0];
       assign out[71:64] = in[103:96];
       assign out[39:32] = in[71:64];
       assign out[7:0] = in[39:32];
endmodule

/////////////////////////////////////////////
// addRoundKey
//   XOR the input and roundKey
/////////////////////////////////////////////
module addRoundKey(input  logic [127:0] in,
                   input  logic [127:0] roundKey,
                   output logic [127:0] out);
  assign out = in ^ roundKey; 
endmodule

/////////////////////////////////////////////
// keyExpansionRound
//   generate the new key based on previous key and round constant
/////////////////////////////////////////////
module keyExpansionRound(input logic clk,
			            input logic [127:0] key,
                    	//input logic [3:0] round, //i value
			            input logic [31:0] Rcon,
                    	output logic [127:0] rkey);

    logic [31:0] key0, key1, key2, key3; 
    assign key3 = key[31:0];
    assign key2 = key[63:32];
    assign key1 = key[95:64];
    assign key0 = key[127:96];

    // define words of roundkeys
    logic [31:0] swkey0, swkey1, swkey2, swkey3;
    logic [31:0] rotkey0, rotkey1, rotkey2, rotkey3, temp;
    //logic [31:0] Rcon;
    logic [31:0] rkey0, rkey1, rkey2, rkey3;

    // call subword, rot word (need to repeat for EACH Rcon)
    rotWord r1(key3, rotkey0);
    assign temp = rotkey0;
    subWord sw1(clk, temp, swkey0);
    assign rkey0 = key0 ^ swkey0 ^ Rcon;
    assign rkey1 = key1 ^ rkey0;
    assign rkey2 = key2 ^ rkey1;
    assign rkey3 = key3 ^ rkey2;
    assign rkey = {rkey0, rkey1, rkey2, rkey3};

endmodule

/////////////////////////////////////////////
// subWord
//  helper function for keyExpansionRound that reorders and
//  substitutes current work with sbox_sync
/////////////////////////////////////////////

module subWord(input logic clk,
	           input logic [31:0] word,
               output logic [31:0] subWordOut);

    logic [7:0] subWordOut0, subWordOut1, subWordOut2, subWordOut3;

    sbox_sync sboxsyncsw1(word[7:0], clk, subWordOut0);
    sbox_sync sboxsyncsw2(word[15:8], clk, subWordOut1);
    sbox_sync sboxsyncsw3(word[23:16], clk, subWordOut2);
    sbox_sync sboxsyncsw4(word[31:24], clk, subWordOut3);
	
    assign subWordOut = {subWordOut3, subWordOut2, subWordOut1, subWordOut0};
endmodule

/////////////////////////////////////////////
// rotWord
//  helper function for keyExpansionRound that reorders
//  word by 8 bits
/////////////////////////////////////////////
module rotWord(
	            input logic [31:0] word,
               output logic [31:0] rotWordOut);
    //always_ff @(posedge clk) begin
    assign rotWordOut = {word[23:0], word[31:24]}; 
    //end
endmodule

/////////////////////////////////////////////
// hsoscEnable
//  generate clock signal in FPGA
/////////////////////////////////////////////
module hsoscEnable(
	output logic clk);

// variables
	logic pulse;
	logic int_osc;
	logic led_state = 0;
	logic pulsr ;

// Internal high-speed oscillator
	HSOSC hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk));
	
assign clk = pulse;
endmodule
