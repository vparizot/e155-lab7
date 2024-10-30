/////////////////////////////////////////////
// aes
//   Top level module with SPI interface and SPI core
/////////////////////////////////////////////

module aes(input  logic clk,
           input  logic sck, 
           input  logic sdi,
           output logic sdo,
           input  logic load,
           output logic done);
                    
    logic [127:0] key, plaintext, cyphertext;
            
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
    // TODO: Your code goes here

    // Define internal variables
    logic [127:0] out1, out2, out3, out4, out5;
    
    // call submodules 
    
    //AddRoundKey(state, key);

    //for round = 1 step 1 to 9 // psuedo code for 9 rounds
    //  subBytes(state);
    //  shiftRows(state);
    //  MixColumns(state);
    //  AddRoundKey(statw, key, keynext);
    //endfor 

    //subBytes (state);
    //ShiftRows (state);
    //AddRoundKey (state, w);

    //out = state 
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

module mixcolumns(input  logic [127:0] a,
                  output logic [127:0] y);

  mixcolumn mc0(a[127:96], y[127:96]);
  mixcolumn mc1(a[95:64],  y[95:64]);
  mixcolumn mc2(a[63:32],  y[63:32]);
  mixcolumn mc3(a[31:0],   y[31:0]);
endmodule

/////////////////////////////////////////////
// mixcolumn
//   Perform Galois field operations on bytes in a column
//   See EQ(4) from E. Ahmed et al, Lightweight Mix Columns Implementation for AES, AIC09
//   for this hardware implementation
/////////////////////////////////////////////

module mixcolumn(input  logic [31:0] a,
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
        assign y = {y0, y1, y2, y3};    
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

// WORKS AS DESIRED IN TB
module subBytes(input  logic [127:0] in,
                input logic clk,
                output logic [127:0] out);
	// logic [7:0] out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out12, out14, out15, out16;
    
// call sbox_sync                       16*x+y
    sbox_sync sboxsync1(in[7:0], clk, out[7:0]);
    sbox_sync sboxsync2(in[15:8], clk, out[15:8]);
    sbox_sync sboxsync3(in[23:16], clk, out[23:16]);
    sbox_sync sboxsync4(in[31:24], clk, out[31:24]);
    sbox_sync sboxsync5(in[39:32], clk, out[39:32]);
    sbox_sync sboxsync6(in[47:40], clk, out[47:40]);
    sbox_sync sboxsync7(in[55:48], clk, out[55:48]);
    sbox_sync sboxsync8(in[63:56], clk, out[63:56]);
    sbox_sync sboxsync9(in[71:64], clk, out[71:64]);
    sbox_sync sboxsync10(in[79:72], clk, out[79:72]);
    sbox_sync sboxsync11(in[87:80], clk, out[87:80]);
    sbox_sync sboxsync12(in[95:88], clk, out[95:88]);
    sbox_sync sboxsync13(in[103:96], clk, out[103:96]);
    sbox_sync sboxsync14(in[111:104], clk, out[111:104]);
    sbox_sync sboxsync15(in[119:112], clk, out[119:112]);
    sbox_sync sboxsync16(in[127:120], clk, out[127:120]);

endmodule


// WORKS AS DESIRED IN TB
module subWord(input logic [31:0] word,
               input logic clk,
               output logic [31:0] subWordOut);

    logic [7:0] subWordOut0, subWordOut1, subWordOut2, subWordOut3;
    //assign {word0, word1, word2, word3} = word;

    sbox_sync sboxsyncsw1(word[7:0], clk, subWordOut0);
    sbox_sync sboxsyncsw2(word[15:8], clk, subWordOut1);
    sbox_sync sboxsyncsw3(word[23:16], clk, subWordOut2);
    sbox_sync sboxsyncsw4(word[31:24], clk, subWordOut3);
    assign subWordOut = {subWordOut3, subWordOut2, subWordOut1, subWordOut0};

endmodule

// WORKS AS DESIRED IN TB
module rotWord(input logic [31:0] word,
               output logic [31:0] rotWordOut);

    assign rotWordOut = {word[23:0], word[31:24]}; 
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
module shiftRows(input  logic [127:0] in,
                 // input logic clk,
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

module addRoundKey(input  logic [127:0] in,
                   input  logic [127:0] roundKey,
                   input logic clk,
                   output logic [127:0] out);

  //internal variable
  // logic [31:0] in0, in1, in2, in3;
  // logic [31:0] rkey0, rkey1, rkey2, rkey3;

  // find roundkey for each word 

  // logic [31:0] y0, y1, y2, y3;

  // assign {a0, a1, a2, a3} = a;
  // assign {rkey0, rkey1, rkey2, rkey3} = roundKey;

  // XOR operations for each word
  // assign out0 = in0 ^ rkey0;
  // assign out1 = in1 ^ rkey1;
  // assign out2 = in2 ^ rkey2;
  // assign out3 = in3 ^ rkey3;

  assign out = in ^ roundKey; 

endmodule

module keyExpansionRound(input logic [127:0] key,
                         input logic [3:0] j,
                         output logic [31:0] rkey0, rkey1, rkey2, rkey3);
    
    logic [31:0] key0, key1, key2, key3; 
    assign key0 = key[31:0];
    assign key1 = key[63:32];
    assign key2 = key[95:64];
    assign key3 = key[127:96];
    
    //round constants
    logic [320:0] Rcon;
    assign Rcon[0] = 32'h01000000;
    assign Rcon[1] = 32'h02000000;
    assign Rcon[2] = 32'h04000000;
    assign Rcon[3] = 32'h08000000;
    assign Rcon[4] = 32'h10000000;
    assign Rcon[5] = 32'h20000000;
    assign Rcon[6] = 32'h40000000;
    assign Rcon[7] = 32'h80000000;
    assign Rcon[8] = 32'h1B000000;
    assign Rcon[9] = 32'h36000000;

    // define words of roundkeys
    logic [31:0] swkey0, swkey1, swkey2, swkey3;
    logic [31:0] rotkey0, rotkey1, rotkey2, rotkey3;
    //logic [31:0] rkey0, rkey1, rkey2, rkey3;

    // call subword, rot word (need to repeat for EACH Rcon)
    rotword(key0, rotkey0);
    subword(rotkey0, clk, swkey0);
    assign rkey0 = key0 ^ swkey0 ^ Rcon[j];

    rotword(key1, rotkey1);
    subword(rotkey1, clk, swkey1);
    assign rkey1 = key1 ^ rkey0;

    rotword(key2, rotkey2);
    subword(rotkey2, clk, swkey2);
    assign rkey2 = key2 ^ rkey1;

    rotword(key3, rotkey3);
    subword(rotkey3, clk, swkey3);
    assign rkey3 = key3 ^ rkey2;


endmodule


// generate round key 
// Expansion of the given cipher key into 11 partial keys
module keyExpansion(input  logic [127:0] key,
                    input  logic clk,
                    output logic [127:0] roundKey);
//        [127:120] [95:88] [63:56] [31:24]     S0,0    S0,1    S0,2    S0,3
//        [119:112] [87:80] [55:48] [23:16]     S1,0    S1,1    S1,2    S1,3
//        [111:104] [79:72] [47:40] [15:8]      S2,0    S2,1    S2,2    S2,3
//        [103:96]  [71:64] [39:32] [7:0]       S3,0    S3,1    S3,2    S3,3
  logic [3:0] j = 0; // keeps track of round
  

  always_ff @(posedge clk)
    begin
    if (j <= 9)
      begin
        //keyExpansionRound(key, j, rkey0, rkey1, rkey2, rkey3);
      end

    j <= j+1;
    end
  //assign roundkey = {rkey0, rkey1, rkey2, rkey3};



    // i <- 0
    // while i <= Nk - 1 do:
      // w[i] <- key[4*i...4*i+3]
      // i <- i +1
    // end while
    // whiel i <= 4*Nr+3 do
      // temp <- w[i-1]
      // if i mod Nk = 0 then
        // temp <- SubWord(RotWord(temp)) XOR Rcon [i/Nk]
      // else if Nk > 6 and i mod Nk = 4 then
        // temp <- SubWord(temp)
      // end if
      // w[i] <- w[i - Nk] xor temp
      // i <- i+1
    // end while
    // return w

endmodule


