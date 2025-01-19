module adder #(parameter WIDTH=8)
(input logic [WIDTH-1:0] a, b,output logic [WIDTH-1:0] y);
assign y = a + b;
endmodule
module FullAdder(input logic Cin, input logic [3:0]A,B, output logic [3:0]S, output logic cout);
logic [3:0]p,g;
logic n0,n1,n2;
logic [3:0]y;
logic [3:0] BI;
assign BI = ~B;
assign y=Cin? BI:B;
always_comb
begin
p[0] = A[0] ^ y[0];
g[0] = A[0] & y[0];
S[0] = p[0] ^ Cin;
n0 = g[0] | (p[0] & Cin);
p[1] = A[1] ^ y[1];
g[1] = A[1] & y[1];
S[1] = p[1] ^ n0;
n1 = g[1] | (p[1] & n0);
p[2] = A[2] ^ y[2];
g[2] = A[2] & y[2];
S[2] = p[2] ^ n1;
n2 = g[2] | (p[2] & n1);
p[3] = A[3] ^ y[3];
g[3] = A[3] & y[3];
S[3] = p[3] ^ n2;
cout = g[3] | p[3] & g[2];
end
endmodule
module fulladder_32 (input logic cin ,input logic  [31:0]A,B, output logic [31:0]sum, output logic cout);
logic n1,n2,n3,n4,n5,n6,n7,n8;
logic [31:0]R3;
FullAdder add0_3 (cin,A[3:0],B[3:0], R3[3:0],n1);
FullAdder add4_7 (n1,A[7:4],B[7:4], R3[7:4],n2);
FullAdder add8_11 (n2,A[11:8],B[11:8], R3[11:8],n3);
FullAdder add12_5 (n3,A[15:12],B[15:12], R3[15:12],n4);
FullAdder add16_19 (n4,A[19:16],B[19:16], R3[19:16],n5);
FullAdder add20_23 (n5,A[23:20],B[23:20], R3[23:20],n6);
FullAdder add24_27 (n6,A[27:24],B[27:24], R3[27:24],n7);
FullAdder add28_31 (n7,A[31:28],B[31:28], R3[31:28],n8);
assign cout = n8;
assign sum = R3;
endmodule
module invert (input logic [31:0]a , output logic [31:0]y);
assign y =~a;
endmodule

module AND (input logic [31:0]a, [31:0]b , output logic [31:0]y);
assign y =a&b;
endmodule

module OR (input logic [31:0]a, [31:0]b , output logic [31:0]y);
assign y =a|b;
endmodule

module XOR (input logic a, b , output logic y);
assign y = a &~b | ~a&b;
endmodule
module XOR_32 (input logic [31:0]a, [31:0]b , output logic [31:0]y);
assign y = a &~b | ~a&b;
endmodule
module BIC_32 (input logic [31:0]a, [31:0]b , output logic [31:0]y);
assign y = a &~b;
endmodule
module XNOR (input logic a, b, output logic y);
logic n;
assign n = a &~b | ~a&b;
assign y =~n;
endmodule

module NAND (input logic a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31 , output logic y);
assign y = a0&a1&a2&a3&a4&a5&a6&a7&a8&a9&a10&a11&a12&a13&a14&a15&a16&a17&a18&a19&a20&a21&a22&a23&a24&a25&a26&a27&a28&a29&a30&a31;
endmodule

module ALU(input logic [31:0]A,[31:0]B, input logic [2:0]S0 ,output logic [31:0]D0, output logic D1,D2,D3,D4);
logic n1,n2,n3,n4,n5,n6,n7,n8,n9,n11,n12,n13,n14;
logic [31:0]R1,R2,R3,R4,R5,R6,R7,R8;
invert inverter (B,R1);
Mux2x1#(32) add_or_sub (R1,B,S0[0],R2); 
FullAdder add0_3 (S0[0],A[3:0],B[3:0], R3[3:0],n1);
FullAdder add4_7 (n1,A[7:4],B[7:4], R3[7:4],n2);
FullAdder add8_11 (n2,A[11:8],B[11:8], R3[11:8],n3);
FullAdder add12_5 (n3,A[15:12],B[15:12], R3[15:12],n4);
FullAdder add16_19 (n4,A[19:16],B[19:16], R3[19:16],n5);
FullAdder add20_23 (n5,A[23:20],B[23:20], R3[23:20],n6);
FullAdder add24_27 (n6,A[27:24],B[27:24], R3[27:24],n7);
FullAdder add28_31 (n7,A[31:28],B[31:28], R3[31:28],n8);
AND and_g (A,B, R4);
OR or_g (A,B,R5);
BIC_32 clear_g (A,B,R7);
XOR_32 xor_g0 (A,B,R8);

 always_comb 
 casex (S0) 
 3'b00x: R6 = R3; 
 3'b010: R6 = R4; 
 3'b011: R6 = R5; 
 3'b110: R6 = R7; 
 3'b100: R6 = R8; 
 endcase
NAND zero_flag ( R6[0],R6[1],R6[2],R6[3],R6[4],R6[5],R6[6],R6[7],R6[8],R6[9],R6[10],R6[11],R6[12],R6[13],R6[14],R6[15],R6[16],R6[17],R6[18],R6[19],R6[20],R6[21],R6[22],R6[23],R6[24],R6[25],R6[26],R6[27],R6[28],R6[29],R6[30],R6[31],n9);
XOR port (A[31], R6[31],n11);
XNOR port1 (A[31], B[31],n12);
XNOR port2 (n12, S0[0],n13);
assign n14 = n13&n11&S0[0];
assign D0 = R6;
assign D1 = n9;
assign D2 = R6[31];
assign D3 = n8&S0[1];
assign D4 = n14;
endmodule
module alu(input logic [31:0] a, b, 
 input logic [2:0] ALUControl, 
 output logic [31:0] Result, 
 output logic [3:0] Flags); 
 logic neg, zero, carry, overflow; 
 logic [31:0] condinvb; 
 logic [32:0] sum; 
 assign condinvb = ALUControl[0] ? ~b : b; 
 assign sum = a + condinvb + ALUControl[0]; 
 always_comb 
 casex (ALUControl[1:0]) 
 2'b00?: Result = sum; 
 2'b010: Result = a & b; 
 2'b011: Result = a | b; 
 2'b100: Result = a &~b | ~a&b; 
 2'b110: Result = a & ~b; 
 endcase 
 assign neg = Result[31]; 
 assign zero = (Result == 32'b0); 
 assign carry = (ALUControl[1] == 1'b0) & sum[32]; 
 assign overflow = (ALUControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ 
ALUControl[0]) & 
 (a[31] ^ sum[31]); 
 assign Flags = {neg, zero, carry, overflow}; 
endmodule 
module equality #(parameter WIDTH = 8)(input logic [WIDTH-1:0] a, b, output logic y); 
 assign y = (a == b); 
endmodule 
module RESETTABLE_FLIP_FLOP #(parameter WIDTH = 8)(input logic clk,input logic reset,input logic [WIDTH-1:0]D,output logic [WIDTH-1:0]Q);
always_ff @(posedge clk, posedge reset)
if (reset) Q <= 0;
else Q <= D;

endmodule

module RESETTABLE_FLIP_FLOP_WITH_ENABLE #(parameter WIDTH = 8)(input logic clk,input logic reset,input logic EN,input logic [WIDTH-1:0]D,output logic [WIDTH-1:0]Q);
always_ff @(posedge clk, posedge reset)
if (reset) Q <= 0;
else if (EN) Q <= D;
endmodule
module RESETTABLE_FLIP_FLOP_WITH_CLEAR #(parameter WIDTH = 8)(input logic clk,input logic reset,clear,input logic [WIDTH-1:0]D,output logic [WIDTH-1:0]Q);
always_ff @(posedge clk, posedge reset)
if (reset) Q <= 0;
else if (clear) Q <= 0;
else Q <= D;

endmodule

module RESETTABLE_FLIP_FLOP_WITH_ENABLE_WITH_CLEAR #(parameter WIDTH = 8)(input logic clk,input logic reset,clear,input logic EN,input logic [WIDTH-1:0]D,output logic [WIDTH-1:0]Q);
always_ff @(posedge clk, posedge reset)
if (reset) Q <= 0;
else if (EN)
if (clear) Q <= 0;
else Q <= D;
endmodule
module Mux2x1#(parameter WIDTH = 8)(input logic [WIDTH-1:0]a, [WIDTH-1:0]b ,input logic s0, output logic [WIDTH-1:0]y);
assign y = s0? b: a;
endmodule

module Mu3x2#(parameter WIDTH = 8)(input logic [WIDTH-1:0]a, [WIDTH-1:0]b ,[WIDTH-1:0]c,input logic [1:0]s0 ,output logic [WIDTH-1:0]y);
assign y = s0[1] ? c : (s0[0] ? b : a); 
endmodule
module extender(input logic [23:0] inimm, input logic [1:0] immsrc, output logic [31:0] extimm);
logic c;
assign c=0;
always_comb
case(immsrc)
2'b00: extimm = {{24{c}}, inimm[7:0]};
2'b01: extimm = {{20{c}}, inimm[11:0]};
2'b10: extimm = {{8{inimm[23]}}, inimm[23:0]};
default: extimm = 32'bx;
endcase
endmodule

module reg_file (input logic clk , input logic [3:0]A1,A2,A3, input logic [31:0]WD3,R15,output logic [31:0]RD1,RD2, input logic EN);
logic [31:0]R[14:0];
always_ff @(posedge clk)
if (EN)  R[A3] <= WD3;
assign RD1= (A1==4'b1111) ? R15: R[A1];
assign RD2= (A2==4'b1111) ? R15: R[A2];
endmodule

module datapath(input logic clk, reset, PCsrcW,BranchtakenE,input logic [1:0] RegsrcD, 
input logic RegwriteW,  input logic [1:0]ImmsrcD, input logic ALUsrcE, input logic [2:0] ALUcontrolE, 
  output logic [3:0] ALUflagsE, input logic MemtoregW,  input logic [31:0] InstrF, ReaddataM,
 output logic [31:0] PCF, InstrD,ALUoutM, WritedataM,


 // the input and out part of the extra hazard unit
input logic StallF, StallD, FlushD, input logic [1:0] ForwardAE, ForwardBE, 
output logic Match_1E_M, Match_1E_W, Match_2E_M,Match_2E_W, Match_12D_E );

logic [31:0] PCplus4F, PCnextFmux1, PCnextFmux2,RD1D, RD2D, PCplus8D,ExtimmD,RD1E, RD2E, ExtimmE, SrcAE, SrcBE, WritedataE, ALUresultE,ReaddataW, ALUoutW, ResultW; 
logic [3:0] RA1D, RA2D, RA1E, RA2E, WA3E, WA3M, WA3W; 
logic Match_1D_E, Match_2D_E,cin,cout; 
 // fetch stage part
 Mux2x1 #(32) fetchmux1(PCplus4F, ResultW, PCsrcW, PCnextFmux1); 
 Mux2x1 #(32) fetchmux2(PCnextFmux1, ALUresultE, BranchtakenE, PCnextFmux2); 
 RESETTABLE_FLIP_FLOP_WITH_ENABLE #(32) fetchreg(clk, reset, ~StallF, PCnextFmux2, PCF); 
adder#(32) fetchadder(PCF, 32'd4, PCplus4F); 
 
 // decode Stage part
 assign PCplus8D = PCplus4F;
 RESETTABLE_FLIP_FLOP_WITH_ENABLE_WITH_CLEAR #(32) decodereg(clk, reset, ~StallD, FlushD, InstrF, InstrD); 
 Mux2x1 #(4) decodemux1(InstrD[19:16], 4'b1111, RegsrcD[0], RA1D); 
 Mux2x1 #(4) decodemux2(InstrD[3:0], InstrD[15:12], RegsrcD[1], RA2D); 
 reg_file decoderegfile(clk, RA1D, RA2D,WA3W, ResultW, PCplus8D,RD1D, RD2D, RegwriteW); 
 extender decodeextender(InstrD[23:0], ImmsrcD, ExtimmD); 

 // execute stage part
 RESETTABLE_FLIP_FLOP #(32) executereg1(clk, reset, RD1D, RD1E); 
 RESETTABLE_FLIP_FLOP #(32) executereg2(clk, reset, RD2D, RD2E); 
 RESETTABLE_FLIP_FLOP #(32) executereg3(clk, reset, ExtimmD, ExtimmE);
 RESETTABLE_FLIP_FLOP #(4) executereg4(clk, reset, InstrD[15:12], WA3E); 
 RESETTABLE_FLIP_FLOP #(4) executereg5(clk, reset, RA1D, RA1E); 
 RESETTABLE_FLIP_FLOP #(4) executereg6(clk, reset, RA2D, RA2E); 
 Mu3x2 #(32) executemux1(RD1E, ResultW, ALUoutM, ForwardAE, SrcAE); 
 Mu3x2 #(32) executemux2(RD2E, ResultW, ALUoutM, ForwardBE, WritedataE); 
 Mux2x1 #(32) executemux3(WritedataE, ExtimmE, ALUsrcE, SrcBE); 
 alu aluexcute(SrcAE, SrcBE, ALUcontrolE, ALUresultE, ALUflagsE); 
// memory stage part
RESETTABLE_FLIP_FLOP #(32) mem1(clk, reset, ALUresultE, ALUoutM); 
RESETTABLE_FLIP_FLOP #(32) mem2(clk, reset, WritedataE, WritedataM); 
RESETTABLE_FLIP_FLOP #(4) mem3(clk, reset, WA3E, WA3M); 
 
 // writeback stage part
 RESETTABLE_FLIP_FLOP #(32) writeback1(clk, reset, ALUoutM, ALUoutW); 
 RESETTABLE_FLIP_FLOP #(32) writeback2(clk, reset, ReaddataM, ReaddataW); 
 RESETTABLE_FLIP_FLOP #(4) writeback3(clk, reset, WA3M, WA3W); 
 Mux2x1 #(32) writebackmux(ALUoutW, ReaddataW, MemtoregW, ResultW); 
 
 equality #(4) eq1(WA3M, RA1E, Match_1E_M); 
 equality #(4) eq2(WA3W, RA1E, Match_1E_W); 
 equality #(4) eq3(WA3M, RA2E, Match_2E_M); 
 equality #(4) eq4(WA3W, RA2E, Match_2E_W); 
 equality #(4) eq5(WA3E, RA1D, Match_1D_E); 
 equality #(4) eq6(WA3E, RA2D, Match_2D_E); 
 assign Match_12D_E = Match_1D_E | Match_2D_E; 
 
endmodule
module conditional(input logic [3:0] Cond, Flags, ALUFlags, input logic [1:0] FlagsWrite, 
 output logic CondEx, 
 output logic [3:0] FlagsNext); 
 
 logic neg, zero, carry, overflow, ge; 
 
 assign {neg, zero, carry, overflow} = Flags; 
 assign ge = (neg == overflow); 
 
 always_comb 
 case(Cond) 
 4'b0000: CondEx = zero; // EQ 
 4'b0001: CondEx = ~zero; // NE 
 4'b0010: CondEx = carry; // CS 
 4'b0011: CondEx = ~carry; // CC 
 4'b0100: CondEx = neg; // MI 
 4'b0101: CondEx = ~neg; // PL 
 4'b0110: CondEx = overflow; // VS 
 4'b0111: CondEx = ~overflow; // VC 
 4'b1000: CondEx = carry & ~zero; // HI 
 4'b1001: CondEx = ~(carry & ~zero); // LS 
 4'b1010: CondEx = ge; // GE
 4'b1011: CondEx = ~ge; // LT 
 4'b1100: CondEx = ~zero & ge; // GT 
 4'b1101: CondEx = ~(~zero & ge); // LE 
 4'b1110: CondEx = 1'b1; // Always
 default: CondEx = 1'bx; // undefined 
 endcase 
 
 assign FlagsNext[3:2] = (FlagsWrite[1] & CondEx) ? ALUFlags[3:2] : 
Flags[3:2]; 
 assign FlagsNext[1:0] = (FlagsWrite[0] & CondEx) ? ALUFlags[1:0] : 
Flags[1:0]; 
endmodule 
module controller(input logic clk, reset,input logic [31:12] InstrD,input logic [3:0] ALUFlagsE,
output logic [1:0] RegSrcD, ImmsrcD,
output logic ALUsrcE, BranchtakenE,output logic [2:0] ALUcontrolE,output logic MemwriteM,
output logic MemtoregW, PCsrcW, RegwriteW,
output logic RegwriteM, MemtoregE,output logic PC_write_pend_F,input logic FlushE);
logic [9:0] controlsD;
logic CondExE, ALUOpD;
logic [2:0] ALUControlD;
logic ALUsrcD;
logic MemtoregD, MemtoregM;
logic RegwriteD, RegwriteE, RegWriteGatedE;
logic MemwriteD, MemwriteE, MemWriteGatedE;
logic BranchD, BranchE;
logic [1:0] FlagwriteD, FlagwriteE;
logic PCsrcD, PCsrcE, PCsrcM;
logic [3:0] FlagsE, FlagsNextE, CondE;

// Decoder Stage

always_comb
casex(InstrD[27:26])
2'b00: if (InstrD[25]) controlsD = 10'b0000101001; // DP imm
else controlsD = 10'b0000001001; // DP reg
2'b01: if (InstrD[20]) controlsD = 10'b0001111000; // LDR
else controlsD = 10'b1001110100; // STR
2'b10: controlsD = 10'b0110100010; // B
default: controlsD = 10'bx; 
endcase
assign {RegSrcD, ImmsrcD, ALUsrcD, MemtoregD,
RegwriteD, MemwriteD, BranchD, ALUOpD} = controlsD;
always_comb
if (ALUOpD) begin //Data-processing Instr
case(InstrD[24:21])
4'b0100: ALUControlD = 3'b000; // ADD
4'b0010: ALUControlD = 3'b001; // SUB
4'b0000: ALUControlD = 3'b010; // AND
4'b1100: ALUControlD = 3'b011; // ORR
4'b0001: ALUControlD = 3'b100; // EOR
4'b1110: ALUControlD = 3'b110; // BIC
default: ALUControlD = 3'bx; // unimplemnt
endcase
FlagwriteD[1] = InstrD[20]; // update N and Z Flags if S bit is set
FlagwriteD[0] = InstrD[20] & (ALUControlD == 3'bx00 | ALUControlD== 3'bx01);
end else begin
ALUControlD = 3'bx00; // perform addition for NDP instructions
FlagwriteD = 3'bx00; // Fixed Flags (don't update)
end
// Execute stage
assign PCsrcD = (((InstrD[15:12] == 4'b1111) & RegwriteD) | BranchD);
// Execute stage
RESETTABLE_FLIP_FLOP_WITH_CLEAR #(7) flushedregsE(clk, reset, FlushE,
{FlagwriteD, BranchD, MemwriteD, RegwriteD,
PCsrcD, MemtoregD},
{FlagwriteE, BranchE, MemwriteE, RegwriteE,
PCsrcE, MemtoregE});
RESETTABLE_FLIP_FLOP #(4) regsE(clk, reset,{ALUsrcD, ALUControlD},{ALUsrcE, ALUcontrolE});
RESETTABLE_FLIP_FLOP #(4) condregE(clk, reset, InstrD[31:28], CondE);
RESETTABLE_FLIP_FLOP #(4) flagsreg(clk, reset, FlagsNextE, FlagsE);

// write and Branch Conditonal unit
conditional Cond(CondE, FlagsE, ALUFlagsE, FlagwriteE, CondExE,FlagsNextE);
assign BranchtakenE = BranchE & CondExE;
assign RegWriteGatedE = RegwriteE & CondExE;
assign MemWriteGatedE = MemwriteE & CondExE;
assign PCSrcGatedE = PCsrcE & CondExE;

// Memory stage
RESETTABLE_FLIP_FLOP #(4) regsM(clk, reset,{MemWriteGatedE, MemtoregE, RegWriteGatedE,PCSrcGatedE},{MemwriteM, MemtoregM, RegwriteM, PCsrcM});
// Writeback stage
RESETTABLE_FLIP_FLOP #(3) regsW(clk, reset,{MemtoregM, RegwriteM, PCsrcM},{MemtoregW, RegwriteW, PCsrcW});
// Hazard Prediction Unit
assign PC_write_pend_F = PCsrcD | PCsrcE | PCsrcM;
endmodule
module hazard(input logic clk, reset,input logic Match_1E_M, Match_1E_W, Match_2E_M,Match_2E_W, Match_12D_E,
input logic RegWriteM, RegWriteW,input logic BranchTakenE, MemtoRegE,input logic PCWrPendingF, PCSrcW,
output logic [1:0] ForwardAE, ForwardBE,output logic StallF, StallD,output logic FlushD, FlushE);
logic ldrStallD;
always_comb begin
if (Match_1E_M & RegWriteM) ForwardAE = 2'b10;
else if (Match_1E_W & RegWriteW) ForwardAE = 2'b01;
else ForwardAE = 2'b00;
if (Match_2E_M & RegWriteM) ForwardBE = 2'b10;
else if (Match_2E_W & RegWriteW) ForwardBE = 2'b01;
else ForwardBE = 2'b00;
end
assign ldrStallD = Match_12D_E & MemtoRegE;
assign StallD = ldrStallD;
assign StallF = ldrStallD | PCWrPendingF;
assign FlushE = ldrStallD | BranchTakenE;
assign FlushD = PCWrPendingF | PCSrcW | BranchTakenE;
endmodule

module arm(input logic clk, reset,  input logic [31:0] ReaddataM, 
output logic [31:0] ALUoutM, WritedataM, output logic MemwriteM, 
input logic [31:0] InstrF, output logic [31:0] PCF);  


 logic [1:0] RegsrcD, ImmsrcD;
 logic [2:0]ALUcontrolE;
 logic ALUsrcE, BranchtakenE, MemtoregW, PCsrcW; 
 logic [3:0] ALUflagsE; 
 logic [31:0] InstrD; 
 
 logic StallF, StallD, FlushD, FlushE; 
 logic [1:0] ForwardAE, ForwardBE; 
 logic RegwriteM, RegwriteW,MemtoregE, PC_write_pend_F; 
 logic Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W,Match_12D_E; 

 controller control1(clk, reset, InstrD[31:12], ALUflagsE, 
 RegsrcD, ImmsrcD, 
 ALUsrcE, BranchtakenE, ALUcontrolE, 
 MemwriteM, 
 MemtoregW, PCsrcW, RegwriteW, 
 RegwriteM, MemtoregE, PC_write_pend_F, 
 FlushE); 
 datapath dat1(clk, reset,  PCsrcW,BranchtakenE, RegsrcD, RegwriteW,   ImmsrcD  ,ALUsrcE,  ALUcontrolE,  ALUflagsE,  MemtoregW,InstrF,ReaddataM,  PCF,  InstrD,  ALUoutM, WritedataM, 

StallF, StallD, FlushD,ForwardAE, ForwardBE,Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W, Match_12D_E  ); 
hazard haz1(clk, reset, Match_1E_M, Match_1E_W, Match_2E_M, Match_2E_W, 
Match_12D_E, 
 RegwriteM, RegwriteW, BranchtakenE, MemtoregE, 
 PC_write_pend_F, PCsrcW, 
 ForwardAE, ForwardBE, 
 StallF, StallD, FlushD, FlushE); 
endmodule

// ARM pipelined processor 
module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 7) begin
          $display("Simulation succeeded");
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteDataM, DataAdrM, 
           output logic        MemWriteM);

  logic [31:0] PCF, InstrF, ReadDataM;
  
  // instantiate processor and memories
arm armprocessor(clk,reset,ReadDataM,DataAdrM,WriteDataM,MemWriteM,InstrF,PCF);  
  imem imem(PCF, InstrF);
  dmem dmem(clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
endmodule

module dmem(input logic clk, we,input logic [31:0] a, wd,output logic [31:0] rd);
logic [31:0] RAM[32:0];
initial
$readmemh("datafile.dat",RAM);
assign rd = RAM[a[22:2]]; 
always_ff @(posedge clk)
if (we) RAM[a[22:2]] <= wd;
endmodule
module imem(input logic [31:0] a,output logic [31:0] rd);
logic [31:0] RAM[32:0];
initial
$readmemh("infile.dat",RAM);
assign rd = RAM[a[22:2]];
endmodule
module Processor(input logic clk, reset,output logic [31:0] WriteDataM, DataAdrM,output logic MemWriteM,
output logic [31:0] PCF,InstrF, ReadDataM);
arm ARM(clk, reset, ReadDataM,DataAdrM, WriteDataM,MemWriteM, InstrF, PCF);
imem imem(PCF, InstrF);
dmem dmem(clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
endmodule