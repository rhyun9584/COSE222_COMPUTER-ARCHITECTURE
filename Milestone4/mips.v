`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, branch;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump;
  wire [3:0]  alucontrol;
  
  // ## Jeonghwa Yu: Start ##
  wire		  branch_inC, branch_outC;
  wire		  memwrite_outC;
  wire [1:0]  aluop_inC, aluop_outC;
  wire [31:0] instr_inC;
  wire [5:0]  funct_inC;
  wire		  controls;
  // ## Jeonghwa Yu: End ##

  // Instantiate Controller
  controller c(
	 // ## Jeonghwa Yu: Start ##
      .op         (instr_inC[31:26]), 
		.funct      (funct_inC), 
	 // ## Jeonghwa Yu: End ##
	 
		.zero       (zero),
		.signext    (signext),
		.shiftl16   (shiftl16),
	 	.memtoreg   (memtoreg),
		.memwrite   (memwrite_outC),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
		.jal        (jal), // new!
		.alucontrol (alucontrol),
		
	 // ## Jeonghwa Yu: Start ##
		.branch_in  (branch_inC),
		.branch_out (branch_outC),
		.aluop_in	(aluop_inC),
		.aluop_out	(aluop_outC),
		.controls	(controls)
	 // ## Jeonghwa Yu: End ##		
		);

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
	 .jal        (jal), // new!
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata),
	 
	 // ## Jeonghwa Yu: Start ##
	 .branch		 (branch_outC),
	 .branch_out (branch_inC),
	 .memwrite	 (memwrite_outC),
  .memwrite_out (memwrite),
	 .aluop		 (aluop_outC),
	 .aluop_EX	 (aluop_inC),
	 .instr_ID	 (instr_inC),
	 .funct_EX	 (funct_inC),
	 .controls   (controls)
	 // ## Jeonghwa Yu: End ##
	 );

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump,
                  output       jal, // new!
                  output [3:0] alucontrol,
						
						// ## Jeonghwa Yu: Start ##
						input			 branch_in,
						output		 branch_out,
						input	 [1:0] aluop_in,
						output [1:0] aluop_out,
						input			 controls
						// ## Jeonghwa Yu: End ##
						);

//  wire [1:0] aluop;
  wire       bne, Rzero; // new!

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
	 // ## Jeonghwa Yu: Start ##
    .branch   (branch_out),
	 // ## Jeonghwa Yu: End ##
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
	 .bne      (bne), // new
	 .jal      (jal), // new
	 // ## Jeonghwa Yu: Start ##
    .aluop    (aluop_out),
  .controlsrc (controls)
	 // ## Jeonghwa Yu: End ##
	 );

  aludec ad( 
    .funct      (funct),
	 // ## Jeonghwa Yu: Start ##
    .aluop      (aluop_in), 
	 // ## Jeonghwa Yu: End ##
    .alucontrol (alucontrol));

  assign Rzero = bne? ~zero : zero; // new

  // ## Jeonghwa Yu: Start ##
  assign pcsrc = branch_in & Rzero ; // new
  // ## Jeonghwa Yu: End ##

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,
               output       bne, jal, // new
               output [1:0] aluop,
					// ## Jeonghwa Yu: Start ##
					input			 controlsrc
					// ## Jeonghwa Yu: End ##
					);

  reg [12:0] controls;

  // ###### Jeonghwa Yu: Start ######
  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, bne, memwrite,
          memtoreg, jump, jal, aluop} = controlsrc ? 13'b0 : controls;
  // ###### Jeonghwa Yu: End ######

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 13'b0011000000011; // Rtype
      6'b100011: controls <= #`mydelay 13'b1010100010000; // LW
      6'b101011: controls <= #`mydelay 13'b1000100100000; // SW
      6'b000100: controls <= #`mydelay 13'b1000010000001; // BEQ
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b1010100000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 13'b0010100000010; // ORI
      6'b001111: controls <= #`mydelay 13'b0110100000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000001000; // J

    	6'b000101: controls <= #`mydelay 13'b1000011000001; // BNE
      6'b000011: controls <= #`mydelay 13'b0010000001100; // JAL 

      default:   controls <= #`mydelay 13'bxxxxxxxxxxxxx; // ???
    endcase

endmodule
  
module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg [3:0] alucontrol);

  always @(*)
    case(aluop)
      2'b00: alucontrol <= #`mydelay 4'b0010;  // add
      2'b01: alucontrol <= #`mydelay 4'b0110;  // sub
      2'b10: alucontrol <= #`mydelay 4'b0001;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 4'b0110; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 4'b0001; // OR
          6'b101010: alucontrol <= #`mydelay 4'b0111; // SLT

          6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU
          6'b001000: alucontrol <= #`mydelay 4'b1010; // JR
          6'b000000: alucontrol <= #`mydelay 4'b0000; // NOP?

          default:   alucontrol <= #`mydelay 4'bxxxx; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump,
                input         jal,
                input  [3:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata,
					 // ## Jeonghwa Yu: Start ##
					 input			branch,
					 output			branch_out,
					 input			memwrite,
					 output			memwrite_out,
					 input  [1:0]  aluop,
					 output [1:0]	aluop_EX,
					 output [31:0] instr_ID,
					 output [5:0] 	funct_EX,
					 output			controls
					 // ## Jeonghwa Yu: End ##
					 );

  wire [4:0]  writereg;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
//  wire        shift;  => not used
  
  wire [4:0]  writereg_wrmux; // new in 3, bit change 32->5 in 4
  wire [31:0] result_resmux, pcnext_pcb; // new! in 3
  
  //	######	Jeonghwa Yu: Start	######
  wire [31:0] pcplus4_ID, writedata_ID;
  
  wire [31:0] pcplus4_EX, srca_EX, aluout_EX, writedata_EX, signimm_EX, writedata_out, instr_EX;
  wire [4:0]  wrmux_in0, wrmux_in1, rs_EX;
 // wire		  regdst_EX, alusrc_EX, shiftl16_EX, zero_EX;
  wire [2:0]  ControlWB_E, ControlM;
  
  wire [31:0] pcbranch_M, pcplus4_M, instr_M;
  wire [4:0]  writereg_M;
  wire [2:0]  ControlWB_M;
  wire		  jump_M;
  
  wire [31:0] readdata_WB, aluout_WB;
  wire [4:0]  writereg_WB;
  wire		  regwrite_WB, memtoreg_WB, jal_WB;
  
  wire [31:0] srca_out;
  wire [1:0]  srcasrc, srcbsrc;
  wire		  IFIDwrite, PCwrite;
  wire		  controls_EX, controls_M, controls_WB;
  //	######	Jeonghwa Yu: End	######
  
  // next PC logic
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
	 .en	  (PCwrite), // add in milestone4
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm_EX), // change in milestone4
    .y (signimmsh));

  adder pcadd2(
    .a (pcplus4_EX), // change in milestone4
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcnext_pcb),
    .d1  (pcbranch_M), // change in milestone4
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4_M[31:28], instr_M[25:0], 2'b00}), // change in milestone4
    .s    (jump_M), // change in milestone4
    .y    (pcnext));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite_WB), // change in milestone4
    .ra1     (instr_ID[25:21]), // change in milestone4
    .ra2     (instr_ID[20:16]), // change in milestone4
    .wa      (writereg_WB), // change in milestone4
    .wd      (result),
    .rd1     (srca),
    .rd2     (writedata_ID)); // change in milestone4

  mux2 #(5) wrmux( 
    .d0  (instr_EX[20:16]), // Rt  change in milestone4
    .d1  (instr_EX[15:11]), // Rd  change in milestone4
    .s   (regdst_EX), // change in milestone4
    .y   (writereg_wrmux));

  mux2 #(32) resmux(
    .d0 (aluout_WB), // change in milestone4
    .d1 (readdata_WB), // change in milestone4
    .s  (memtoreg_WB), // change in milestone4
    .y  (result_resmux));

  sign_zero_ext sze(
    .a       (instr_ID[15:0]), // change in milestone4
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm_EX[31:0]), // change in milestone4
    .shiftl16  (shiftl16_EX), // change in milestone4
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata_out), // change in milestone4
    .d1 (shiftedimm[31:0]),
    .s  (alusrc_EX), // change in milestone4
    .y  (srcb));

  alu alu(
    .a       (srca_out), // change in milestone4
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout_EX), // change in milestone4
    .zero    (zero_EX)); // change in milestone4
    
  mux2 #(5) jalamux(
    .d0 (writereg_wrmux),
    .d1 (5'b11111),
    .s  (ControlWB_E[1]), // jal_EX  change in milestone4
    .y  (writereg));
  
  mux2 #(32) jaldmux(
    .d0 (result_resmux),
    .d1 (pcplus4),
    .s  (jal_WB), // change in milestone4
    .y  (result));

  mux_jr pcjrmux(
    .d0 (pcplus4),
    .d1 (aluout),
    .s  (alucontrol),
    .y  (pcnext_pcb));

// ######	Jeonghwa Yu: Start	######
  mux3 #(32) srcamux3(
    .d0  (srca_EX),
	 .d1  (aluout),
	 .d2  (result),
	 .s   (srcasrc),
	 .y   (srca_out));
	 
  mux3 #(32) srcbmux3(
    .d0  (writedata_EX),
	 .d1  (aluout),
	 .d2  (result),
	 .s   (srcbsrc),
	 .y   (writedata_out));
	 
  fwunit f(
	 .rt		 (instr_EX[20:16]),
	 .rs		 (instr_EX[25:21]),
	 .MEMrd	 (writereg_M),
	 .WBrd	 (writereg_WB),
	 .hazardM (controls_M),
	.hazardWB (controls_WB),
	 .srcasrc (srcasrc),
	 .srcbsrc (srcbsrc)
	 );
	 
  hzdunit h(
    .rt			(instr_EX[20:16]),
	 .rs_ID		(instr_ID[25:21]),
	 .rt_ID	   (instr_ID[20:16]),
	 .memread   (ControlWB_E[0]), // memtoreg_EX
	 .controls  (controls),
	 .IFIDwrite (IFIDwrite),
	 .PCwrite   (PCwrite));
  
  flopenr #(64) IFtoID(
    .clk   (clk),
    .reset (reset),
    .en	  (IFIDwrite),
    .d     ({pcplus4	  , instr}),
    .q     ({pcplus4_ID, instr_ID}));

  flopenr #(178) IDtoEX(
    .clk   (clk),
    .reset (reset),
	 .en	  (1'b1),
    .d     ({regwrite, jal, memtoreg, branch, memwrite, jump,
				 regdst	 ,	aluop   , alusrc	 , pcplus4_ID, srca	 , shiftl16   , controls   ,
				 writedata_ID, signimm   , signimm[5:0], instr_ID}),
    .q     ({ControlWB_E				, ControlM					,
				 regdst_EX, aluop_EX, alusrc_EX, pcplus4_EX, srca_EX, shiftl16_EX, controls_EX,
				 writedata_EX, signimm_EX, funct_EX    , instr_EX}));

  flopenr #(173) EXtoMEM(
    .clk   (clk),
    .reset (reset),
	 .en	  (1'b1),
    .d     ({ControlWB_E, ControlM						 		  ,
				 pcbranch  , zero_EX, aluout_EX, writedata_out, writereg  , controls_EX, pcplus4_EX, instr_EX}),
    .q     ({ControlWB_M, branch_out , memwrite_out, jump_M,
				 pcbranch_M, zero	 , aluout	, writedata    , writereg_M, controls_M , pcplus4_M , instr_M}));	 
	
  flopenr #(78) MEMtoWB(
    .clk   (clk),
    .reset (reset),
	 .en	  (1'b1),
    .d     ({ControlWB_M							, readdata   , aluout	, writereg_M , controls_M}),
    .q     ({regwrite_WB, jal_WB, memtoreg_WB, readdata_WB, aluout_WB, writereg_WB, controls_WB})); 
	
// ######	Jeonghwa Yu: End	######
    
endmodule
