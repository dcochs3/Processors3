module Project(
  input        CLOCK_50,
  input        RESET_N,
  input  [3:0] KEY,
  input  [9:0] SW,
  output [6:0] HEX0,
  output [6:0] HEX1,
  output [6:0] HEX2,
  output [6:0] HEX3,
  output [6:0] HEX4,
  output [6:0] HEX5,
  output [9:0] LEDR
);

  parameter DBITS    = 32;
  parameter INSTSIZE = 32'd4;
  parameter INSTBITS = 32;
  parameter REGNOBITS = 4;
  parameter REGWORDS = (1 << REGNOBITS);
  parameter IMMBITS  = 16;
  parameter STARTPC  = 32'h100;
  parameter ADDRHEX  = 32'hFFFFF000;
  parameter ADDRLEDR = 32'hFFFFF020;
  parameter ADDRKEY  = 32'hFFFFF080;
  parameter ADDRSW   = 32'hFFFFF090;

  // Change this to fmedian2.mif before submitting
  parameter IMEMINITFILE = "Test.mif";
  //parameter IMEMINITFILE = "fmedian2.mif";
  
  parameter IMEMADDRBITS = 16;
  parameter IMEMWORDBITS = 2;
  parameter IMEMWORDS	 = (1 << (IMEMADDRBITS - IMEMWORDBITS));
  parameter DMEMADDRBITS = 16;
  parameter DMEMWORDBITS = 2;
  parameter DMEMWORDS	 = (1 << (DMEMADDRBITS - DMEMWORDBITS));
   
  parameter OP1BITS  = 6;
  parameter OP1_ALUR = 6'b000000;
  parameter OP1_BEQ  = 6'b001000;
  parameter OP1_BLT  = 6'b001001;
  parameter OP1_BLE  = 6'b001010;
  parameter OP1_BNE  = 6'b001011;
  parameter OP1_JAL  = 6'b001100;
  parameter OP1_LW   = 6'b010010;
  parameter OP1_SW   = 6'b011010;
  parameter OP1_ADDI = 6'b100000;
  parameter OP1_ANDI = 6'b100100;
  parameter OP1_ORI  = 6'b100101;
  parameter OP1_XORI = 6'b100110;
  
  // Add parameters for secondary opcode values 
  /* OP2 */
  parameter OP2BITS  = 8;
  parameter OP2_EQ   = 8'b00001000;
  parameter OP2_LT   = 8'b00001001;
  parameter OP2_LE   = 8'b00001010;
  parameter OP2_NE   = 8'b00001011;
  parameter OP2_ADD  = 8'b00100000;
  parameter OP2_AND  = 8'b00100100;
  parameter OP2_OR   = 8'b00100101;
  parameter OP2_XOR  = 8'b00100110;
  parameter OP2_SUB  = 8'b00101000;
  parameter OP2_NAND = 8'b00101100;
  parameter OP2_NOR  = 8'b00101101;
  parameter OP2_NXOR = 8'b00101110;
  parameter OP2_RSHF = 8'b00110000;
  parameter OP2_LSHF = 8'b00110001;
  
  parameter HEXBITS  = 24;
  parameter LEDRBITS = 10;
  parameter KEYBITS = 4;
 
  //*** PLL ***//
  // The reset signal comes from the reset button on the DE0-CV board
  // RESET_N is active-low, so we flip its value ("reset" is active-high)
  // The PLL is wired to produce clk and locked signals for our logic
  wire clk;
  wire locked;
  wire reset;

  
  Pll myPll(
    .refclk	(CLOCK_50),
    .rst     	(!RESET_N),
    .outclk_0 	(clk),
    .locked   	(locked)
  );
  
  assign reset = !locked;


  //*** FETCH STAGE ***//
  // The PC register and update logic
  wire [DBITS-1:0] pcplus_FE;
  wire [DBITS-1:0] pcpred_FE;
  wire [DBITS-1:0] inst_FE_w;
  wire stall_pipe;
  wire mispred_EX_w;
  
  reg [DBITS-1:0] pcgood_EX;
  reg [DBITS-1:0] PC_FE;
  reg [INSTBITS-1:0] inst_FE;
  // I-MEM
  (* ram_init_file = IMEMINITFILE *)
  reg [DBITS-1:0] imem [IMEMWORDS-1:0];
  reg mispred_EX;
  
  // This statement is used to initialize the I-MEM
  // during simulation using Model-Sim
  //initial begin
  //  $readmemh("test.hex", imem);
  //end
    
  assign inst_FE_w = imem[PC_FE[IMEMADDRBITS-1:IMEMWORDBITS]];
  
  always @ (posedge clk or posedge reset) begin
    if(reset)
      PC_FE <= STARTPC;
    else if(mispred_EX)
      PC_FE <= pcgood_EX;
    else if(!stall_pipe)
      PC_FE <= pcpred_FE;
    else
      PC_FE <= PC_FE + INSTSIZE;
  end

  // This is the value of "incremented PC", computed in the FE stage
  assign pcplus_FE = PC_FE + INSTSIZE;
  // This is the predicted value of the PC that we use to fetch the next instruction
  assign pcpred_FE = pcplus_FE;

  // FE_latch
  always @ (posedge clk or posedge reset) begin
    if(reset)
      inst_FE <= {INSTBITS{1'b0}};
    //else
	   // TODO: Specify inst_FE considering misprediction and stall
		// ...
  end


  //*** DECODE STAGE ***//
  wire [OP1BITS-1:0] op1_ID_w;
  wire [OP2BITS-1:0] op2_ID_w;
  wire [IMMBITS-1:0] imm_ID_w;
  wire [REGNOBITS-1:0] rd_ID_w;
  wire [REGNOBITS-1:0] rs_ID_w;
  wire [REGNOBITS-1:0] rt_ID_w;
  // Two read ports, always using rs and rt for register numbers
  wire [DBITS-1:0] regval1_ID_w;
  wire [DBITS-1:0] regval2_ID_w;
  wire [DBITS-1:0] sxt_imm_ID_w;
  wire is_br_ID_w;
  wire is_jmp_ID_w;
  wire rd_mem_ID_w;
  wire wr_mem_ID_w;
  wire wr_reg_ID_w;
  wire [4:0] ctrlsig_ID_w;
  wire [REGNOBITS-1:0] wregno_ID_w;
  wire wr_reg_EX_w;
  wire wr_reg_MEM_w;
  
  //control signal wires
  wire [1:0] alu_src_ID_w;
  wire [1:0] new_pc_src_ID_w;
  wire [0:0] mem_we_ID_w;
  wire [0:0] mem_re_ID_w;
  wire [0:0] reg_we_ID_w;
  wire [1:0] reg_wr_src_sel_ID_w;
  wire [0:0] reg_wr_dst_sel_ID_w; 
  
  // Register file
  reg [DBITS-1:0] PC_ID;
  reg [DBITS-1:0] regs [REGWORDS-1:0];
  reg signed [DBITS-1:0] regval1_ID;
  reg signed [DBITS-1:0] regval2_ID;
  reg signed [DBITS-1:0] immval_ID;
  reg [OP1BITS-1:0] op1_ID;
  reg [OP2BITS-1:0] op2_ID;
  reg [4:0] ctrlsig_ID;
  reg [REGNOBITS-1:0] wregno_ID;
  // Declared here for stall check
  reg [REGNOBITS-1:0] wregno_EX;
  reg [REGNOBITS-1:0] wregno_MEM;
  reg [INSTBITS-1:0] inst_ID;
  
  //Buffer Registers
  reg [DBITS-1:0] sxt_imm_ID;
  reg [REGNOBITS-1:0] rt_spec_ID;
  reg [REGNOBITS-1:0] rd_spec_ID; //RdSpec
  //Control signals
  reg [1:0] alu_src_ID; //ALUSrc (2 bits)
  reg [1:0] new_pc_src_ID; //NewPCSrc (2 bits)
  reg [0:0] mem_we_ID; //MemWE (1 bit)
  reg [0:0] mem_re_ID; //MemRE (1 bit)
  reg [0:0] reg_we_ID; //RegWE (1 bit)
  reg [1:0] reg_wr_src_sel_ID; //RegWrSrcSel (2 bits)
  reg [0:0] reg_wr_dst_sel_ID; //RegWrDstSel (1 bit)
 

  // TODO: Specify signals such as op*_ID_w, imm_ID_w, r*_ID_w
  assign op1_ID_w = inst_FE[31:26];
  assign op2_ID_w = inst_FE[25:18];
  assign rd_ID_w = inst_FE[11:8];
  assign rt_ID_w = inst_FE[3:0];
  assign rs_ID_w = inst_FE[7:4];
  assign imm_ID_w = inst_FE[23:8];

  // Read register values
  assign regval1_ID_w = regs[rs_ID_w];
  assign regval2_ID_w = regs[rt_ID_w];

  // Sign extension
  SXT mysxt (.IN(imm_ID_w), .OUT(sxt_imm_ID_w));

  // TODO: Specify control signals such as is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, etc.
  // You may add or change control signals if needed
  // assign is_br_ID_w = ... ;
  // ...
  
// Set the control signal wires
//	alu_src:
//		2b'00 Rt contents
//		2b'01 sxtImm
//		2b'10	sxtImm x 4
//	new_pc_src:
//		2b'00	PC + 4
//		2b'01	JAL PC
//		2b'10	BR PC
//	mem_we:
//		1b'0	writing to mem NOT enabled
//		1b'1	writing to mem ENABLED
//	mem_re:
//		1b'0	reading from mem NOT enabled
//		1b'1	reading from mem ENABLED
//	reg_we:
//		1b'0	writing to regs NOT enabled
//		1b'1	writing to regs ENABLED
//	reg_wr_src_sel
//		2b'00	PC
//		2b'01	Mem data
//		2b'10	ALU Result
//	reg_wr_dst_sel
//		1b'0	Rt specifier
//		1b'1	Rd specifier
  
  if (op1_ID_w == OP1_ALUR) begin
	//EXT instructions
	//all of the control signals are the same for these types of instructions
		alu_src_ID_w = 2b'00;
		new_pc_src_ID_w = 2b'00;
		mem_we_ID_w = 1b'0;
		mem_re_ID_w = 1b'0;
		reg_we_ID_w = 1b'1;
		reg_wr_src_sel_ID_w = 2b'10;
		reg_wr_dst_sel_ID_w = 1b'1;
  end else begin
	//its a BR, JAL, LW, SW, or ALUI instruction
		case(op1_ID_w)
			OP1_BEQ : begin
				alu_src_ID_w = 2b'00;
				new_pc_src_ID_w = 2b'10;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'0;
				reg_wr_src_sel_ID_w = 2b'00; //don't care, default
				reg_wr_dst_sel_ID_w = 1b'0; //don't care, default
						 end
			OP1_BLT : begin
				alu_src_ID_w = 2b'00;
				new_pc_src_ID_w = 2b'10;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'0;
				reg_wr_src_sel_ID_w = 2b'00; //don't care, default
				reg_wr_dst_sel_ID_w = 1b'0; //don't care, default
						 end
			OP1_BLE : begin
				alu_src_ID_w = 2b'00;
				new_pc_src_ID_w = 2b'10;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'0;
				reg_wr_src_sel_ID_w = 2b'00; //don't care, default
				reg_wr_dst_sel_ID_w = 1b'0; //don't care, default
						 end
			OP1_BNE : begin
				alu_src_ID_w = 2b'00;
				new_pc_src_ID_w = 2b'10;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'0;
				reg_wr_src_sel_ID_w = 2b'00; //don't care, default
				reg_wr_dst_sel_ID_w = 1b'0; //don't care, default
						 end
			OP1_JAL : begin
				alu_src_ID_w = 2b'10;
				new_pc_src_ID_w = 2b'01;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'1;
				reg_wr_src_sel_ID_w = 2b'00;
				reg_wr_dst_sel_ID_w = 1b'0;
						 end
			OP1_LW  : begin
				alu_src_ID_w = 2b'01;
				new_pc_src_ID_w = 2b'00;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'1;
				reg_we_ID_w = 1b'1;
				reg_wr_src_sel_ID_w = 2b'01;
				reg_wr_dst_sel_ID_w = 1b'0;
						 end
			OP1_SW  : begin
				alu_src_ID_w = 2b'01;
				new_pc_src_ID_w = 2b'00;
				mem_we_ID_w = 1b'1;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'0;
				reg_wr_src_sel_ID_w = 2b'00; //don't care, default
				reg_wr_dst_sel_ID_w = 1b'0; //don't care, default
						 end
			OP1_ADDI: begin
				alu_src_ID_w = 2b'01;
				new_pc_src_ID_w = 2b'00;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'1;
				reg_wr_src_sel_ID_w = 2b'10;
				reg_wr_dst_sel_ID_w = 1b'0;
						 end
			OP1_ANDI: begin
				alu_src_ID_w = 2b'01;
				new_pc_src_ID_w = 2b'00;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'1;
				reg_wr_src_sel_ID_w = 2b'10;
				reg_wr_dst_sel_ID_w = 1b'0;
						 end
			OP1_ORI : begin
				alu_src_ID_w = 2b'01;
				new_pc_src_ID_w = 2b'00;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'1;
				reg_wr_src_sel_ID_w = 2b'10;
				reg_wr_dst_sel_ID_w = 1b'0;
						 end
			OP1_XORI: begin
				alu_src_ID_w = 2b'01;
				new_pc_src_ID_w = 2b'00;
				mem_we_ID_w = 1b'0;
				mem_re_ID_w = 1b'0;
				reg_we_ID_w = 1b'1;
				reg_wr_src_sel_ID_w = 2b'10;
				reg_wr_dst_sel_ID_w = 1b'0;
						 end		 		 
			//default
		endcase		
  end
  
  assign ctrlsig_ID_w = {is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};
  
  // TODO: Specify stall condition
  // assign stall_pipe = ... ;

  // ID_latch
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
      PC_ID	 <= {DBITS{1'b0}};
		inst_ID	 <= {INSTBITS{1'b0}};
      op1_ID	 <= {OP1BITS{1'b0}};
      op2_ID	 <= {OP2BITS{1'b0}};
      regval1_ID  <= {DBITS{1'b0}};
      regval2_ID  <= {DBITS{1'b0}};
      wregno_ID	 <= {REGNOBITS{1'b0}};
      ctrlsig_ID <= 5'h0;
    end else begin
	   // TODO: Specify ID latches
      PC_ID	 <= PC_FE; //PC
		rt_spec_ID <= rt_ID_w; //RtSpec
		regval2_ID <= regval2_ID_w; //RtCont
		regval1_ID <= ; regval1_ID_w//RsCont
		sxt_imm_ID <= sxt_imm_ID_w; //sxtImm
		
		rd_spec_ID <= rd_spec_ID_w; //RdSpec
		alu_src_ID <= alu_src_ID_w; //ALUSrc
		new_pc_src_ID <= new_pc_src_ID_w; //NewPCSrc
		mem_we_ID <= mem_we_ID_w; //MemWE
		mem_re_ID <= mem_re_ID_w; //MemRE
		reg_we_ID <= reg_we_ID_w; //RegWE
		reg_wr_src_sel_ID <= reg_wr_src_sel_ID_w; //RegWrSrcSel
		reg_wr_dst_sel_ID <= reg_wr_dst_sel_ID_w; //RegWrDstSel
		
		//these are in place of ALUOp
      op1_ID	 <= op1_ID_w;
      op2_ID	 <= op2_ID_w;

		//stall signal

    end
  end


  //*** AGEN/EXEC STAGE ***//

  wire is_br_EX_w;
  wire is_jmp_EX_w;
  wire [DBITS-1:0] pcgood_EX_w;

  reg [INSTBITS-1:0] inst_EX; /* This is for debugging */
  reg br_cond_EX;
  reg [2:0] ctrlsig_EX;
  // Note that aluout_EX_r is declared as reg, but it is output signal from combinational logic
  reg signed [DBITS-1:0] aluout_EX_r;
  reg [DBITS-1:0] aluout_EX;
  reg [DBITS-1:0] regval2_EX;

  always @ (op1_ID or regval1_ID or regval2_ID) begin
    case (op1_ID)
      OP1_BEQ : br_cond_EX = (regval1_ID == regval2_ID);
      OP1_BLT : br_cond_EX = (regval1_ID < regval2_ID);
      OP1_BLE : br_cond_EX = (regval1_ID <= regval2_ID);
      OP1_BNE : br_cond_EX = (regval1_ID != regval2_ID);
      default : br_cond_EX = 1'b0;
    endcase
  end

  always @ (op1_ID or op2_ID or regval1_ID or regval2_ID or immval_ID) begin
    if(op1_ID == OP1_ALUR)
	 //these are the EXT instructions
      case (op2_ID)
			// TODO: complete OP2_*...
			OP2_EQ	 : aluout_EX_r = {31'b0, regval1_ID == regval2_ID};
			OP2_LT	 : aluout_EX_r = {31'b0, regval1_ID < regval2_ID};
			OP2_LE	 : aluout_EX_r = {31'b0, regval1_ID <= regval2_ID};
			OP2_NE	 : aluout_EX_r = {31'b0, regval1_ID != regval2_ID};
			
//			OP2_ADD	 : aluout_EX_r = {31'b0, regval1_ID + regval2_ID};
//			OP2_AND	 : aluout_EX_r = {31'b0, regval1_ID and regval2_ID};
//			OP2_OR	 : aluout_EX_r = {31'b0, regval1_ID or regval2_ID};
//			OP2_XOR	 : aluout_EX_r = {31'b0, regval1_ID xor regval2_ID};
//			OP2_SUB	 : aluout_EX_r = {31'b0, regval1_ID - regval2_ID};
//			OP2_NAND	 : aluout_EX_r = {31'b0, regval1_ID nand regval2_ID};
//			OP2_NOR	 : aluout_EX_r = {31'b0, regval1_ID nor regval2_ID};
//			OP2_NXOR	 : aluout_EX_r = {31'b0, regval1_ID xnor regval2_ID};
//			OP2_RSHF	 : aluout_EX_r = {31'b0, regval1_ID sra regval2_ID};
//			OP2_LSHF	 : aluout_EX_r = {31'b0, regval1_ID sla regval2_ID};
			
			default	 : aluout_EX_r = {DBITS{1'b0}};
      endcase
    else if(op1_ID == OP1_LW || op1_ID == OP1_SW || op1_ID == OP1_ADDI)
      aluout_EX_r = regval1_ID + immval_ID;
    else if(op1_ID == OP1_ANDI)
      aluout_EX_r = regval1_ID & immval_ID;
    else if(op1_ID == OP1_ORI)
      aluout_EX_r = regval1_ID | immval_ID;
    else if(op1_ID == OP1_XORI)
      aluout_EX_r = regval1_ID ^ immval_ID;
    else
      aluout_EX_r = {DBITS{1'b0}};
  end

  assign is_br_EX_w = ctrlsig_ID[4];
  assign is_jmp_EX_w = ctrlsig_ID[3];
  assign wr_reg_EX_w = ctrlsig_ID[0];
  
  // TODO: Specify signals such as mispred_EX_w, pcgood_EX_w
  // assign mispred_EX_w = ... ;
  // assign pcgood_EX_w = ... ;

  // EX_latch
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
	   inst_EX	 <= {INSTBITS{1'b0}};
      aluout_EX	 <= {DBITS{1'b0}};
      wregno_EX	 <= {REGNOBITS{1'b0}};
      ctrlsig_EX <= 3'h0;
      mispred_EX <= 1'b0;
		pcgood_EX  <= {DBITS{1'b0}};
		regval2_EX	<= {DBITS{1'b0}};
    end else begin
		// TODO: Specify EX latches
		// ...
    end
  end
  

  //*** MEM STAGE ***//

  wire rd_mem_MEM_w;
  wire wr_mem_MEM_w;
  
  wire [DBITS-1:0] memaddr_MEM_w;
  wire [DBITS-1:0] rd_val_MEM_w;

  reg [INSTBITS-1:0] inst_MEM; /* This is for debugging */
  reg [DBITS-1:0] regval_MEM;  
  reg ctrlsig_MEM;
  // D-MEM
  (* ram_init_file = IMEMINITFILE *)
  reg [DBITS-1:0] dmem[DMEMWORDS-1:0];

  assign memaddr_MEM_w = aluout_EX;
  assign rd_mem_MEM_w = ctrlsig_EX[2];
  assign wr_mem_MEM_w = ctrlsig_EX[1];
  assign wr_reg_MEM_w = ctrlsig_EX[0];
  // Read from D-MEM
  assign rd_val_MEM_w = (memaddr_MEM_w == ADDRKEY) ? {{(DBITS-KEYBITS){1'b0}}, ~KEY} :
									dmem[memaddr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]];

  // Write to D-MEM
  always @ (posedge clk) begin
    if(wr_mem_MEM_w)
      dmem[memaddr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]] <= regval2_EX;
  end

  always @ (posedge clk or posedge reset) begin
    if(reset) begin
	   inst_MEM		<= {INSTBITS{1'b0}};
      regval_MEM  <= {DBITS{1'b0}};
      wregno_MEM  <= {REGNOBITS{1'b0}};
      ctrlsig_MEM <= 1'b0;
    end else begin
		inst_MEM		<= inst_EX;
      regval_MEM  <= rd_mem_MEM_w ? rd_val_MEM_w : aluout_EX;
      wregno_MEM  <= wregno_EX;
      ctrlsig_MEM <= ctrlsig_EX[0];
    end
  end


  /*** WRITE BACK STAGE ***/ 

  wire wr_reg_WB_w; 
  // regs is already declared in the ID stage

  assign wr_reg_WB_w = ctrlsig_MEM;
  
  always @ (negedge clk or posedge reset) begin
    if(reset) begin
		regs[0] <= {DBITS{1'b0}};
		regs[1] <= {DBITS{1'b0}};
		regs[2] <= {DBITS{1'b0}};
		regs[3] <= {DBITS{1'b0}};
		regs[4] <= {DBITS{1'b0}};
		regs[5] <= {DBITS{1'b0}};
		regs[6] <= {DBITS{1'b0}};
		regs[7] <= {DBITS{1'b0}};
		regs[8] <= {DBITS{1'b0}};
		regs[9] <= {DBITS{1'b0}};
		regs[10] <= {DBITS{1'b0}};
		regs[11] <= {DBITS{1'b0}};
		regs[12] <= {DBITS{1'b0}};
		regs[13] <= {DBITS{1'b0}};
		regs[14] <= {DBITS{1'b0}};
		regs[15] <= {DBITS{1'b0}};
	 end else if(wr_reg_WB_w) begin
      regs[wregno_MEM] <= regval_MEM;
	 end
  end
  
  
  /*** I/O ***/
  // Create and connect HEX register
  reg [23:0] HEX_out;
 
  SevenSeg ss5(.OUT(HEX5), .IN(HEX_out[23:20]), .OFF(1'b0));
  SevenSeg ss4(.OUT(HEX4), .IN(HEX_out[19:16]), .OFF(1'b0));
  SevenSeg ss3(.OUT(HEX3), .IN(HEX_out[15:12]), .OFF(1'b0));
  SevenSeg ss2(.OUT(HEX2), .IN(HEX_out[11:8]), .OFF(1'b0));
  SevenSeg ss1(.OUT(HEX1), .IN(HEX_out[7:4]), .OFF(1'b0));
  SevenSeg ss0(.OUT(HEX0), .IN(HEX_out[3:0]), .OFF(1'b0));
  
  always @ (posedge clk or posedge reset) begin
    if(reset)
	   HEX_out <= 24'hFEDEAD;
	 else if(wr_mem_MEM_w && (memaddr_MEM_w == ADDRHEX))
      HEX_out <= regval2_EX[HEXBITS-1:0];
  end

  // TODO: Write the code for LEDR here

  reg [9:0] LEDR_out;
  
  // ...

  assign LEDR = LEDR_out;
  
endmodule


module SXT(IN, OUT);
  parameter IBITS = 16;
  parameter OBITS = 32;

  input  [IBITS-1:0] IN;
  output [OBITS-1:0] OUT;

  assign OUT = {{(OBITS-IBITS){IN[IBITS-1]}}, IN};
endmodule

