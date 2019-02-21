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
  
  // Constants relevant to FETCH stage
  parameter take_pcplus = 2'b00;
  parameter take_jalpc = 2'b01;
  parameter take_brpc = 2'b11;
  
  assign stall_pipe = 1'b1;
  
  // Display part of PC on sevenseg
  reg[31:0] counter;
  `define ONE_SECOND							32'd50000000
  
  always @ (posedge clk or posedge reset) begin
    if (reset)
	   counter <= 32'b0;
	 else
	   if (counter >= `ONE_SECOND)
		  begin
          HEX_out[3:0] = PC_FE[3:0];
	       HEX_out[7:4] = PC_FE[7:4];
	       HEX_out[11:8] = PC_FE[11:8];
	       HEX_out[15:12] = PC_FE[15:12];
	       HEX_out[19:16] = PC_FE[19:16];
	       HEX_out[23:20] = PC_FE[23:20];
			 counter <= 32'b0;
		  end
		else
		  counter <= counter + 1;
  end
  
  // This statement is used to initialize the I-MEM
  // during simulation using Model-Sim
  //initial begin
  //  $readmemh("test.hex", imem);
  //end
    
  assign inst_FE_w = imem[PC_FE[IMEMADDRBITS-1:IMEMWORDBITS]];
  
  // Select a PC value
  always @ (posedge clk or posedge reset) begin
    if(reset)
	   PC_FE <= STARTPC;            // Set PC to initial value after a reset
 //   else if(mispred_EX)
 //     PC_FE <= pcgood_EX;          // Used to set PC in case of branch misprediction
    else if(!stall_pipe)
      PC_FE <= pcpred_FE;          // If stalling, ???
    else
	   case(new_pc_src_EX_r & branch_logic_out)
		  take_pcplus: PC_FE <= PC_FE + INSTSIZE;   // Take PC + 4 as normal
		  take_jalpc:  PC_FE <= aluout_EX_r;        // Take ALU result as new PC for JAL
		  take_brpc:   PC_FE <= sxt_addr_out_EX_r;       // Take PC + sxtImm from EX stage
      endcase
  end

  // This is the value of "incremented PC", computed in the FE stage
  assign pcplus_FE = PC_FE + INSTSIZE;
  // This is the predicted value of the PC that we use to fetch the next instruction
  assign pcpred_FE = pcplus_FE;

  // FE_latch
  always @ (posedge clk or posedge reset) begin
    if(reset)
      inst_FE <= {INSTBITS{1'b0}};
    else if(stall_pipe)        // Only change the register contents if stall signal is 1 (= not stalling)
      inst_FE <= inst_FE_w;    // Don't need to worry about branch prediction (yet)
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
  
  // Control signal generator
  CONTROL_SIGNAL_GENERATOR control_signal_generator_inst(
    .OPCODE1_IN(op1_ID_w),
	 .CLOCK(clk),
	 .ALUSRC_OUT(alu_src_ID_w),
	 .NEWPCSRC_OUT(new_pc_src_ID_w),
	 .MEMWE_OUT(mem_we_ID_w),
	 .MEMRE_OUT(mem_re_ID_w),
	 .REGWE_OUT(reg_we_ID_w),
	 .REGWRSRCSEL_OUT(reg_wr_src_sel_ID_w),
	 .REGWRDSTSEL_OUT(reg_wr_dst_sel_ID_w)
  );

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
		regval1_ID <= regval1_ID_w; //RsCont
		sxt_imm_ID <= sxt_imm_ID_w; //sxtImm
		
		rd_spec_ID <= rd_ID_w; //RdSpec
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
  
  // Constants relevant to EXECUTE stage
  parameter take_rtspec = 1'b0;
  parameter take_rdspec = 1'b1;
  parameter take_rtcont = 2'b00;
  parameter take_sxtimm = 2'b01;
  parameter take_sxtimm_4 = 2'b10;

  wire is_br_EX_w;
  wire is_jmp_EX_w;
  wire [DBITS-1:0] pcgood_EX_w;

  reg [INSTBITS-1:0] inst_EX; /* This is for debugging */
  reg br_cond_EX;
  reg [2:0] ctrlsig_EX;
  // Note that aluout_EX_r is declared as reg, but it is output signal from combi logic
  reg signed [DBITS-1:0] aluout_EX_r;
  reg [DBITS-1:0] aluout_EX;
  reg [DBITS-1:0] regval2_EX; //RtCont
    
   //my wires and registers
	reg [DBITS-1:0] alu_in_EX_r; //ALU input (from mux)
	reg [REGNOBITS-1:0] dst_reg_EX_r; //DstReg
	reg [DBITS-1:0] sxt_imm_4_r; //sxtImm x 4
	
	reg [DBITS-1:0] new_pc_src_EX_r;
	reg [DBITS-1:0] sxt_addr_out_EX_r;
	
	reg [DBITS-1:0] PC_EX; //PC
	reg [REGNOBITS-1:0] dst_reg_EX; //DstReg
	reg [0:0] mem_we_EX; //MemWE (1 bit)
	reg [0:0] mem_re_EX; //MemRE (1 bit)
	reg [0:0] reg_we_EX; //RegWE (1 bit)
	reg [1:0] reg_wr_src_sel_EX; //RegWrSrcSel (2 bits)
	
	always @ (*) begin
		new_pc_src_EX_r = new_pc_src_ID;
	end
	
	always @ (*) begin
		//shift left 2 bits with 0s
		sxt_imm_4_r = sxt_imm_ID << 2;
	
		//set dst_reg_EX_r
		if (reg_wr_dst_sel_ID == take_rtspec)
			dst_reg_EX_r = rt_spec_ID; //RtSpec		
		else if (reg_wr_dst_sel_ID == take_rdspec)
			dst_reg_EX_r = rd_spec_ID; //RdSpec
			
		//set alu's 2nd input (alu_in_EX_r)
		if (alu_src_ID == take_rtcont)
			alu_in_EX_r = regval2_ID; //take RtCont
		else if (alu_src_ID == take_sxtimm)
			alu_in_EX_r = sxt_imm_ID; //take sxtImm
		else if (alu_src_ID == take_sxtimm_4)
			alu_in_EX_r = sxt_imm_4_r; //take sxtImm x 4
			
		//set PC increment
		sxt_addr_out_EX_r = (PC_ID + sxt_imm_4_r); 
			
	end	

  always @ (op1_ID or op2_ID or regval1_ID or alu_in_EX_r) begin
	 case (op1_ID)
      OP1_BEQ : aluout_EX_r = {31'b0, regval1_ID == alu_in_EX_r};
      OP1_BLT : aluout_EX_r = {31'b0, regval1_ID < alu_in_EX_r};
      OP1_BLE : aluout_EX_r = {31'b0, regval1_ID <= alu_in_EX_r};
      OP1_BNE : aluout_EX_r = {31'b0, regval1_ID != alu_in_EX_r};
      default : aluout_EX_r = {DBITS{1'b0}};
    endcase
    if(op1_ID == OP1_ALUR)
      case (op2_ID)
			OP2_EQ	 : aluout_EX_r = {31'b0, regval1_ID == alu_in_EX_r};
			OP2_LT	 : aluout_EX_r = {31'b0, regval1_ID < alu_in_EX_r};
			OP2_LE	 : aluout_EX_r = {31'b0, regval1_ID <= alu_in_EX_r};
			OP2_NE	 : aluout_EX_r = {31'b0, regval1_ID != alu_in_EX_r};

			OP2_ADD	 : aluout_EX_r = (regval1_ID + alu_in_EX_r);
			OP2_AND	 : aluout_EX_r = (regval1_ID & alu_in_EX_r);
			OP2_OR	 : aluout_EX_r = (regval1_ID | alu_in_EX_r);
			OP2_XOR	 : aluout_EX_r = (regval1_ID ^ alu_in_EX_r); //xor
			OP2_SUB	 : aluout_EX_r = (regval1_ID - alu_in_EX_r);
			//OP2_NAND	 : aluout_EX_r = {31'b0, regval1_ID ~& alu_in_EX_r}; //nand
			//OP2_NOR	 : aluout_EX_r = {31'b0, regval1_ID ~| alu_in_EX_r}; //nor
			OP2_NXOR	 : aluout_EX_r = (regval1_ID ~^ alu_in_EX_r); //xnor
			OP2_RSHF	 : aluout_EX_r = (regval1_ID >>> alu_in_EX_r); //arithmetic shift
			OP2_LSHF	 : aluout_EX_r = (regval1_ID <<< alu_in_EX_r); //arithmetic shift
	default	 : aluout_EX_r = {DBITS{1'b0}};
      endcase
    else if(op1_ID == OP1_LW || op1_ID == OP1_SW || op1_ID == OP1_ADDI || op1_ID == OP1_JAL)
      aluout_EX_r = regval1_ID + alu_in_EX_r;
    else if(op1_ID == OP1_ANDI)
      aluout_EX_r = regval1_ID & alu_in_EX_r;
    else if(op1_ID == OP1_ORI)
      aluout_EX_r = regval1_ID | alu_in_EX_r;
    else if(op1_ID == OP1_XORI)
      aluout_EX_r = regval1_ID ^ alu_in_EX_r;
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
		PC_EX <= PC_ID; //PC
		regval2_EX <= regval2_ID; //RtCont
		aluout_EX <= aluout_EX_r; //ALUResult
		dst_reg_EX <= dst_reg_EX_r; //DstReg
		mem_we_EX <= mem_we_ID; //MemWE
		mem_re_EX <= mem_re_ID; //MemRE
		reg_we_EX <= reg_we_ID; //RegWE
		reg_wr_src_sel_EX <= reg_wr_src_sel_ID; //RegWrSrcSel
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
  
  
 
  /*** Branch Handling Logic ***/
  reg[1:0] branch_logic_out;
  reg[0:0] aluout_EX_r_1bit;
  reg[0:0] take_branch; //0 = don't take branch, 1 = DO take branch
  reg[1:0] take_branch_sxt;
  reg[0:0] is_jal; //0 = not jal, 1 = is jal
  reg[0:0] flush_logic_out;
  
  
  //if opcode is a branch or JAL
  //do something special
  always @ (*) begin
		aluout_EX_r_1bit = aluout_EX_r[0:0];
		//if the instruction is a branch & if the branch condition is true or not
		take_branch = (op1_ID == OP1_BEQ | op1_ID == OP1_BLT | op1_ID == OP1_BLE | op1_ID == OP1_BNE) & aluout_EX_r_1bit;	
		
		//Sign extend take_branch to be 2 bits
		take_branch_sxt = {take_branch, take_branch};
		
		//is the instruction a jal?
		is_jal = (op1_ID == OP1_JAL);
		
		//if the instruction is a JAL or we are taking the branch, we need to flush
		flush_logic_out = (is_jal | take_branch);
		
		//Sign extend inverse of is_jal to be 2 bits: {~is_jal, ~is_jal}
		//or it with take_branch_sxt
		branch_logic_out = ({~is_jal, ~is_jal} | take_branch_sxt);
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
  
//  always @ (posedge clk or posedge reset) begin
//    if(reset)
//	   HEX_out <= 24'hFEDEAD;
//	 else if(wr_mem_MEM_w && (memaddr_MEM_w == ADDRHEX))
//      HEX_out <= regval2_EX[HEXBITS-1:0];
//  end

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

module CONTROL_SIGNAL_GENERATOR(
  input        [5:0] OPCODE1_IN,
  input              CLOCK,
  output reg   [1:0] ALUSRC_OUT,
  output reg   [1:0] NEWPCSRC_OUT,
  output reg         MEMWE_OUT,
  output reg         MEMRE_OUT,
  output reg         REGWE_OUT,
  output reg   [1:0] REGWRSRCSEL_OUT,
  output reg         REGWRDSTSEL_OUT
);

always @ (*) begin
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
  
  case (OPCODE1_IN)
    //EXT instructions
    //all of the control signals are the same for these types of instructions
    OP1_ALUR : begin
		ALUSRC_OUT = 2'b00;
		NEWPCSRC_OUT = 2'b00;
		MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b1;
		REGWRSRCSEL_OUT = 2'b10;
		REGWRDSTSEL_OUT = 1'b1;
	 end
		
	 OP1_BEQ : begin
		ALUSRC_OUT = 2'b00;
		NEWPCSRC_OUT = 2'b10;
		MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b0;
		REGWRSRCSEL_OUT = 2'b00; //don't care, default
		REGWRDSTSEL_OUT = 1'b0; //don't care, default
	 end
	 
    OP1_BLT : begin
		ALUSRC_OUT = 2'b00;
		NEWPCSRC_OUT = 2'b10;
		MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b0;
		REGWRSRCSEL_OUT = 2'b00; //don't care, default
		REGWRDSTSEL_OUT = 1'b0; //don't care, default
	 end
		
	 OP1_BLE : begin
		ALUSRC_OUT = 2'b00;
		NEWPCSRC_OUT = 2'b10;
		MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b0;
		REGWRSRCSEL_OUT = 2'b00; //don't care, default
		REGWRDSTSEL_OUT = 1'b0; //don't care, default
	 end
	 
	 OP1_BNE : begin
	   ALUSRC_OUT = 2'b00;
		NEWPCSRC_OUT = 2'b10;
		MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b0;
		REGWRSRCSEL_OUT = 2'b00; //don't care, default
		REGWRDSTSEL_OUT = 1'b0; //don't care, default
	 end
	 
    OP1_JAL : begin
		ALUSRC_OUT = 2'b10;
		NEWPCSRC_OUT = 2'b01;
		MEMWE_OUT = 1'b0;
	   MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b1;
		REGWRSRCSEL_OUT = 2'b00;
		REGWRDSTSEL_OUT = 1'b0;
	 end
	 
	 OP1_LW  : begin
		ALUSRC_OUT = 2'b01;
		NEWPCSRC_OUT = 2'b00;
	   MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b1;
		REGWE_OUT = 1'b1;
		REGWRSRCSEL_OUT = 2'b01;
		REGWRDSTSEL_OUT = 1'b0;
	 end
	 
	 OP1_SW  : begin
		ALUSRC_OUT = 2'b01;
		NEWPCSRC_OUT = 2'b00;
		MEMWE_OUT = 1'b1;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b0;
		REGWRSRCSEL_OUT = 2'b00; //don't care, default
		REGWRDSTSEL_OUT = 1'b0; //don't care, default
	 end
	 
	 OP1_ADDI: begin
		ALUSRC_OUT = 2'b01;
		NEWPCSRC_OUT = 2'b00;
		MEMWE_OUT = 1'b0;
	   MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b1;
		REGWRSRCSEL_OUT = 2'b10;
		REGWRDSTSEL_OUT = 1'b0;
	 end
	 
	 OP1_ANDI: begin
		ALUSRC_OUT = 2'b01;
		NEWPCSRC_OUT = 2'b00;
		MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b1;
		REGWRSRCSEL_OUT = 2'b10;
	   REGWRDSTSEL_OUT = 1'b0;
	 end
	 
	 OP1_ORI : begin
		ALUSRC_OUT = 2'b01;
		NEWPCSRC_OUT = 2'b00;
		MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b1;
		REGWRSRCSEL_OUT = 2'b10;
		REGWRDSTSEL_OUT = 1'b0;
	 end
	 
	 OP1_XORI: begin
		ALUSRC_OUT = 2'b01;
		NEWPCSRC_OUT = 2'b00;
	   MEMWE_OUT = 1'b0;
		MEMRE_OUT = 1'b0;
		REGWE_OUT = 1'b1;
		REGWRSRCSEL_OUT = 2'b10;
	   REGWRDSTSEL_OUT = 1'b0;
	 end	
  endcase		
end
endmodule
