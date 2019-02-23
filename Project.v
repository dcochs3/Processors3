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

  parameter DBITS     = 32;
  parameter INSTSIZE  = 32'd4;
  parameter INSTBITS  = 32;
  parameter REGNOBITS = 4;
  parameter REGWORDS  = (1 << REGNOBITS);
  parameter IMMBITS   = 16;
  parameter STARTPC   = 32'h100;
  parameter ADDRHEX   = 32'hFFFFF000;
  parameter ADDRLEDR  = 32'hFFFFF020;
  parameter ADDRKEY   = 32'hFFFFF080;
  parameter ADDRSW    = 32'hFFFFF090;
 
  // Change this to fmedian2.mif before submitting
  parameter IMEMINITFILE = "Test.mif";
  //parameter IMEMINITFILE = "fmedian2.mif";
  
  parameter IMEMADDRBITS = 16;
  parameter IMEMWORDBITS = 2;
  parameter IMEMWORDS    = (1 << (IMEMADDRBITS - IMEMWORDBITS));
  parameter DMEMADDRBITS = 16;
  parameter DMEMWORDBITS = 2;
  parameter DMEMWORDS    = (1 << (DMEMADDRBITS - DMEMWORDBITS));
   
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
  parameter KEYBITS  = 4;
  
  reg [23:0] HEX_out;
  wire [0:0] branch_or_jal_stall;
  wire stall_logic_out;
 
   // Note that aluout_EX_r is declared as reg, but it is output signal from combi logic
  reg signed [DBITS-1:0] aluout_EX_r;
  
  reg [DBITS-1:0]        new_pc_src_EX_r;
  reg [DBITS-1:0]        sxt_addr_out_EX_r;
  reg[1:0] branch_logic_out;
  reg[0:0] flush_logic_out;
 
 
  //*** PLL ***//
  // The reset signal comes from the reset button on the DE0-CV board
  // RESET_N is active-low, so we flip its value ("reset" is active-high)
  // The PLL is wired to produce clk and locked signals for our logic
  wire clk;
  wire locked;
  wire reset;

  
  Pll myPll(
    .refclk    (CLOCK_50),
    .rst       (!RESET_N),
    .outclk_0  (clk),
    .locked    (locked)
  );
  
  assign reset = !locked;


  //*** FETCH STAGE ***//
  
  // Wires
  wire [DBITS-1:0]   inst_FE_w;
  wire               stall_pipe;
  
  
  // Registers
  reg [DBITS-1:0]    PC_FE;       // The PC that goes in the FETCH buffer
  reg [INSTBITS-1:0] inst_FE;
  reg                waiting_to_branch;
  reg                done_branching;
  reg [DBITS-1:0]    branch_pc_from_stall;
  
  // I-MEM
  (* ram_init_file = IMEMINITFILE *)
  reg [DBITS-1:0]    imem [IMEMWORDS-1:0];
  
  // This statement is used to initialize the I-MEM
  // during simulation using Model-Sim
  initial begin
    $readmemh("test.hex", imem);
  end
 
  // Constants
  parameter TAKE_INCR_PC = 2'b00;
  parameter TAKE_JAL_PC  = 2'b01;
  parameter TAKE_BR_PC   = 2'b11;
  parameter TAKE_STALL   = 1'b0;
  
  assign stall_pipe = branch_or_jal_stall == 1 && stall_logic_out == 1;
    
  // Assignments to wires
  assign inst_FE_w = imem[PC_FE[IMEMADDRBITS-1:IMEMWORDBITS]];
  
  // Select a PC value
  always @ (posedge clk or posedge reset) begin
    if(reset)
        PC_FE <= STARTPC;            // Set PC to initial value after a reset
    else if (stall_pipe == TAKE_STALL) begin
        //stall the pipeline
        //do not fetch a new PC
        PC_FE <= PC_FE;
    end
    else if (flush_logic_out)       // Does this make sense?
        PC_FE <= STARTPC;
    else
        if (waiting_to_branch == 1) begin
            PC_FE             <= branch_pc_from_stall;
            done_branching    <= 1'b1;
        end
        else begin
            done_branching    <= 1'b0;
            case(new_pc_src_EX_r & branch_logic_out)
                TAKE_INCR_PC: PC_FE <= PC_FE + INSTSIZE;   // Take PC + 4 as normal
                TAKE_JAL_PC:  PC_FE <= aluout_EX_r;        // Take ALU result as new PC for JAL
                TAKE_BR_PC:   PC_FE <= sxt_addr_out_EX_r;  // Take PC + sxtImm from EX stage
            endcase
        end
  end

  // FETCH buffer
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
      inst_FE <= {INSTBITS{1'b0}};
    end
    else if(stall_pipe != TAKE_STALL)   // Only change the register contents if stall signal is 1 (1 means *not* stalling)
      inst_FE <= inst_FE_w;             // Don't need to worry about branch prediction (yet)        
    else if (flush_logic_out) begin
      inst_FE <= {INSTBITS{1'b0}};
    end
  end
  
  

  //*** DECODE STAGE ***//
  
  /* Wires */
  
  // General Wires
  wire [DBITS-1:0]     PC_ID_w;
  wire [OP1BITS-1:0]   op1_ID_w;
  wire [OP2BITS-1:0]   op2_ID_w;
  wire [IMMBITS-1:0]   imm_ID_w;
  wire [REGNOBITS-1:0] rd_ID_w;
  wire [REGNOBITS-1:0] rs_ID_w;
  wire [REGNOBITS-1:0] rt_ID_w;
  wire [DBITS-1:0]     regval1_ID_w;
  wire [DBITS-1:0]     regval2_ID_w;
  wire [DBITS-1:0]     sxt_imm_ID_w;
  wire                 rd_mem_ID_w;
  wire                 wr_mem_ID_w;
  wire                 wr_reg_ID_w;
  wire [4:0]           ctrlsig_ID_w;
  wire                 wr_reg_EX_w;
  wire                 wr_reg_MEM_w;
  
  // Control Signal Wires
  wire [1:0] alu_src_ID_w;
  wire [1:0] new_pc_src_ID_w;
  wire [0:0] mem_we_ID_w;
  wire [0:0] mem_re_ID_w;
  wire [0:0] reg_we_ID_w;
  wire [1:0] reg_wr_src_sel_ID_w;
  wire [0:0] reg_wr_dst_sel_ID_w; 
   

  /* Registers */
  
  // Register File
  reg [DBITS-1:0]        PC_ID;
  reg [DBITS-1:0]        regs [REGWORDS-1:0];
  reg signed [DBITS-1:0] regval1_ID;
  reg signed [DBITS-1:0] regval2_ID;
  reg signed [DBITS-1:0] immval_ID;
  reg [OP1BITS-1:0]      op1_ID;
  reg [OP2BITS-1:0]      op2_ID;
  
  // Buffer Registers
  reg [DBITS-1:0]     sxt_imm_ID;
  reg [REGNOBITS-1:0] rt_spec_ID;
  reg [REGNOBITS-1:0] rd_spec_ID; //RdSpec
  reg                 stall_pipe_ID;
  
  // Control Signals
  reg [1:0] alu_src_ID;        //ALUSrc (2 bits)
  reg [1:0] new_pc_src_ID;     //NewPCSrc (2 bits)
  reg [0:0] mem_we_ID;         //MemWE (1 bit)
  reg [0:0] mem_re_ID;         //MemRE (1 bit)
  reg [0:0] reg_we_ID;         //RegWE (1 bit)
  reg [1:0] reg_wr_src_sel_ID; //RegWrSrcSel (2 bits)
  reg [0:0] reg_wr_dst_sel_ID; //RegWrDstSel (1 bit)
 

  /* Assignments to Wires */
  
  // Data from FETCH Buffer
  assign PC_ID_w  = PC_FE;
  assign op1_ID_w = inst_FE[31:26];
  assign op2_ID_w = inst_FE[25:18];
  assign rd_ID_w  = inst_FE[11:8];
  assign rt_ID_w  = inst_FE[3:0];
  assign rs_ID_w  = inst_FE[7:4];
  assign imm_ID_w = inst_FE[23:8];

  // Read Register Values
  assign regval1_ID_w = regs[rs_ID_w];
  assign regval2_ID_w = regs[rt_ID_w];

  // Sign Extender
  SXT mysxt (.IN(imm_ID_w), .OUT(sxt_imm_ID_w));
  
  // Control Signal Generator
  CONTROL_SIGNAL_GENERATOR control_signal_generator_inst(
    .OPCODE1_IN(op1_ID_w),
    .CLOCK(clk),
    .ALUSRC_OUT(alu_src_ID_w),                //00 = RtCont, 01 = sxtImm, 10 = sxtImm x 4
    .NEWPCSRC_OUT(new_pc_src_ID_w),           //00 = PC + 4, 01 = JAL PC, 10 = BR PC
    .MEMWE_OUT(mem_we_ID_w),                  //0 = writing to mem NOT enabled, 1 = writing to mem ENABLED
    .MEMRE_OUT(mem_re_ID_w),                  //0 = reading from mem NOT enabled, 1 = reading from mem ENABLED
    .REGWE_OUT(reg_we_ID_w),                  //0 = writing to regs NOT enabled, 1 = writing to regs ENABLED
    .REGWRSRCSEL_OUT(reg_wr_src_sel_ID_w),    //00 = PC, 01 = MemData, 10 = ALUResult
    .REGWRDSTSEL_OUT(reg_wr_dst_sel_ID_w)     //0 = RtSpec, 1 = RdSpec
  );
  
  
  // ID Buffer
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
        PC_ID             <= {DBITS{1'b0}};
        rt_spec_ID        <= {REGNOBITS{1'b0}};
        sxt_imm_ID        <= {DBITS{1'b0}};
        rd_spec_ID        <= {REGNOBITS{1'b0}};
        alu_src_ID        <= 2'b00;
        new_pc_src_ID     <= 2'b00;
        mem_we_ID         <= 1'b0;
        mem_re_ID         <= 1'b0;
        reg_we_ID         <= 1'b0;
        op1_ID            <= {OP1BITS{1'b0}};
        op2_ID            <= {OP2BITS{1'b0}};
        regval1_ID        <= {DBITS{1'b0}};
        regval2_ID        <= {DBITS{1'b0}};
        reg_wr_src_sel_ID <= 2'b0;
        reg_wr_dst_sel_ID <= 1'b0;
        stall_pipe_ID     <= 1'b0;
    end
    else if (stall_pipe_ID == 1) begin
        PC_ID             <= {DBITS{1'b0}};
        rt_spec_ID        <= {REGNOBITS{1'b0}};
        sxt_imm_ID        <= {DBITS{1'b0}};
        rd_spec_ID        <= {REGNOBITS{1'b0}};
        alu_src_ID        <= 2'b00;
        new_pc_src_ID     <= 2'b00;
        mem_we_ID         <= 1'b0;
        mem_re_ID         <= 1'b0;
        reg_we_ID         <= 1'b0;
        op1_ID            <= {OP1BITS{1'b0}};
        op2_ID            <= {OP2BITS{1'b0}};
        regval1_ID        <= {DBITS{1'b0}};
        regval2_ID        <= {DBITS{1'b0}};
        reg_wr_src_sel_ID <= 2'b0;
        reg_wr_dst_sel_ID <= 1'b0;
        stall_pipe_ID     <= 1'b0;
    end
    else if (stall_logic_out == TAKE_STALL)
        stall_pipe_ID     <= 1'b1;
    else begin // Only change the register contents if stall signal is 1 (1 means *not* stalling)
        PC_ID             <= PC_ID_w;             //PC
        rt_spec_ID        <= rt_ID_w;             //RtSpec
        regval2_ID        <= regval2_ID_w;        //RtCont
        regval1_ID        <= regval1_ID_w;        //RsCont
        sxt_imm_ID        <= sxt_imm_ID_w;        //sxtImm
        rd_spec_ID        <= rd_ID_w;             //RdSpec
        alu_src_ID        <= alu_src_ID_w;        //ALUSrc
        new_pc_src_ID     <= new_pc_src_ID_w;     //NewPCSrc
        mem_we_ID         <= mem_we_ID_w;         //MemWE
        mem_re_ID         <= mem_re_ID_w;         //MemRE
        reg_we_ID         <= reg_we_ID_w;         //RegWE
        reg_wr_src_sel_ID <= reg_wr_src_sel_ID_w; //RegWrSrcSel
        reg_wr_dst_sel_ID <= reg_wr_dst_sel_ID_w; //RegWrDstSel
        stall_pipe_ID     <= 1'b0;
        
        // These are in place of ALUOp
        op1_ID            <= op1_ID_w;
        op2_ID            <= op2_ID_w; 
    end      
  end


    //*** EX STAGE ***//
  
    // Constants relevant to EXECUTE stage
    parameter take_rtspec   = 1'b0;
    parameter take_rdspec   = 1'b1;
    parameter take_rtcont   = 2'b00;
    parameter take_sxtimm   = 2'b01;
    parameter take_sxtimm_4 = 2'b10;

    /* Registers */
    
    reg br_cond_EX;
    
    reg [DBITS-1:0]        aluout_EX;
    reg [DBITS-1:0]        regval2_EX;        //RtCont
    reg [DBITS-1:0]        alu_in_EX_r;       //ALU input (from mux)
    reg [REGNOBITS-1:0]    dst_reg_EX_r;      //DstReg
    reg [DBITS-1:0]        sxt_imm_4_r;       //sxtImm x 4  
    reg [DBITS-1:0]        PC_EX;             //PC
    reg [REGNOBITS-1:0]    dst_reg_EX;        //DstReg
    reg [0:0]              mem_we_EX;         //MemWE (1 bit)
    reg [0:0]              mem_re_EX;         //MemRE (1 bit)
    reg [0:0]              reg_we_EX;         //RegWE (1 bit)
    reg [1:0]              reg_wr_src_sel_EX; //RegWrSrcSel (2 bits)
    
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
        alu_in_EX_r = regval2_ID;  //take RtCont
      else if (alu_src_ID == take_sxtimm)
        alu_in_EX_r = sxt_imm_ID;  //take sxtImm
      else if (alu_src_ID == take_sxtimm_4)
        alu_in_EX_r = sxt_imm_4_r; //take sxtImm x 4
            
      //set PC increment
      sxt_addr_out_EX_r = (PC_ID + sxt_imm_4_r); 
      
      if (branch_or_jal_stall == 0 && stall_pipe == TAKE_STALL) begin
        waiting_to_branch    = 1'b1;
        branch_pc_from_stall = sxt_addr_out_EX_r;
      end
      else if (done_branching == 1)
        waiting_to_branch    = 1'b0;
            
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
            OP2_EQ     : aluout_EX_r = {31'b0, regval1_ID == alu_in_EX_r};
            OP2_LT     : aluout_EX_r = {31'b0, regval1_ID < alu_in_EX_r};
            OP2_LE     : aluout_EX_r = {31'b0, regval1_ID <= alu_in_EX_r};
            OP2_NE     : aluout_EX_r = {31'b0, regval1_ID != alu_in_EX_r};
            OP2_ADD    : aluout_EX_r = regval1_ID + alu_in_EX_r;
            OP2_AND    : aluout_EX_r = regval1_ID & alu_in_EX_r;
            OP2_OR     : aluout_EX_r = regval1_ID | alu_in_EX_r;
            OP2_XOR    : aluout_EX_r = regval1_ID ^ alu_in_EX_r;
            OP2_SUB    : aluout_EX_r = regval1_ID - alu_in_EX_r;
            OP2_NAND   : aluout_EX_r = (~regval1_ID) | (~alu_in_EX_r);
            OP2_NOR    : aluout_EX_r = (~regval1_ID) & (~alu_in_EX_r);
            OP2_NXOR   : aluout_EX_r = regval1_ID ~^ alu_in_EX_r;
            OP2_RSHF   : aluout_EX_r = regval1_ID >>> alu_in_EX_r;     // Arithmetic Shift
            OP2_LSHF   : aluout_EX_r = regval1_ID <<< alu_in_EX_r;     // Arithmetic Shift
            default    : aluout_EX_r = {DBITS{1'b0}};
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
  
  // EX Buffer
  always @ (posedge clk or posedge reset) begin
    if(reset) begin
        PC_EX             <= {DBITS{1'b0}};
        aluout_EX         <= {DBITS{1'b0}};
        regval2_EX        <= {DBITS{1'b0}};
        dst_reg_EX        <= {REGNOBITS{1'b0}};
        mem_we_EX         <= 1'b0;
        mem_re_EX         <= 1'b0;
        reg_we_EX         <= 1'b0;
        reg_wr_src_sel_EX <= 2'b0;
    end
    else if (stall_pipe_ID == 1) begin
        PC_EX             <= {DBITS{1'b0}};
        aluout_EX         <= {DBITS{1'b0}};
        regval2_EX        <= {DBITS{1'b0}};
        dst_reg_EX        <= {REGNOBITS{1'b0}};
        mem_we_EX         <= 1'b0;
        mem_re_EX         <= 1'b0;
        reg_we_EX         <= 1'b0;
        reg_wr_src_sel_EX <= 2'b0;
    end
    else begin
        PC_EX             <= PC_ID;             //PC
        regval2_EX        <= regval2_ID;        //RtCont
        aluout_EX         <= aluout_EX_r;       //ALUResult
        dst_reg_EX        <= dst_reg_EX_r;      //DstReg
        mem_we_EX         <= mem_we_ID;         //MemWE
        mem_re_EX         <= mem_re_ID;         //MemRE
        reg_we_EX         <= reg_we_ID;         //RegWE
        reg_wr_src_sel_EX <= reg_wr_src_sel_ID; //RegWrSrcSel
    end
  end
  

  //*** MEM STAGE ***//

//  wire rd_mem_MEM_w;
//  wire wr_mem_MEM_w;
  
  wire [DBITS-1:0] PC_MEM_w;
  wire [DBITS-1:0] mem_addr_MEM_w;
  wire [DBITS-1:0] mem_val_out_MEM_w;
  wire mem_we_MEM_w;
  wire mem_re_MEM_w;
  wire [DBITS-1:0] aluout_MEM_w;
  wire [REGNOBITS-1:0] dst_reg_MEM_w;
  wire reg_we_MEM_w;
  wire [1:0] reg_wr_src_sel_MEM_w;

  reg [INSTBITS-1:0] inst_MEM; /* This is for debugging */ 
  reg [DBITS-1:0] PC_MEM;
  reg [DBITS-1:0] mem_val_out_MEM;
  reg [DBITS-1:0] aluout_MEM;
  reg [REGNOBITS-1:0] dst_reg_MEM;
  reg reg_we_MEM;
  reg [1:0] reg_wr_src_sel_MEM;
  
  // D-MEM
  (* ram_init_file = IMEMINITFILE *)
  reg [DBITS-1:0] dmem[DMEMWORDS-1:0];
  
  assign PC_MEM_w = PC_EX;
  
  assign mem_addr_MEM_w = aluout_EX;
  assign mem_we_MEM_w = mem_we_EX;
  assign mem_re_MEM_w = mem_re_EX;
  assign aluout_MEM_w = aluout_EX;
  assign reg_we_MEM_w = reg_we_EX;
  assign reg_wr_src_sel_MEM_w = reg_wr_src_sel_EX;
  assign dst_reg_MEM_w = dst_reg_EX;
  
  // Read from D-MEM
  assign mem_val_out_MEM_w = (mem_addr_MEM_w == ADDRKEY) ? {{(DBITS-KEYBITS){1'b0}}, ~KEY} :
                                    dmem[mem_addr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]];

  // Write to D-MEM
  always @ (posedge clk) begin
    if(mem_we_MEM_w)
      dmem[mem_addr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]] <= regval2_EX;
  end

  always @ (posedge clk or posedge reset) begin
    if(reset) begin
        PC_MEM             <= {DBITS{1'b0}};
        mem_val_out_MEM    <= {DBITS{1'b0}};
        aluout_MEM         <= {DBITS{1'b0}};
        dst_reg_MEM        <= {REGNOBITS{1'b0}};
        reg_we_MEM         <= {2{1'b0}};
        reg_wr_src_sel_MEM <= {2{1'b0}};
    end else begin
        PC_MEM             <= PC_MEM_w;
        mem_val_out_MEM    <= mem_val_out_MEM_w;
        aluout_MEM         <= aluout_MEM_w;
        dst_reg_MEM        <= dst_reg_MEM_w;
        reg_we_MEM         <= reg_we_MEM_w;
        reg_wr_src_sel_MEM <= reg_wr_src_sel_MEM_w;
    end
  end


  /*** WRITE BACK STAGE ***/ 

  // Wires
  wire                 reg_we_WB_w; 
  wire [1:0]           reg_wr_src_sel_WB_w;
  wire [REGNOBITS-1:0] dst_reg_WB_w;
  wire [DBITS-1:0]     PC_WB_w;
  wire [DBITS-1:0]     mem_val_out_WB_w;
  wire [DBITS-1:0]     aluout_WB_w;

  // Assignments to wires from MEM buffer
  assign reg_we_WB_w         = reg_we_MEM;
  assign reg_wr_src_sel_WB_w = reg_wr_src_sel_MEM;
  assign dst_reg_WB_w        = dst_reg_MEM;
  assign PC_WB_w             = PC_MEM;
  assign mem_val_out_WB_w    = mem_val_out_MEM;
  assign aluout_WB_w         = aluout_MEM;
  
  // Definitions of possible values for RegWrSrcSel
  parameter WRITE_PC       = 2'b00;
  parameter WRITE_MEM_DATA = 2'b01;
  parameter WRITE_ALUOUT   = 2'b10;
  
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
    end
    else if(reg_we_WB_w) begin
        case (reg_wr_src_sel_WB_w)
            WRITE_PC:       regs[dst_reg_WB_w] <= PC_WB_w;
            WRITE_MEM_DATA: regs[dst_reg_WB_w] <= mem_val_out_WB_w;
            WRITE_ALUOUT:   regs[dst_reg_WB_w] <= aluout_WB_w;
        endcase
    end
  end
  
  /*** Branch Handling Logic ***/
  
    reg[0:0] aluout_EX_r_1bit;
    reg[0:0] take_branch; //0 = don't take branch, 1 = DO take branch
    reg[1:0] take_branch_sxt;
    reg[0:0] is_jal; //0 = not jal, 1 = is jal

    //if opcode is a branch or JAL
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
	 
  /*** Stall Logic ***/
  
  //stall for branch and jal
  parameter is_branch_or_jal = 3'b001;
  assign branch_or_jal_stall = ~(op1_ID[5:3] == is_branch_or_jal);
  
  wire should_stall_ID;
  wire should_stall_EX;
  wire should_stall_MEM;
  
  assign should_stall_ID = op1_ID_w == OP1_ALUR | op1_ID_w == OP1_BEQ | op1_ID_w == OP1_BLT
    | op1_ID_w == OP1_BLE | op1_ID_w == OP1_BNE | op1_ID_w == OP1_SW;
  
  assign should_stall_EX = ((dst_reg_EX_r == rt_ID_w) & should_stall_ID & reg_we_ID)
    | ((dst_reg_EX_r == rs_ID_w) & reg_we_ID);
   
  assign should_stall_MEM = ((dst_reg_MEM_w == rt_ID_w) & should_stall_ID & reg_we_MEM_w)
    | ((dst_reg_MEM_w == rs_ID_w) & reg_we_MEM_w);
    
  assign stall_logic_out = ~(should_stall_EX | should_stall_MEM);
      
   

  /*** I/O ***/
  // Create and connect HEX register
 
  SevenSeg ss5(.OUT(HEX5), .IN(HEX_out[23:20]), .OFF(1'b0));
  SevenSeg ss4(.OUT(HEX4), .IN(HEX_out[19:16]), .OFF(1'b0));
  SevenSeg ss3(.OUT(HEX3), .IN(HEX_out[15:12]), .OFF(1'b0));
  SevenSeg ss2(.OUT(HEX2), .IN(HEX_out[11:8]), .OFF(1'b0));
  SevenSeg ss1(.OUT(HEX1), .IN(HEX_out[7:4]), .OFF(1'b0));
  SevenSeg ss0(.OUT(HEX0), .IN(HEX_out[3:0]), .OFF(1'b0));
  
  always @ (posedge clk or posedge reset) begin
    if(reset)
        HEX_out <= 24'hFEDEAD;
    else if(mem_we_MEM_w && (mem_addr_MEM_w == ADDRHEX))
        HEX_out <= regval2_EX[HEXBITS-1:0];
  end

  reg [9:0] LEDR_out;
  
  always @ (posedge clk or posedge reset) begin
    if(reset)
        LEDR_out <= 10'b0000000000;
    else if(mem_we_MEM_w && (mem_addr_MEM_w == ADDRLEDR))
        LEDR_out <= regval2_EX[LEDRBITS-1:0];
  end

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

always @ (*) begin
  
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
        NEWPCSRC_OUT = 2'b11;
        MEMWE_OUT = 1'b0;
        MEMRE_OUT = 1'b0;
        REGWE_OUT = 1'b0;
        REGWRSRCSEL_OUT = 2'b00; //don't care, default
        REGWRDSTSEL_OUT = 1'b0; //don't care, default
    end

    OP1_BLT : begin
        ALUSRC_OUT = 2'b00;
        NEWPCSRC_OUT = 2'b11;
        MEMWE_OUT = 1'b0;
        MEMRE_OUT = 1'b0;
        REGWE_OUT = 1'b0;
        REGWRSRCSEL_OUT = 2'b00; //don't care, default
        REGWRDSTSEL_OUT = 1'b0; //don't care, default
    end

    OP1_BLE : begin
        ALUSRC_OUT = 2'b00;
        NEWPCSRC_OUT = 2'b11;
        MEMWE_OUT = 1'b0;
        MEMRE_OUT = 1'b0;
        REGWE_OUT = 1'b0;
        REGWRSRCSEL_OUT = 2'b00; //don't care, default
        REGWRDSTSEL_OUT = 1'b0; //don't care, default
    end

    OP1_BNE : begin
        ALUSRC_OUT = 2'b00;
        NEWPCSRC_OUT = 2'b11;
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