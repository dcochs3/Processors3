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
    parameter IMEMINITFILE = "test.mif";
    //parameter IMEMINITFILE = "fmedian2.mif";

    parameter IMEMADDRBITS = 16;
    parameter IMEMWORDBITS = 2;
    parameter IMEMWORDS    = (1 << (IMEMADDRBITS - IMEMWORDBITS));
    parameter DMEMADDRBITS = 16;
    parameter DMEMWORDBITS = 2;
    parameter DMEMWORDS	   = (1 << (DMEMADDRBITS - DMEMWORDBITS));
 
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
    // The PC register and update logic
    wire [DBITS-1:0] pcplus_FE;
    wire [DBITS-1:0] pcpred_FE;
    wire [DBITS-1:0] inst_FE_w;
    wire stall_pipe;
    wire mispred_EX_w;

    wire [DBITS-1:0] pcgood_EX_w;
    reg [DBITS-1:0] PC_FE;
    reg [INSTBITS-1:0] inst_FE;

    wire stall_pipe_branch;
    wire stall_pipe_reg_rd;
    wire stall_pipe_reg_rd_rs;
    wire stall_pipe_rt_check;
    wire stall_pipe_reg_rd_rt;
    wire stall_pipe_mem_rd;
    wire [REGNOBITS-1:0] dst_reg_ID_w;
    reg [REGNOBITS-1:0] dst_reg_EX;
    reg [REGNOBITS-1:0] dst_reg_MEM;
    reg signed [DBITS-1:0] aluout_EX_r;
    reg [5:0] op1_EX;
    reg [DBITS-1:0] aluout_EX;
    reg [REGNOBITS-1:0] dst_reg_ID;
    reg [DBITS-1:0] PC_REG;
    wire is_br_ID_w;
    wire is_jmp_ID_w;
    reg is_nop_FE;
    reg is_nop_ID;
    reg is_nop_EX;
    reg is_nop_MEM;
    reg mispred_EX;

    // I-MEM
    (* ram_init_file = IMEMINITFILE *)
    reg [DBITS-1:0] imem [IMEMWORDS-1:0];

    // This statement is used to initialize the I-MEM
    // during simulation using Model-Sim
    initial begin
        $readmemh("test.hex", imem);
    end

    assign inst_FE_w = imem[PC_REG[IMEMADDRBITS-1:IMEMWORDBITS]];

    always @ (posedge clk or posedge reset) begin
        if(reset) begin
            PC_REG <= STARTPC;
            PC_FE <= {DBITS{1'b0}};
        end
        else if(mispred_EX_w) begin //use branch or jal target address
            PC_REG <= pcgood_EX_w;
            PC_FE <= pcgood_EX_w;
        end
        //else if(stall_pipe_branch) begin
            //PC_REG <= PC_REG;
            //PC_FE <= PC_FE;
        //end
        else if (stall_pipe_reg_rd) begin
            PC_REG <= PC_REG;
            PC_FE <= PC_FE;
        end
        else if (stall_pipe_mem_rd) begin
            PC_REG <= PC_REG;
            PC_FE <= PC_FE;
        end
        //else if (is_br_ID_w || is_jmp_ID_w) begin
            //flush
            //PC_REG <= PC_REG;
            //PC_FE <= {DBITS{1'b0}};
        //end
        else begin
            PC_REG <= pcplus_FE;
            PC_FE <= pcplus_FE;
        end
    end

    // This is the value of "incremented PC", computed in the FE stage
    assign pcplus_FE = PC_REG + INSTSIZE;
    assign pcpred_FE = pcplus_FE;

    // FE_latch
    always @ (posedge clk or posedge reset) begin
        if(reset) begin
            inst_FE <= {INSTBITS{1'b0}};
            is_nop_FE <= 1'b0;
        end
        else if (stall_pipe_reg_rd)
            inst_FE <= inst_FE;
        else if (mispred_EX_w) begin
            inst_FE <= {INSTBITS{1'b0}};
            is_nop_FE <= 1'b1;
        end
        else if (stall_pipe_mem_rd) begin
            inst_FE <= inst_FE;
        end
        else begin
            inst_FE <= inst_FE_w;
            is_nop_FE <= 1'b0;
        end
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
    //wire is_br_ID_w;
    //wire is_jmp_ID_w;
    wire rd_mem_ID_w;
    wire wr_mem_ID_w;
    wire wr_reg_ID_w;
    wire [4:0] ctrlsig_ID_w;
    wire [2:0] ctrlsig_EX_w;
    wire [REGNOBITS-1:0] wregno_ID_w;
    wire wr_reg_EX_w;
    wire wr_reg_MEM_w;
    wire [DBITS-1:0] mem_addr_ID_w;

    // Register file
    reg [DBITS-1:0] PC_ID;
    reg [DBITS-1:0] regs [REGWORDS-1:0];
    reg signed [DBITS-1:0] regval1_ID;
    reg signed [DBITS-1:0] regval2_ID;
    reg signed [DBITS-1:0] immval_ID;
    reg [OP1BITS-1:0] op1_ID;
    reg [OP2BITS-1:0] op2_ID;
    reg [4:0] ctrlsig_ID;
    reg [2:0] ctrlsig_EX;
    reg [0:0] ctrlsig_MEM;
    reg [REGNOBITS-1:0] wregno_ID;
    // Declared here for stall check
    reg [REGNOBITS-1:0] wregno_EX;
    reg [REGNOBITS-1:0] wregno_MEM;
    reg [INSTBITS-1:0] inst_ID;
    reg [INSTBITS-1:0] inst_EX;
    reg [INSTBITS-1:0] inst_MEM;

    // Specify signals such as op*_ID_w, imm_ID_w, r*_ID_w
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
    //control signal wires
    wire [1:0] alu_src_ID_w;
    wire [1:0] new_pc_src_ID_w;
    wire [0:0] mem_we_ID_w;
    wire [0:0] mem_re_ID_w;
    wire [0:0] reg_we_ID_w;
    wire [1:0] reg_wr_src_sel_ID_w;
    wire [0:0] reg_wr_dst_sel_ID_w;

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

    // You may add or change control signals if needed
    assign is_br_ID_w = (op1_ID_w == OP1_BEQ || op1_ID_w == OP1_BLT || op1_ID_w == OP1_BLE || op1_ID_w == OP1_BNE);
    assign is_jmp_ID_w = op1_ID_w == OP1_JAL;

    assign mem_addr_ID_w = regval1_ID_w + sxt_imm_ID_w; //for sw/lw

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

    assign rd_mem_ID_w = mem_re_ID_w;
    assign wr_mem_ID_w = mem_we_ID_w;
    assign wr_reg_ID_w = reg_we_ID_w;

    assign ctrlsig_ID_w = {is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};

    // Specify stall condition
    assign stall_pipe = (stall_pipe_branch || stall_pipe_reg_rd || stall_pipe_mem_rd);

    assign stall_pipe_branch = (op1_ID_w != 6'b000000) && (op1_ID_w == OP1_BEQ || op1_ID_w == OP1_BLT || op1_ID_w == OP1_BLE || op1_ID_w == OP1_BNE || op1_ID_w == OP1_JAL);

    assign stall_pipe_reg_rd = stall_pipe_reg_rd_rs || (stall_pipe_rt_check && stall_pipe_reg_rd_rt);

    assign stall_pipe_reg_rd_rs = (rs_ID_w != 4'b0000) && ((rs_ID_w == dst_reg_ID) || (rs_ID_w == dst_reg_EX) || (rs_ID_w == dst_reg_MEM));

    assign stall_pipe_rt_check = (op1_ID_w == OP1_ALUR) || (op1_ID_w == OP1_BEQ) || (op1_ID_w == OP1_BLT) || (op1_ID_w == OP1_BLE) || (op1_ID_w == OP1_BNE) || (op1_ID_w == OP1_SW);

    assign stall_pipe_reg_rd_rt = (rt_ID_w != 4'b0000) && ((rt_ID_w == dst_reg_ID) || (rt_ID_w == dst_reg_EX) || (rt_ID_w == dst_reg_MEM)); //(rt_ID_w == dst_reg_ID_w) || 

    assign stall_pipe_mem_rd = ((mem_addr_ID_w != {DBITS{1'b0}}) && (op1_ID_w != 6'b000000)) && (op1_ID_w == OP1_LW && ((op1_ID_w == OP1_SW && aluout_EX_r == mem_addr_ID_w) || (op1_EX == OP1_SW && aluout_EX == mem_addr_ID_w)));
 
    assign dst_reg_ID_w = (reg_wr_dst_sel_ID_w == 0) ? rt_ID_w : rd_ID_w;

    // ID_latch
    always @ (posedge clk or posedge reset) begin
        if(reset) begin
            PC_ID      <= {DBITS{1'b0}};
            rt_spec_ID <= {REGNOBITS{1'b0}}; //RtSpec
            regval2_ID <= {DBITS{1'b0}}; //RtCont
            regval1_ID <= {DBITS{1'b0}}; //RsCont
            sxt_imm_ID <= {DBITS{1'b0}}; //sxtImm

            rd_spec_ID        <= {REGNOBITS{1'b0}}; //RdSpec
            alu_src_ID        <= 2'b00; //ALUSrc
            new_pc_src_ID     <= 2'b00; //NewPCSrc
            mem_we_ID         <= 1'b0; //MemWE
            mem_re_ID         <= 1'b0; //MemRE
            reg_we_ID         <= 1'b0; //RegWE
            reg_wr_src_sel_ID <= 2'b00; //RegWrSrcSel
            reg_wr_dst_sel_ID <= 1'b0; //RegWrDstSel
            dst_reg_ID        <= {REGNOBITS{1'b0}};

            //these are in place of ALUOp
            op1_ID     <= {OP1BITS{1'b0}};
            op2_ID     <= {OP2BITS{1'b0}};
            ctrlsig_ID <= 5'b00000;
            inst_ID    <= {INSTBITS{1'b0}};
            is_nop_ID  <= 1'b0;
        end else if (stall_pipe_reg_rd) begin
            PC_ID      <= {DBITS{1'b0}};
            rt_spec_ID <= {REGNOBITS{1'b0}}; //RtSpec
            regval2_ID <= {DBITS{1'b0}}; //RtCont
            regval1_ID <= {DBITS{1'b0}}; //RsCont
            sxt_imm_ID <= {DBITS{1'b0}}; //sxtImm

            rd_spec_ID        <= {REGNOBITS{1'b0}}; //RdSpec
            alu_src_ID        <= 2'b00; //ALUSrc
            new_pc_src_ID     <= 2'b00; //NewPCSrc
            mem_we_ID         <= 1'b0; //MemWE
            mem_re_ID         <= 1'b0; //MemRE
            reg_we_ID         <= 1'b0; //RegWE
            reg_wr_src_sel_ID <= 2'b00; //RegWrSrcSel
            reg_wr_dst_sel_ID <= 1'b0; //RegWrDstSel
            dst_reg_ID        <= {REGNOBITS{1'b0}};

            //these are in place of ALUOp
            op1_ID     <= {OP1BITS{1'b0}};
            op2_ID     <= {OP2BITS{1'b0}};
            ctrlsig_ID <= 5'b00000;
            inst_ID    <= {INSTBITS{1'b0}};
            is_nop_ID  <= 1'b1;
        end else if (stall_pipe_mem_rd) begin
            PC_ID      <= {DBITS{1'b0}};
            rt_spec_ID <= {REGNOBITS{1'b0}}; //RtSpec
            regval2_ID <= {DBITS{1'b0}}; //RtCont
            regval1_ID <= {DBITS{1'b0}}; //RsCont
            sxt_imm_ID <= {DBITS{1'b0}}; //sxtImm

            rd_spec_ID        <= {REGNOBITS{1'b0}}; //RdSpec
            alu_src_ID        <= 2'b00; //ALUSrc
            new_pc_src_ID     <= 2'b00; //NewPCSrc
            mem_we_ID         <= 1'b0; //MemWE
            mem_re_ID         <= 1'b0; //MemRE
            reg_we_ID         <= 1'b0; //RegWE
            reg_wr_src_sel_ID <= 2'b00; //RegWrSrcSel
            reg_wr_dst_sel_ID <= 1'b0; //RegWrDstSel
            dst_reg_ID        <= {REGNOBITS{1'b0}};

            //these are in place of ALUOp
            op1_ID     <= {OP1BITS{1'b0}};
            op2_ID     <= {OP2BITS{1'b0}};
            ctrlsig_ID <= 5'b00000;
            inst_ID    <= {INSTBITS{1'b0}};
            is_nop_ID  <= 1'b1;
        end else if (mispred_EX_w) begin
            //flush
            PC_ID      <= {DBITS{1'b0}};
            rt_spec_ID <= {REGNOBITS{1'b0}}; //RtSpec
            regval2_ID <= {DBITS{1'b0}}; //RtCont
            regval1_ID <= {DBITS{1'b0}}; //RsCont
            sxt_imm_ID <= {DBITS{1'b0}}; //sxtImm

            rd_spec_ID        <= {REGNOBITS{1'b0}}; //RdSpec
            alu_src_ID        <= 2'b00; //ALUSrc
            new_pc_src_ID     <= 2'b00; //NewPCSrc
            mem_we_ID         <= 1'b0; //MemWE
            mem_re_ID         <= 1'b0; //MemRE
            reg_we_ID         <= 1'b0; //RegWE
            reg_wr_src_sel_ID <= 2'b00; //RegWrSrcSel
            reg_wr_dst_sel_ID <= 1'b0; //RegWrDstSel
            dst_reg_ID        <= {REGNOBITS{1'b0}};

            //these are in place of ALUOp
            op1_ID     <= {OP1BITS{1'b0}};
            op2_ID     <= {OP2BITS{1'b0}};
            ctrlsig_ID <= 5'b00000;
            inst_ID    <= {INSTBITS{1'b0}};
            is_nop_ID  <= 1'b1;
        end else begin
            // Specify ID latches
            PC_ID      <= PC_FE; //PC 
            rt_spec_ID <= rt_ID_w; //RtSpec
            regval2_ID <= regval2_ID_w; //RtCont
            regval1_ID <= regval1_ID_w; //RsCont
            sxt_imm_ID <= sxt_imm_ID_w; //sxtImm
        
            dst_reg_ID        <= dst_reg_ID_w;
            rd_spec_ID        <= rd_ID_w; //RdSpec
            alu_src_ID        <= alu_src_ID_w; //ALUSrc
            new_pc_src_ID     <= new_pc_src_ID_w; //NewPCSrc
            mem_we_ID         <= mem_we_ID_w; //MemWE
            mem_re_ID         <= mem_re_ID_w; //MemRE
            reg_we_ID         <= reg_we_ID_w; //RegWE
            reg_wr_src_sel_ID <= reg_wr_src_sel_ID_w; //RegWrSrcSel
            reg_wr_dst_sel_ID <= reg_wr_dst_sel_ID_w; //RegWrDstSel
        
            //these are in place of ALUOp
            op1_ID     <= op1_ID_w;
            op2_ID     <= op2_ID_w;
            ctrlsig_ID <= ctrlsig_ID_w;
            inst_ID    <= inst_FE;
            is_nop_ID  <= 1'b0;

        end
    end


    //*** EXEC STAGE ***//

    wire is_br_EX_w;
    wire is_jmp_EX_w;

    //reg [INSTBITS-1:0] inst_EX; /* This is for debugging */
    reg br_cond_EX;
    // Note that aluout_EX_r is declared as reg, but it is output signal from combi logic
    reg [DBITS-1:0] regval2_EX;
    wire signed [DBITS-1:0] alu_in_EX_r;

    reg [DBITS-1:0] PC_EX;
    reg [0:0] mem_we_EX;
    reg [0:0] mem_re_EX;
    reg [0:0] reg_we_EX;
    reg [1:0] reg_wr_src_sel_EX;

    assign alu_in_EX_r = (alu_src_ID == 00) ? regval2_ID : //take RtCont
                        (alu_src_ID == 01) ? sxt_imm_ID : //take sxtImm
                        sxt_imm_ID << 2; //take sxtImm x 4
                        //there should never be a case where alu_src_ID == 11

    always @ (*) begin
        case (op1_ID)
            OP1_BEQ : br_cond_EX = (regval1_ID == regval2_ID); //TO DO: should these be regval1_ID and alu_in_EX_r
            OP1_BLT : br_cond_EX = (regval1_ID < regval2_ID);
            OP1_BLE : br_cond_EX = (regval1_ID <= regval2_ID);
            OP1_BNE : br_cond_EX = (regval1_ID != regval2_ID);
            OP1_JAL : regs[rt_spec_ID] <= PC_ID;
            // OP1_JAL : br_cond_EX = 1'b1; //JAL is always taken aka always "mispredicted"
        default : br_cond_EX = 1'b0;
    endcase
    if(op1_ID == OP1_ALUR)
        case (op2_ID)
            OP2_EQ   : aluout_EX_r = {31'b0, regval1_ID == alu_in_EX_r};
            OP2_LT   : aluout_EX_r = {31'b0, regval1_ID < alu_in_EX_r};
            OP2_LE   : aluout_EX_r = {31'b0, regval1_ID <= alu_in_EX_r};
            OP2_NE   : aluout_EX_r = {31'b0, regval1_ID != alu_in_EX_r};

            OP2_ADD  : aluout_EX_r = regval1_ID + alu_in_EX_r;
            OP2_AND  : aluout_EX_r = regval1_ID & alu_in_EX_r;
            OP2_OR   : aluout_EX_r = regval1_ID | alu_in_EX_r;
            OP2_XOR  : aluout_EX_r = regval1_ID ^ alu_in_EX_r; //xor
            OP2_SUB  : aluout_EX_r = regval1_ID - alu_in_EX_r;
            OP2_NAND : aluout_EX_r = ~(regval1_ID & alu_in_EX_r); //nand
            OP2_NOR  : aluout_EX_r = ~(regval1_ID | alu_in_EX_r); //nor
            OP2_NXOR : aluout_EX_r = ~(regval1_ID ^ alu_in_EX_r); //xnor
            OP2_RSHF : aluout_EX_r = regval1_ID >>> alu_in_EX_r; //arithmetic shift
            OP2_LSHF : aluout_EX_r = regval1_ID <<< alu_in_EX_r; //arithmetic shift
            
            default     : aluout_EX_r = {DBITS{1'b0}};
        endcase
    else if(op1_ID == OP1_LW || op1_ID == OP1_SW || op1_ID == OP1_ADDI)
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

    assign ctrlsig_EX_w = {rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};

    // Specify signals such as mispred_EX_w, pcgood_EX_w
    assign mispred_EX_w = br_cond_EX || (op1_ID == OP1_JAL);
    assign pcgood_EX_w = (op1_ID == OP1_JAL)?(regval1_ID + (sxt_imm_ID << 2)):
                       (br_cond_EX)?(PC_ID + (sxt_imm_ID << 2)):
                       PC_FE + INSTSIZE; //this case should not matter
                       
    // EX_latch
    always @ (posedge clk or posedge reset) begin
        if(reset) begin
            PC_EX <= {DBITS{1'b0}}; //PC
            regval2_EX <= {DBITS{1'b0}}; //RtCont
            aluout_EX <= {DBITS{1'b0}}; //ALUResult
            dst_reg_EX <= {REGNOBITS{1'b0}}; //DstReg
            mem_we_EX <= 1'b0; //MemWE
            mem_re_EX <= 1'b0; //MemRE
            reg_we_EX <= 1'b0; //RegWE
            reg_wr_src_sel_EX <= 2'b00; //RegWrSrcSel
            op1_EX     <= 6'b000000;
            ctrlsig_EX <= 3'b000;
            inst_EX <= {INSTBITS{1'b0}};
            is_nop_EX <= 1'b0;
        end else if (mispred_EX_w) begin
            //do not latch
            //flush
            PC_EX <= {DBITS{1'b0}}; //PC
            regval2_EX <= {DBITS{1'b0}}; //RtCont
            aluout_EX <= {DBITS{1'b0}}; //ALUResult
            dst_reg_EX <= {REGNOBITS{1'b0}}; //DstReg
            mem_we_EX <= 1'b0; //MemWE
            mem_re_EX <= 1'b0; //MemRE
            reg_we_EX <= 1'b0; //RegWE
            reg_wr_src_sel_EX <= 2'b00; //RegWrSrcSel
            op1_EX     <= 6'b000000;
            ctrlsig_EX <= 3'b000;
            inst_EX <= {INSTBITS{1'b0}};
            is_nop_EX <= 1'b1;
        end else begin
            // Specify EX latches
            PC_EX <= PC_ID; //PC
            regval2_EX <= regval2_ID; //RtCont
            aluout_EX <= aluout_EX_r; //ALUResult
            dst_reg_EX <= dst_reg_ID; //DstReg        
            mem_we_EX <= mem_we_ID; //MemWE
            mem_re_EX <= mem_re_ID; //MemRE
            reg_we_EX <= reg_we_ID; //RegWE
            reg_wr_src_sel_EX <= reg_wr_src_sel_ID; //RegWrSrcSel
            op1_EX     <= op1_ID;
            ctrlsig_EX <= ctrlsig_EX_w;
            inst_EX <= inst_ID;
            is_nop_EX <= is_nop_ID;
        end
    end


    //*** MEM STAGE ***//

    wire rd_mem_MEM_w;
    wire wr_mem_MEM_w;

    wire [DBITS-1:0] PC_MEM_w;
    wire [DBITS-1:0] mem_addr_MEM_w;
    wire [DBITS-1:0] mem_val_out_MEM_w;
    wire mem_we_MEM_w;
    wire mem_re_MEM_w;
    wire [DBITS-1:0] aluout_MEM_w;
    wire [REGNOBITS-1:0] dst_reg_MEM_w;
    wire reg_we_MEM_w;
    wire [1:0] reg_wr_src_sel_MEM_w;

    //reg [INSTBITS-1:0] inst_MEM; /* This is for debugging */
    reg [DBITS-1:0] PC_MEM;
    reg [DBITS-1:0] mem_val_out_MEM;
    reg [DBITS-1:0] aluout_MEM;
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

    assign rd_mem_MEM_w = ctrlsig_EX[2];
    assign wr_mem_MEM_w = ctrlsig_EX[1];
    assign wr_reg_MEM_w = ctrlsig_EX[0];

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
            ctrlsig_MEM        <= 1'b0;
            inst_MEM           <= {INSTBITS{1'b0}};
            is_nop_MEM         <= 1'b0;
        end else begin
            PC_MEM             <= PC_MEM_w;
            mem_val_out_MEM    <= mem_val_out_MEM_w;
            aluout_MEM         <= aluout_MEM_w;
            dst_reg_MEM        <= dst_reg_MEM_w;
            reg_we_MEM         <= reg_we_MEM_w;
            reg_wr_src_sel_MEM <= reg_wr_src_sel_MEM_w;
            ctrlsig_MEM        <= ctrlsig_EX[0];
            inst_MEM           <= inst_EX;
            is_nop_MEM         <= is_nop_EX;
        end
    end


    /*** WRITE BACK STAGE ***/

    // Wires
    wire                   reg_we_WB_w; 
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
    parameter WRITE_ALUOUT = 2'b10;

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
        end else if(reg_we_WB_w) begin
            case (reg_wr_src_sel_WB_w)
                WRITE_PC:       regs[dst_reg_WB_w] <= PC_WB_w;
                WRITE_MEM_DATA: regs[dst_reg_WB_w] <= mem_val_out_WB_w;
                WRITE_ALUOUT:   regs[dst_reg_WB_w] <= aluout_WB_w;
            endcase
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
        else if(mem_we_MEM_w && (mem_addr_MEM_w == ADDRHEX))
            HEX_out <= regval2_EX[HEXBITS-1:0];
    end

    // TODO: Write the code for LEDR here

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