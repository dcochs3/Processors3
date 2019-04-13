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

    parameter DBITS      = 32;
    parameter INSTSIZE   = 32'd4;
    parameter INSTBITS   = 32;
    parameter REGNOBITS  = 4;
    parameter REGWORDS   = (1 << REGNOBITS);
    parameter IMMBITS    = 16;
    parameter STARTPC    = 32'h100;
    parameter ADDRHEX    = 32'hFFFFF000;
    parameter ADDRLEDR   = 32'hFFFFF020;
    parameter ADDRKEY    = 32'hFFFFF080;
    parameter ADDRKCTRL  = 32'hFFFFF084;
    parameter ADDRSW     = 32'hFFFFF090;
    parameter ADDRSWCTRL = 32'hFFFFF094;
    parameter ADDRTCNT   = 32'hFFFFF100;
    parameter ADDRTLIM   = 32'hFFFFF104;
    parameter ADDRTCTRL  = 32'hFFFFF108;

    // Change this to fmedian2.mif before submitting
//    parameter IMEMINITFILE = "fmedian2.mif";
    parameter IMEMINITFILE = "xmas.mif";

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
    
    parameter OP1_SYS  = 6'b111111;

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
    
    parameter OP2_RETI = 8'b00000001;
    parameter OP2_RSR  = 8'b00000010;
    parameter OP2_WSR  = 8'b00000011;

    parameter HEXBITS    = 24;
    parameter LEDRBITS   = 10;
    parameter KEYBITS    = 4;
    parameter KCTRLBITS  = 3;
    parameter SWBITS     = 10;
    parameter SWCTRLBITS = 3;
    parameter TCNTBITS   = 32;
    parameter TLIMBITS   = 32;
    parameter TCTRLBITS  = 3;
    
    // Used to index into control/status registers
    parameter READYBIT   = 0;
    parameter OVERRUNBIT = 1;
    parameter IEBIT      = 2;
 
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
    
    wire [DBITS-1:0] mem_addr_MEM_w;
    wire             mem_we_MEM_w;
    reg  [DBITS-1:0] regval2_EX;
    
    reg is_nop_FE;
    reg is_nop_ID;
    reg is_nop_EX;
    reg is_nop_MEM;
    
    wire forward_rs_from_EX;
    wire forward_rs_from_MEM;
    
    wire forward_rs_from_EX_sys;
    wire forward_rs_from_MEM_sys;
    
    //For interrupt handling
    reg [1:0] PCS;       //processor control status -- bit 0 is IE, bit 1 is OIE
    reg [DBITS-1:0] IHA; //interrupt handler address
    reg [DBITS-1:0] IRA; //interrupt return address
    reg [2:0] IDN;       //interrupt device number -- this is 3 bits wide because there are 3 devices (key, switch, timer) that generate interrupts
  
    wire rdsysreg_ID_w;
    wire wrsysreg_EX_w;
    wire wrsysreg_MEM_w;
    
    //*** Interrupt Related Things ***//
    wire key_IRQ;
    wire sw_IRQ;
    wire t_IRQ;
    wire proc_IRQ;
    
    wire [KCTRLBITS-1:0]  kctrl_reg;
    wire [SWCTRLBITS-1:0] swctrl_reg;
    wire [TCTRLBITS-1:0]  tctrl_reg;
    
    wire pipeline_not_empty = !is_nop_FE || !is_nop_ID || !is_nop_EX || !is_nop_MEM;
    
    wire pipeline_empty = is_nop_FE && is_nop_ID && is_nop_EX && is_nop_MEM;
    
    assign key_IRQ = kctrl_reg[IEBIT] && kctrl_reg[READYBIT];
    assign sw_IRQ = swctrl_reg[IEBIT] && swctrl_reg[READYBIT];
    assign t_IRQ = tctrl_reg[IEBIT] && tctrl_reg[READYBIT];
    
    //This is set by ORing the individual devices' interrupt request signals
    //And it is only set if interrupts are enabled (PCS[0] == 1)
    assign proc_IRQ = PCS[0] && (key_IRQ || sw_IRQ || t_IRQ);
    
    parameter TIMER_DEV_NUM = 4'b0001;
    parameter KEY_DEV_NUM   = 4'b0010;
    parameter SW_DEV_NUM    = 4'b0011;
    
    //Device number priority encoding
    wire [3:0] device_num =
        t_IRQ ? TIMER_DEV_NUM :
        key_IRQ ? KEY_DEV_NUM :
        sw_IRQ ? SW_DEV_NUM :
        4'b1111;
        
    wire servicing_timer  = proc_IRQ && pipeline_empty && (device_num == TIMER_DEV_NUM);
    wire servicing_key    = proc_IRQ && pipeline_empty && (device_num == KEY_DEV_NUM);
    wire servicing_switch = proc_IRQ && pipeline_empty && (device_num == SW_DEV_NUM);
  
    //*** IO DEVICES AND CONNECTIONS ***//
    
    wire [DBITS-1:0] io_abus;
    
    // We need to know if the address in MEM stage is the address of
    // a memory-mapped IO device
    assign io_abus = mem_addr_MEM_w;
  
    //** LEDR **//
  
    tri  [DBITS-1:0]    ledr_dbus;
    wire                ledr_we;
    wire [LEDRBITS-1:0] ledr_out;
    
    assign ledr_we = mem_we_MEM_w && (mem_addr_MEM_w == ADDRLEDR);

    // If memory is write-enabled, use regval2_EX, the memory input value;
    // if memory is not write-enabled, use high-impedance to allow LEDR
    // to drive it's input value to the data bus
    assign ledr_dbus = ledr_we ? regval2_EX : {DBITS{1'bz}};
    
    // Note that the ledr_out wire is used as an intermediate value so that
    // we have a way to assign the current output of our device module to
    // the device itself
    assign LEDR = ledr_out;
  
    LEDR_DEV my_ledr (.ABUS(io_abus), .DBUS(ledr_dbus), .WE(ledr_we), .CLK(clk), .RESET(reset), .LEDR_OUT(ledr_out));
  
    //** HEX **//
    
    tri  [DBITS-1:0]   hex_dbus;
    wire               hex_we;
    wire [HEXBITS-1:0] HEX;
    wire [HEXBITS-1:0] hex_out;
 
    // Connect hex_out to the 6 hex displays
    SevenSeg ss5(.OUT(HEX5), .IN(HEX[23:20]), .OFF(1'b0));
    SevenSeg ss4(.OUT(HEX4), .IN(HEX[19:16]), .OFF(1'b0));
    SevenSeg ss3(.OUT(HEX3), .IN(HEX[15:12]), .OFF(1'b0));
    SevenSeg ss2(.OUT(HEX2), .IN(HEX[11:8]), .OFF(1'b0));
    SevenSeg ss1(.OUT(HEX1), .IN(HEX[7:4]), .OFF(1'b0));
    SevenSeg ss0(.OUT(HEX0), .IN(HEX[3:0]), .OFF(1'b0));
    
    assign hex_we = mem_we_MEM_w && (mem_addr_MEM_w == ADDRHEX);

    // If memory is write-enabled, use regval2_EX, the memory input value;
    // if memory is not write-enabled, use high-impedance to allow HEX
    // to drive it's input value to the data bus
    assign hex_dbus = hex_we ? regval2_EX : {DBITS{1'bz}};
    
    // Note that the hex_out wire is used as an intermediate value so that
    // we have a way to assign the current output of our device module to
    // the device itself
    assign HEX = hex_out;
  
    HEX_DEV my_hex (.ABUS(io_abus), .DBUS(hex_dbus), .WE(hex_we), .CLK(clk), .RESET(reset), .HEX_OUT(hex_out));

    //** KEY **//

    tri  [DBITS-1:0]     key_dbus;   // The current value of KDATA (=KEY)
    wire                 key_we;
    
    // Only check for KCTRL because KDATA cannot be written to
    assign key_we = mem_we_MEM_w && (mem_addr_MEM_w == ADDRKCTRL);
    
    // If memory is write-enabled, use regval2_EX, the memory input value;
    // if memory is not write-enabled, use high-impedance to allow KEY
    // to drive it's input value to the data bus
    assign key_dbus = key_we ? regval2_EX : {DBITS{1'bz}};

    KEY_DEV my_key (.ABUS(io_abus), .DBUS(key_dbus), .WE(key_we), .CLK(clk), .RESET(reset), .KEY_IN(KEY), .KCTRL_OUT(kctrl_reg), .SERVICE_SIG_IN(servicing_key));
    
    //** SW **//
    
    tri  [DBITS-1:0]      sw_dbus;
    wire                  sw_we;
    
    // Only check for SWCTRL because SWDATA cannot be written to
    assign sw_we = mem_we_MEM_w && (mem_addr_MEM_w == ADDRSWCTRL);
    
    // If memory is write-enabled, use regval2_EX, the memory input value;
    // if memory is not write-enabled, use high-impedance to allow SW
    // to drive it's input value to the data bus
    assign sw_dbus = sw_we ? regval2_EX : {DBITS{1'bz}};

    SW_DEV my_sw (.ABUS(io_abus), .DBUS(sw_dbus), .WE(sw_we), .CLK(clk), .RESET(reset), .SW_IN(SW), .SWCTRL_OUT(swctrl_reg), .SERVICE_SIG_IN(servicing_switch));  
    
    //** TIMER **//
    
    tri  [DBITS-1:0]     timer_dbus;
    wire                 timer_we;   // 1 if writing to TCNT, TLIM, *or* TCTRL
    
    assign timer_we = mem_we_MEM_w && ((mem_addr_MEM_w == ADDRTCNT)
                      || (mem_addr_MEM_w == ADDRTLIM) || (mem_addr_MEM_w == ADDRTCTRL));
    
    // If memory is write-enabled, use regval2_EX, the memory input value;
    // if memory is not write-enabled, use high-impedance to allow TIMER
    // to drive it's input value to the data bus
    assign timer_dbus = timer_we ? regval2_EX : {DBITS{1'bz}};
    
    TIMER_DEV my_timer (.ABUS(io_abus), .DBUS(timer_dbus), .WE(timer_we), .CLK(clk), .RESET(reset), .TCTRL_OUT(tctrl_reg), .SERVICE_SIG_IN(servicing_timer));
  
    //*** FETCH STAGE ***//
    // The PC register and update logic
    wire [DBITS-1:0] pcplus_FE;
    wire [DBITS-1:0] pcpred_FE;
    wire [DBITS-1:0] inst_FE_w;
    wire mispred_EX_w;

    wire [DBITS-1:0] pcgood_EX_w;
    reg [DBITS-1:0] PC_FE;
    reg [INSTBITS-1:0] inst_FE;
   
    wire data_dep_rs;
    wire data_dep_rt_check;
    wire data_dep_rt;
    wire forward_rt_from_EX;
    wire forward_rt_from_MEM;
    wire data_dep_mem;
    wire forward_mem_cont_from_EX;
    wire forward_mem_cont_from_MEM;
    wire stall_lw_EX;
    wire forward_lw_to_rs_from_MEM;
    wire forward_lw_to_rt_from_MEM;
    wire [DBITS-1:0] mem_val_out_MEM_w;
    
    wire [REGNOBITS-1:0] dst_reg_ID_w;
    reg [REGNOBITS-1:0] dst_reg_EX;
    reg [REGNOBITS-1:0] dst_reg_MEM;
    wire signed [DBITS-1:0] aluout_EX_r;
    reg [5:0] op1_EX;
    reg [7:0] op2_EX;
    reg [DBITS-1:0] aluout_EX;
    reg [REGNOBITS-1:0] dst_reg_ID;
    reg [DBITS-1:0] PC_REG;
    wire is_br_ID_w;
    wire is_jmp_ID_w;
    reg mispred_EX;
    
    
    parameter PCS_reg_ID = 4'b0000;
    parameter IHA_reg_ID = 4'b0001;
    parameter IRA_reg_ID = 4'b0010;
    parameter IDN_reg_ID = 4'b0011;

    // I-MEM
    (* ram_init_file = IMEMINITFILE *)
    reg [DBITS-1:0] imem [IMEMWORDS-1:0];

    // This statement is used to initialize the I-MEM
    // during simulation using Model-Sim
//    initial begin
//        $readmemh("xmas.hex", imem);
//        $readmemh("xmas.hex", dmem);
//    end
  
    assign inst_FE_w = imem[PC_REG[IMEMADDRBITS-1:IMEMWORDBITS]];

    always @ (posedge clk or posedge reset) begin
        if(reset) begin
            PC_REG <= STARTPC;
            PC_FE <= {DBITS{1'b0}};
        end
        else if (proc_IRQ && mispred_EX_w) begin
            PC_REG <= pcgood_EX_w;
            PC_FE <= pcgood_EX_w;
        end
        else if (proc_IRQ && pipeline_not_empty) begin
            PC_REG <= PC_REG;
            PC_FE <= PC_FE;
        end
        else if (proc_IRQ) begin          
            //Jump to the interrupt handler (set the PC value to be the address in IHA)
            PC_REG <= IHA;
            PC_FE <= IHA;
        end
        else if(mispred_EX_w) begin
            //use branch or jal target address
            PC_REG <= pcgood_EX_w;
            PC_FE <= pcgood_EX_w;
        end
        else if (stall_lw_EX) begin
            PC_REG <= PC_REG;
            PC_FE <= PC_FE;
        end
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
        else if (proc_IRQ || mispred_EX_w) begin
            inst_FE <= {INSTBITS{1'b0}};
            is_nop_FE <= 1'b1;
        end
        else if (stall_lw_EX) begin
            inst_FE <= inst_FE;
            is_nop_FE <= 1'b0;
        end else begin
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
    assign rd_ID_w = inst_FE[11:8];     //if OP2_WSR, rd maps to a system register
    assign rt_ID_w = inst_FE[3:0];
    assign rs_ID_w = inst_FE[7:4];      //if OP2_RSR, rs maps to a system register
    assign imm_ID_w = inst_FE[23:8];
    
    // Read register values
    //FORWARDING    
    assign regval1_ID_w = (forward_lw_to_rs_from_MEM) ? mem_val_out_MEM_w :
                          (forward_rs_from_EX_sys) ? aluout_EX_r :
                          (forward_rs_from_EX) ? aluout_EX_r :
                          (forward_rs_from_MEM_sys) ? aluout_EX :
                          (forward_rs_from_MEM) ? aluout_EX :
                          (rdsysreg_ID_w && (rs_ID_w == PCS_reg_ID)) ? PCS :                 
                          (rdsysreg_ID_w && (rs_ID_w == IHA_reg_ID)) ? IHA :
                          (rdsysreg_ID_w && (rs_ID_w == IRA_reg_ID)) ? IRA :
                          (rdsysreg_ID_w && (rs_ID_w == IDN_reg_ID)) ? IDN :
                          regs[rs_ID_w];
                          
    assign regval2_ID_w = (forward_mem_cont_from_EX) ? regval2_ID :
                          (forward_mem_cont_from_MEM) ? regval2_EX :
                          (forward_lw_to_rt_from_MEM) ? mem_val_out_MEM_w :
                          (forward_rt_from_EX) ? aluout_EX_r :
                          (forward_rt_from_MEM) ? aluout_EX : 
                          regs[rt_ID_w];
                          

    // Sign extension
    SXT mysxt (.IN(imm_ID_w), .OUT(sxt_imm_ID_w));

    // Specify control signals such as is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, etc.
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
    reg [REGNOBITS-1:0] rs_spec_ID; //RsSpec

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


    // Assign control signals (replaced control signal generator)
    assign alu_src_ID_w        = {(op1_ID_w == OP1_JAL),
                                 ((op1_ID_w == OP1_LW)
                                 || (op1_ID_w == OP1_SW)
                                 || (op1_ID_w == OP1_ADDI)
                                 || (op1_ID_w == OP1_ANDI)
                                 || (op1_ID_w == OP1_ORI)
                                 || (op1_ID_w == OP1_XORI))};
   
    assign new_pc_src_ID_w     = {((op1_ID_w == OP1_BEQ)
                                 || (op1_ID_w == OP1_BLT)
                                 || (op1_ID_w == OP1_BLE)
                                 || (op1_ID_w == OP1_BNE)),
                                 ((op1_ID_w == OP1_BEQ)
                                 || (op1_ID_w == OP1_BLT)
                                 || (op1_ID_w == OP1_BLE)
                                 || (op1_ID_w == OP1_BNE)
                                 || (op1_ID_w == OP1_JAL))};
    
    assign mem_we_ID_w         = (inst_FE != {DBITS{1'b0}}) && ((op1_ID_w == OP1_SW) || ((op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_RSR)));
    
    assign mem_re_ID_w         = (inst_FE != {DBITS{1'b0}}) && (op1_ID_w == OP1_LW);
    
    assign reg_we_ID_w         = (inst_FE != {DBITS{1'b0}}) &&
                                 ((op1_ID_w == OP1_ALUR)
                                 || (op1_ID_w == OP1_JAL)
                                 || (op1_ID_w == OP1_LW)
                                 || (op1_ID_w == OP1_ADDI)
                                 || (op1_ID_w == OP1_ANDI)
                                 || (op1_ID_w == OP1_ORI)
                                 || (op1_ID_w == OP1_XORI))
                                 || ((op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_RSR));
                                 
    assign reg_wr_src_sel_ID_w = {((op1_ID_w == OP1_ALUR)
                                 || (op1_ID_w == OP1_ADDI)
                                 || (op1_ID_w == OP1_ANDI)
                                 || (op1_ID_w == OP1_ORI)
                                 || (op1_ID_w == OP1_XORI)),
                                 (op1_ID_w == OP1_LW)
                                 || ((op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_WSR))
                                 || ((op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_RSR))};
    
    assign reg_wr_dst_sel_ID_w = (inst_FE != {DBITS{1'b0}}) && ((op1_ID_w == OP1_ALUR)
                                 || ((op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_WSR))
                                 || ((op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_RSR)));

            
    assign rd_mem_ID_w = mem_re_ID_w;
    assign wr_mem_ID_w = mem_we_ID_w;
    assign wr_reg_ID_w = reg_we_ID_w;

    assign ctrlsig_ID_w = {is_br_ID_w, is_jmp_ID_w, rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};   
    
    //DATA FORWARDING
    //if the instruction currently in ID stage reads from the destination register of the instruction currently in execute
    //we need to forward (use "aluout_EX_r" as your read register value)
    
    //1 if the instruction in the ID stage is RSR, we are reading from a system register
    assign rdsysreg_ID_w = (op1_ID_w == OP1_SYS) && (op2_ID_w == OP2_RSR);
    //1 if the instruction in the EX stage is WSR
    assign wrsysreg_EX_w = (op1_ID == OP1_SYS) && (op2_ID == OP2_WSR);
    //1 if the instruction in the MEM stage is WSR
    assign wrsysreg_MEM_w = (op1_EX == OP1_SYS) && (op2_EX == OP2_WSR);
    
    //Special WSR then RSR case
    assign sys_reg_dep = (rs_ID_w != 4'b0000) && (rdsysreg_ID_w)
                         && (((rs_ID_w == dst_reg_ID) && wrsysreg_EX_w) || ((rs_ID_w == dst_reg_EX)) && wrsysreg_MEM_w);
                         
    assign forward_rs_from_EX_sys = sys_reg_dep && ((rs_ID_w != 4'b0000) && (rs_ID_w == dst_reg_ID) && (wrsysreg_EX_w));
    
    assign forward_rs_from_MEM_sys = data_dep_rs && ((rs_ID_w != 4'b0000) && (rs_ID_w == dst_reg_EX) && (wrsysreg_MEM_w));
    

    // 1 if the instruction currently in ID stage uses the Rs of the instruction in EX or MEM as an operand
    // and the instruction in the ID stage is not reading from a system register
    assign data_dep_rs = (rs_ID_w != 4'b0000) && (!rdsysreg_ID_w) && ((rs_ID_w == dst_reg_ID) || (rs_ID_w == dst_reg_EX));
    
    //if there is a data dependency
    //determine where we need to forward from
    assign forward_rs_from_EX = data_dep_rs && ((rs_ID_w != 4'b0000) && (rs_ID_w == dst_reg_ID) && (!wrsysreg_EX_w))
                                && ((op1_ID != OP1_SW) && (op1_ID != OP1_BEQ) && (op1_ID != OP1_BLT)
                                && (op1_ID != OP1_BLE) && (op1_ID != OP1_BNE));
    
    assign forward_rs_from_MEM = data_dep_rs && ((rs_ID_w != 4'b0000) && (rs_ID_w == dst_reg_EX) && (!wrsysreg_MEM_w))
                                 && ((op1_EX != OP1_SW) && (op1_EX != OP1_BEQ) && (op1_EX != OP1_BLT)
                                 && (op1_EX != OP1_BLE) && (op1_EX != OP1_BNE));

    // 1 if the instruction currently in ID stage uses Rt as an operand (need to check because only EXT and BR type instructions uses Rt as an operand)
    assign data_dep_rt_check = (op1_ID_w == OP1_ALUR) || (op1_ID_w == OP1_BEQ) || (op1_ID_w == OP1_BLT) || (op1_ID_w == OP1_BLE) || (op1_ID_w == OP1_BNE) || (op1_ID_w == OP1_SW);

    // 1 if the instruction currently in ID stage uses the Rt of the instruction in EX as an operand
    assign data_dep_rt = (rt_ID_w != 4'b0000) && ((rt_ID_w == dst_reg_ID) || (rt_ID_w == dst_reg_EX));
    
    //determine where we need to forward from
    assign forward_rt_from_EX = data_dep_rt_check && data_dep_rt && ((rt_ID_w != 4'b0000) && (rt_ID_w == dst_reg_ID) && (!wrsysreg_EX_w))
                                && ((op1_ID != OP1_SW) && (op1_ID != OP1_BEQ) && (op1_ID != OP1_BLT)
                                && (op1_ID != OP1_BLE) && (op1_ID != OP1_BNE));
    
    assign forward_rt_from_MEM = data_dep_rt_check && data_dep_rt && ((rt_ID_w != 4'b0000) && (rt_ID_w == dst_reg_EX) && (!wrsysreg_MEM_w))
                                 && ((op1_EX != OP1_SW) && (op1_EX != OP1_BEQ) && (op1_EX != OP1_BLT)
                                 && (op1_EX != OP1_BLE) && (op1_EX != OP1_BNE));

    // 1 if the instruction currently in ID stage is a LW and we need the value being written to memory from a SW that is currently in EX stage or MEM stage
    assign data_dep_mem = (mem_addr_ID_w != {DBITS{1'b0}}) && (op1_ID_w == OP1_LW && ((op1_ID == OP1_SW && aluout_EX_r == mem_addr_ID_w) || (op1_EX == OP1_SW && aluout_EX == mem_addr_ID_w)));

   //if mem rd, just forward the reg contents
    assign forward_mem_cont_from_EX = data_dep_mem && ((mem_addr_ID_w != {DBITS{1'b0}}) && (op1_ID_w == OP1_LW && (op1_ID == OP1_SW && aluout_EX_r == mem_addr_ID_w)));
    
    assign forward_mem_cont_from_MEM = data_dep_mem && ((mem_addr_ID_w != {DBITS{1'b0}}) && (op1_ID_w == OP1_LW && (op1_EX == OP1_SW && aluout_EX == mem_addr_ID_w)));
    
    //CASES WHERE WE STILL HAVE TO STALL
    //WE HAVE TO WAIT FOR INFO TO BE FORWARDED FROM MEM STAGE
    //if there is a data dependency (as above) AND the instruction in EX is a LW
    assign stall_lw_EX = (forward_rs_from_EX || forward_rt_from_EX) && (op1_ID == OP1_LW);
    
    assign forward_lw_to_rs_from_MEM = forward_rs_from_MEM && (op1_EX == OP1_LW);

    assign forward_lw_to_rt_from_MEM = forward_rt_from_MEM && (op1_EX == OP1_LW);
   
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
            rs_spec_ID        <= {REGNOBITS{1'b0}}; //RsSpec
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
        end else if (is_nop_FE || stall_lw_EX) begin
            PC_ID      <= {DBITS{1'b0}};
            rt_spec_ID <= {REGNOBITS{1'b0}}; //RtSpec
            regval2_ID <= {DBITS{1'b0}}; //RtCont
            regval1_ID <= {DBITS{1'b0}}; //RsCont
            sxt_imm_ID <= {DBITS{1'b0}}; //sxtImm

            rd_spec_ID        <= {REGNOBITS{1'b0}}; //RdSpec
            rs_spec_ID        <= {REGNOBITS{1'b0}}; //RsSpec
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
            rs_spec_ID        <= {REGNOBITS{1'b0}}; //RsSpec
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
            rs_spec_ID        <= rs_ID_w; //RsSpec
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
    wire [OP1BITS-1:0] op1_EX_w;
    wire [OP2BITS-1:0] op2_EX_w;

    wire br_cond_EX;
    // Note that aluout_EX_r is declared as reg, but it is output signal from combi logic
    wire signed [DBITS-1:0] alu_in_EX_r;

    reg [DBITS-1:0] PC_EX;
    reg [0:0] mem_we_EX;
    reg [0:0] mem_re_EX;
    reg [0:0] reg_we_EX;
    reg [1:0] reg_wr_src_sel_EX;
    reg [REGNOBITS-1:0] rs_spec_EX; //RsSpec

    
    assign op1_EX_w = op1_ID;
    assign op2_EX_w = op2_ID;

    assign alu_in_EX_r = (alu_src_ID == 00) ? regval2_ID : //take RtCont
                        (alu_src_ID == 01) ? sxt_imm_ID : //take sxtImm
                        sxt_imm_ID << 2; //take sxtImm x 4
                        //there should never be a case where alu_src_ID == 11

    assign br_cond_EX = (op1_EX_w == OP1_BEQ) ? (regval1_ID == regval2_ID) :
                        (op1_EX_w == OP1_BLT) ? (regval1_ID < regval2_ID)  :
                        (op1_EX_w == OP1_BLE) ? (regval1_ID <= regval2_ID) :
                        (op1_EX_w == OP1_BNE) ? (regval1_ID != regval2_ID) :
                        1'b0;
    
    assign aluout_EX_r = (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_EQ) ? {31'b0, regval1_ID == alu_in_EX_r} :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_LT) ? {31'b0, regval1_ID < alu_in_EX_r} :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_LE) ? {31'b0, regval1_ID <= alu_in_EX_r} :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_NE) ? {31'b0, regval1_ID != alu_in_EX_r} :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_ADD) ? (regval1_ID + alu_in_EX_r) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_AND) ? (regval1_ID & alu_in_EX_r) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_OR) ? (regval1_ID | alu_in_EX_r) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_XOR) ? (regval1_ID ^ alu_in_EX_r) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_SUB) ? (regval1_ID - alu_in_EX_r) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_NAND) ? (~(regval1_ID & alu_in_EX_r)) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_NOR) ? (~(regval1_ID | alu_in_EX_r)) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_NXOR) ? (~(regval1_ID ^ alu_in_EX_r)) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_LSHF) ? (regval1_ID << alu_in_EX_r) :
                         (op1_EX_w == OP1_ALUR && op2_EX_w == OP2_RSHF) ? (regval1_ID >>> alu_in_EX_r) :
                         (op1_EX_w == OP1_LW || op1_EX_w == OP1_SW || op1_EX_w == OP1_ADDI) ? (regval1_ID + alu_in_EX_r) :
                         (op1_EX_w == OP1_ANDI) ? (regval1_ID & alu_in_EX_r) :
                         (op1_EX_w == OP1_ORI) ? (regval1_ID | alu_in_EX_r) :
                         (op1_EX_w == OP1_XORI) ? (regval1_ID ^ alu_in_EX_r) :
                         
                         ((op1_EX_w == OP1_SYS) && (op2_EX_w == OP2_RSR)) ? regval1_ID : 
                         ((op1_EX_w == OP1_SYS) && (op2_EX_w == OP2_WSR)) ? regval1_ID :
                         //((op1_EX_w == OP1_SYS) && (op2_EX_w == OP2_RETI)) ?  :
                         {DBITS{1'b0}};

    assign is_br_EX_w = ctrlsig_ID[4];
    assign is_jmp_EX_w = ctrlsig_ID[3];
    assign wr_reg_EX_w = ctrlsig_ID[0];

    assign ctrlsig_EX_w = {rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};

    // Specify signals such as mispred_EX_w, pcgood_EX_w
    //assign mispred_EX_w = br_cond_EX || (op1_ID == OP1_JAL);
    assign mispred_EX_w = br_cond_EX || (op1_ID == OP1_JAL) || ((op1_ID == OP1_SYS) && (op2_ID == OP2_RETI));

    assign pcgood_EX_w = (op1_ID == OP1_JAL) ? (regval1_ID + (sxt_imm_ID << 2)) :
                         ((op1_ID == OP1_SYS) && (op2_ID == OP2_RETI)) ? IRA :
                         (br_cond_EX) ? (PC_ID + (sxt_imm_ID << 2)) :
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
            op2_EX     <= 8'b00000000;
            ctrlsig_EX <= 3'b000;
            inst_EX <= {INSTBITS{1'b0}};
            is_nop_EX <= 1'b0;
            rs_spec_EX <= {REGNOBITS{1'b0}}; //RsSpec
        end else if (is_nop_ID || br_cond_EX) begin
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
            op2_EX     <= 8'b00000000;
            ctrlsig_EX <= 3'b000;
            inst_EX <= {INSTBITS{1'b0}};
            is_nop_EX <= 1'b1;
            rs_spec_EX <= {REGNOBITS{1'b0}}; //RsSpec
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
            op1_EX     <= op1_EX_w;
            op2_EX     <= op2_EX_w;
            ctrlsig_EX <= ctrlsig_EX_w;
            inst_EX <= inst_ID;
            is_nop_EX <= is_nop_ID;
            rs_spec_EX <= rs_spec_ID; //RsSpec
        end
    end


    //*** MEM STAGE ***//
    wire rd_mem_MEM_w;
    wire wr_mem_MEM_w;

    wire [DBITS-1:0]        PC_MEM_w;
    wire                    mem_re_MEM_w;
    wire [DBITS-1:0]        aluout_MEM_w;
    wire [REGNOBITS-1:0]    dst_reg_MEM_w;
    wire                    reg_we_MEM_w;
    wire [1:0]              reg_wr_src_sel_MEM_w;
    wire                    isRSR;

    reg [DBITS-1:0] PC_MEM;
    reg [DBITS-1:0] mem_val_out_MEM;
    reg [DBITS-1:0] aluout_MEM;
    reg reg_we_MEM;
    reg [1:0] reg_wr_src_sel_MEM;
    reg [5:0] op1_MEM;
    reg [7:0] op2_MEM;

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
    assign isRSR = (op1_EX == OP1_SYS) && (op2_EX == OP2_RSR);

    assign rd_mem_MEM_w = ctrlsig_EX[2];
    assign wr_mem_MEM_w = ctrlsig_EX[1];
    assign wr_reg_MEM_w = ctrlsig_EX[0];

    // Read from D-MEM
    assign mem_val_out_MEM_w = //reading from system registers
                               (isRSR && (rs_spec_EX == PCS_reg_ID)) ? PCS :
                               (isRSR && (rs_spec_EX == IHA_reg_ID)) ? IHA :
                               (isRSR && (rs_spec_EX == IRA_reg_ID)) ? IRA :
                               (isRSR && (rs_spec_EX == IDN_reg_ID)) ? IDN :
                               (mem_addr_MEM_w == ADDRLEDR) ? ledr_dbus :
                               (mem_addr_MEM_w == ADDRHEX) ? hex_dbus :
                               ((mem_addr_MEM_w == ADDRKEY) || (mem_addr_MEM_w == ADDRKCTRL)) ? key_dbus :
                               ((mem_addr_MEM_w == ADDRSW) || (mem_addr_MEM_w == ADDRSWCTRL)) ? sw_dbus :
                               ((mem_addr_MEM_w == ADDRTCNT) || (mem_addr_MEM_w == ADDRTLIM)
                                  || (mem_addr_MEM_w == ADDRTCTRL)) ? timer_dbus :
                               dmem[mem_addr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]];

    // Write to D-MEM
    always @ (posedge clk) begin
        if(mem_we_MEM_w) begin
            dmem[mem_addr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]] <= regval2_EX;
        end
    end

    //System register and interrupt handling
    always @ (posedge clk or posedge reset) begin
        if (reset) begin
            PCS <= 2'b0;
            IHA <= {DBITS{1'b0}};
            IRA <= {DBITS{1'b0}};
            IDN <= 3'b0;
        end
        else if (proc_IRQ && pipeline_empty) begin
            //Things we need TODO if we detect an interrupt:
            //Save the next instruction address in IRA
            IRA <= PC_REG;
            
            //Determine which device raised the interrupt and save that device's ID in IDN
            IDN <= device_num;
            
            //Disable interrupts:
                //Copy the IE bit to the OIE bit in the PCS register
                //Then set IE to 0
            PCS[1] <= PCS[0];   //OIE <= IE
            PCS[0] <= 1'b0;     //IE <= 0
        end else if ((op1_EX == OP1_SYS) && (op2_EX == OP2_RETI)) begin
            PCS[0] <= PCS[1];
        end else if ((op1_EX == OP1_SYS) && (op2_EX == OP2_WSR)) begin
            //writing to system registers
            case (dst_reg_MEM_w)
                PCS_reg_ID: PCS <= mem_addr_MEM_w;
                IHA_reg_ID: IHA <= mem_addr_MEM_w;
                IRA_reg_ID: IRA <= mem_addr_MEM_w;
                IDN_reg_ID: IDN <= mem_addr_MEM_w;
            endcase     
        end
    end
    
    


    always @ (posedge clk or posedge reset) begin
        if(reset) begin
            PC_MEM             <= {DBITS{1'b0}};
            mem_val_out_MEM    <= {DBITS{1'b0}};
            aluout_MEM         <= {DBITS{1'b0}};
            dst_reg_MEM        <= {REGNOBITS{1'b0}};
            reg_we_MEM         <= 1'b0;
            reg_wr_src_sel_MEM <= {2{1'b0}};
            ctrlsig_MEM        <= 1'b0;
            inst_MEM           <= {INSTBITS{1'b0}};
            is_nop_MEM         <= 1'b0;
            op1_MEM             <= 6'b000000;
            op2_MEM             <= 8'b00000000;
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
            op1_MEM            <= op1_EX;
            op2_MEM            <= op2_EX;
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
        end else if(reg_we_WB_w) begin
            case (reg_wr_src_sel_WB_w)
                WRITE_PC:       regs[dst_reg_WB_w] <= PC_WB_w;
                WRITE_MEM_DATA: regs[dst_reg_WB_w] <= mem_val_out_WB_w;
                WRITE_ALUOUT:   regs[dst_reg_WB_w] <= aluout_WB_w;
            endcase
        end
    end
endmodule


module LEDR_DEV(ABUS, DBUS, WE, CLK, RESET, LEDR_OUT);
    parameter DBITS    = 32;
    parameter LEDRBITS = 10;
    parameter ADDRLEDR = 32'hFFFFF020;

    input  [DBITS-1:0]     ABUS;
    inout  [DBITS-1:0]     DBUS;
    input                  WE;
    input                  CLK;
    input                  RESET;
    output [LEDRBITS-1:0]  LEDR_OUT;
    
    wire             write_ctrl = WE == 1'b1 && ABUS == ADDRLEDR;
    wire             read_ctrl  = WE == 1'b0 && ABUS == ADDRLEDR;
    reg  [LEDRBITS-1:0] ledr_out;
    
    always @ (posedge CLK or posedge RESET) begin
        if (RESET)
            ledr_out <= {LEDRBITS{1'b0}};
        else if (write_ctrl)
            ledr_out <= DBUS[LEDRBITS-1:0];
    end
    
    // If we want to read, then put the LEDR out value on the data bus;
    // else use high-impedance to allow the processor to drive to data bus
    assign DBUS = read_ctrl ? {{(DBITS-LEDRBITS){1'b0}}, ledr_out} : {DBITS{1'bz}};
    
    assign LEDR_OUT = ledr_out;
endmodule


module HEX_DEV(ABUS, DBUS, WE, CLK, RESET, HEX_OUT);
    parameter DBITS    = 32;
    parameter HEXBITS  = 24;
    parameter ADDRHEX  = 32'hFFFFF000;

    input  [DBITS-1:0]     ABUS;
    inout  [DBITS-1:0]     DBUS;
    input                  WE;
    input                  CLK;
    input                  RESET;
    output [HEXBITS-1:0]   HEX_OUT;
    
    wire               write_ctrl = WE == 1'b1 && ABUS == ADDRHEX;
    wire               read_ctrl  = WE == 1'b0 && ABUS == ADDRHEX;
    reg  [HEXBITS-1:0] hex_out;
    
    always @ (posedge CLK or posedge RESET) begin
        if (RESET)
            hex_out <= 24'hFEDEAD;         
        else if (write_ctrl)
            hex_out <= DBUS[HEXBITS-1:0];
    end
    
    // If we want to read, then put the LEDR out value on the data bus;
    // else use high-impedance to allow the processor to drive to data bus
    assign DBUS = read_ctrl ? {{(DBITS-HEXBITS){1'b0}}, hex_out} : {DBITS{1'bz}};
    
    assign HEX_OUT = hex_out;
endmodule


module KEY_DEV(ABUS, DBUS, WE, CLK, RESET, KEY_IN, KCTRL_OUT, SERVICE_SIG_IN);
    parameter DBITS     = 32;
    parameter KEYBITS   = 4;
    parameter ADDRKDATA = 32'hFFFFF080;
    parameter KCTRLBITS = 3;
    parameter ADDRKCTRL = 32'hFFFFF084;
    
    // These are for indexing into KCTRL
    parameter READYBIT   = 0;  
    parameter OVERRUNBIT = 1;
    parameter IEBIT      = 2;         // Should it be 4??? Who knows???
    
    input  [DBITS-1:0]     ABUS;
    inout  [DBITS-1:0]     DBUS;
    input                  WE;
    input                  CLK;
    input                  RESET;
    input  [KEYBITS-1:0]   KEY_IN;    // The input of the Project module named
                                      // KEY; included as an input to KEY_DEV to
                                      // allow us to implement KEY as a proper
                                      // IO device; whenever the processor needs
                                      // to read the value of the KEY, it should
                                      // check the value from KEY_DEV's DBUS
    output [KCTRLBITS-1:0] KCTRL_OUT;
    input                  SERVICE_SIG_IN;  // Only when 1 we are about to service
                                            // a KEY int; used to reset ready and
                                            // overrun bits
    
    reg  [KEYBITS-1:0]   KDATA_old;   // Holds most recent value of KDATA   
                                      // to allow for detection of changes
    reg  [KEYBITS-1:0]   KDATA;
    reg  [KCTRLBITS-1:0] KCTRL;
    wire                 kctrl_write_ctrl = WE == 1'b1 && ABUS == ADDRKCTRL;
    wire                 kdata_read_ctrl  = WE == 1'b0 && ABUS == ADDRKDATA;
    wire                 kctrl_read_ctrl  = WE == 1'b0 && ABUS == ADDRKCTRL;
    
    always @ (posedge CLK or posedge RESET) begin
        if (RESET) begin
            KDATA_old <= {KEYBITS{1'b0}};
            KDATA     <= {KEYBITS{1'b0}};
            KCTRL     <= 3'b100;
        end else begin
            KDATA_old         <= KDATA;
            KDATA             <= ~KEY_IN;
            KCTRL[READYBIT]   <= ((KDATA != KDATA_old) || (KCTRL[READYBIT] == 1'b1)) && !SERVICE_SIG_IN;
            
            if (SERVICE_SIG_IN)
                KCTRL[OVERRUNBIT] <= 1'b0;
            else if ((kctrl_write_ctrl == 1'b1) && (DBUS[OVERRUNBIT] == 1'b0))
                KCTRL[OVERRUNBIT] <= 1'b0;
            else if ((KDATA != KDATA_old) && (KCTRL[READYBIT] == 1'b1))
                KCTRL[OVERRUNBIT] <= 1'b1;
            else
                KCTRL[OVERRUNBIT] <= KCTRL[OVERRUNBIT];
                
            if (kctrl_write_ctrl == 1'b1)
                KCTRL[IEBIT] <= DBUS[IEBIT];
            else
                KCTRL[IEBIT] <= KCTRL[IEBIT];
        end
    end
    
    assign DBUS      = kdata_read_ctrl ? {{(DBITS-KEYBITS){1'b0}}, KDATA} :
                       kctrl_read_ctrl ? {{(DBITS-KCTRLBITS){1'b0}}, KCTRL} :
                       {DBITS{1'bz}};
    assign KCTRL_OUT = KCTRL;
endmodule


module SW_DEV(ABUS, DBUS, WE, CLK, RESET, SW_IN, SWCTRL_OUT, SERVICE_SIG_IN);
    parameter DBITS      = 32;
    parameter SWBITS     = 10;
    parameter ADDRSWDATA = 32'hFFFFF090;
    parameter SWCTRLBITS = 3;
    parameter ADDRSWCTRL = 32'hFFFFF094;
    
    // These are for indexing into SWCTRL
    parameter READYBIT   = 0;  
    parameter OVERRUNBIT = 1;
    parameter IEBIT      = 2;           // Should it be 4??? Who knows???
    
    // Number of clock cycles (with 50 MHZ clock) in different time units
    parameter ONE_MILLISECOND          = 'd50000;
    parameter TEN_MILLISECONDS         = 'd500000;
    parameter FIFTY_MILLISECONDS = 'd25000000;
    
    input  [DBITS-1:0]      ABUS;
    inout  [DBITS-1:0]      DBUS;
    input                   WE;
    input                   CLK;
    input                   RESET;
    input  [SWBITS-1:0]     SW_IN;      // The input of the Project module named
                                        // SW; included as an input to SW_DEV to
                                        // allow us to implement SW as a proper
                                        // IO device; whenever the processor needs
                                        // to read the value of the SW, it should
                                        // check the value from SW_DEV's DBUS
    output [SWCTRLBITS-1:0] SWCTRL_OUT;
    input                   SERVICE_SIG_IN;
    
    reg  [SWBITS-1:0]     SWDATA_old;   // Holds most recent value of SWDATA   
                                        // to allow for detection of changes
    reg  [SWBITS-1:0]     SWDATA;
    reg  [SWCTRLBITS-1:0] SWCTRL;
    wire                  swctrl_write_ctrl = WE == 1'b1 && ABUS == ADDRSWCTRL;
    wire                  swdata_read_ctrl  = WE == 1'b0 && ABUS == ADDRSWDATA;
    wire                  swctrl_read_ctrl  = WE == 1'b0 && ABUS == ADDRSWCTRL;
    
    // Used for keeping track of when 
    reg [DBITS-1:0]  clock_cycles;
    reg [SWBITS-1:0] SWDATA_temp;
    reg [SWBITS-1:0] SWDATA_temp_old;
    
    always @ (posedge CLK or posedge RESET) begin
        if (RESET) begin
            SWDATA_old   <= {SWBITS{1'b0}};
            SWDATA       <= {SWBITS{1'b0}};
            SWCTRL       <= 3'b100;          // IE bit should be 1 by default
            clock_cycles <= 0;
        end else begin
            if (clock_cycles + 1 >= FIFTY_MILLISECONDS && SWDATA_temp == SWDATA_temp_old) begin
                SWDATA_old         <= SWDATA;
                SWDATA             <= SWDATA_temp;
                SWCTRL[READYBIT]   <= (SWDATA != SWDATA_old) || (SWCTRL[READYBIT] == 1'b1) && !SERVICE_SIG_IN;
                clock_cycles       <= 0;
                SWDATA_temp        <= SW_IN;
                SWDATA_temp_old    <= SW_IN;
            end else if (SWDATA_temp == SWDATA_temp_old) begin
                clock_cycles    <= clock_cycles + 1;
                SWDATA_temp     <= SW_IN;
                SWDATA_temp_old <= SWDATA_temp;
            end else begin
                clock_cycles    <= 0;
                SWDATA_temp     <= SW_IN;
                SWDATA_temp_old <= SW_IN;
            end
            
            if (SERVICE_SIG_IN)
                SWCTRL[OVERRUNBIT] <= 1'b0;
            else if ((swctrl_write_ctrl == 1'b1) && (DBUS[OVERRUNBIT] == 1'b0))
                SWCTRL[OVERRUNBIT] <= 1'b0;
            else if ((SWDATA != SWDATA_old) && (SWCTRL[READYBIT] == 1'b1))
                SWCTRL[OVERRUNBIT] <= 1'b1;
            else
                SWCTRL[OVERRUNBIT] <= SWCTRL[OVERRUNBIT];
                
            if (swctrl_write_ctrl == 1'b1)
                SWCTRL[IEBIT] <= DBUS[IEBIT];
            else
                SWCTRL[IEBIT] <= SWCTRL[IEBIT];
        end
    end
    
    assign DBUS       = swdata_read_ctrl ? {{(DBITS-SWBITS){1'b0}}, SWDATA} :
                        swctrl_read_ctrl ? {{(DBITS-SWCTRLBITS){1'b0}}, SWCTRL} :
                        {DBITS{1'bz}};
    assign SWCTRL_OUT = SWCTRL;
endmodule


module TIMER_DEV(ABUS, DBUS, WE, CLK, RESET, TCTRL_OUT, SERVICE_SIG_IN);
    parameter DBITS     = 32;
    parameter TCNTBITS  = 32;
    parameter TLIMBITS  = 32;
    parameter TCTRLBITS = 3;
    parameter ADDRTCNT  = 32'hFFFFF100;
    parameter ADDRTLIM  = 32'hFFFFF104;
    parameter ADDRTCTRL = 32'hFFFFF108;
    
    // These are for indexing into TCTRL
    parameter READYBIT   = 0;  
    parameter OVERRUNBIT = 1;
    parameter IEBIT      = 2;           // Should it be 4??? Who knows???
    
    // Number of clock cycles (with 50 MHZ clock) in different time units
    parameter ONE_MILLISECOND  = 'd50000;
    
    input  [DBITS-1:0]     ABUS;
    inout  [DBITS-1:0]     DBUS;
    input                  WE;
    input                  CLK;
    input                  RESET;
    output [TCTRLBITS-1:0] TCTRL_OUT;
    input                  SERVICE_SIG_IN;
    
    reg  [TCNTBITS-1:0]  TCNT;
    reg  [TLIMBITS-1:0]  TLIM;
    reg  [TCTRLBITS-1:0] TCTRL;
    wire                 tcnt_write_ctrl  = WE == 1'b1 && ABUS == ADDRTCNT;
    wire                 tcnt_read_ctrl   = WE == 1'b0 && ABUS == ADDRTCNT;
    wire                 tlim_write_ctrl  = WE == 1'b1 && ABUS == ADDRTLIM;
    wire                 tlim_read_ctrl   = WE == 1'b0 && ABUS == ADDRTLIM;
    wire                 tctrl_write_ctrl = WE == 1'b1 && ABUS == ADDRTCTRL;
    wire                 tctrl_read_ctrl  = WE == 1'b0 && ABUS == ADDRTCTRL;
    
    // Used to keep track of when to increment TCNT
    reg [DBITS-1:0] clock_cycles;
    
    always @ (posedge CLK or posedge RESET) begin
        if (RESET) begin
            TCNT         <= {TCNTBITS{1'b0}};
            TLIM         <= {TLIMBITS{1'b0}};
            TCTRL        <= 3'b100;            // IE bit should be 1 by default
            clock_cycles <= {DBITS-1{1'b0}};
        end else begin
            if (tcnt_write_ctrl) begin
                TCNT         <= DBUS;
                clock_cycles <= 0;
            end else if ((TLIM != 0) && (TCNT >= (TLIM - 1)) && (clock_cycles >= (ONE_MILLISECOND - 1))) begin
                TCNT         <= 0;
                clock_cycles <= 0;
            end else if (clock_cycles >= (ONE_MILLISECOND - 1)) begin
                TCNT            <= TCNT + 1;
                clock_cycles    <= 0;
            end else
                clock_cycles <= clock_cycles + 1;
                
            TLIM <= tlim_write_ctrl ? DBUS : TLIM;
            
            if ((tctrl_write_ctrl == 1'b1) && (DBUS[READYBIT] == 1'b0) || SERVICE_SIG_IN)
                TCTRL[READYBIT] <= 1'b0;
            else if ((TLIM != 0) && (TCNT >= (TLIM - 1)) && (clock_cycles >= (ONE_MILLISECOND - 1)))
                TCTRL[READYBIT] <= 1'b1;
            else
                TCTRL[READYBIT] <= TCTRL[READYBIT];
            
            if (SERVICE_SIG_IN)
                TCTRL[OVERRUNBIT] <= 1'b0;
            else if ((tctrl_write_ctrl == 1'b1) && (DBUS[OVERRUNBIT] == 1'b0))
                TCTRL[OVERRUNBIT] <= 1'b0;
            else if ((TLIM != 0) && (TCNT == (TLIM - 1)) && (clock_cycles >= (ONE_MILLISECOND - 1)) && (TCTRL[READYBIT] == 1'b1))
                TCTRL[OVERRUNBIT] <= 1'b1;
            else
                TCTRL[OVERRUNBIT] <= TCTRL[OVERRUNBIT];
                
            if (tctrl_write_ctrl == 1'b1)
                TCTRL[IEBIT] <= DBUS[IEBIT];
            else
                TCTRL[IEBIT] <= TCTRL[IEBIT];
        end
    end
    
    assign DBUS      = tcnt_read_ctrl ? {{(DBITS-TCNTBITS){1'b0}}, TCNT} :
                       tlim_read_ctrl ? {{(DBITS-TLIMBITS){1'b0}}, TLIM} :
                       tctrl_read_ctrl ? {{(DBITS-TCTRLBITS){1'b0}}, TCTRL} :
                       {DBITS{1'bz}};
    assign TCTRL_OUT = TCTRL;
endmodule


module SXT(IN, OUT);
    parameter IBITS = 16;
    parameter OBITS = 32;

    input  [IBITS-1:0] IN;
    output [OBITS-1:0] OUT;

    assign OUT = {{(OBITS-IBITS){IN[IBITS-1]}}, IN};
endmodule