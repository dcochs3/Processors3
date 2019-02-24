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