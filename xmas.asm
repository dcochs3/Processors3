; Addresses for I/O
.NAME   HEX    = 0xFFFFF000
.NAME   LEDR   = 0xFFFFF020
.NAME   KEY    = 0xFFFFF080
.NAME   KCTRL  = 0xFFFFF084
.NAME   SW     = 0xFFFFF090
.NAME   SWCTRL = 0xFFFFF094
.NAME   TCNT   = 0xFFFFF100
.NAME   TLIM   = 0xFFFFF104
.NAME   TCTRL  = 0xFFFFF108

; Memory addresses to use for variables
.NAME   CurrState     = 0xFFFFE000
.NAME   CurrCycle     = 0xFFFFE010
.NAME   HexShiftAmt   = 0xFFFFE020    ; This is used so that when there is key
                                      ; press and the blink speed changes, the
                                      ; key interrupt handler knows which
                                      ; portion of the hex display to use for
                                      ; the new speed

; Blink speed constants (in milliseconds)
.NAME   QuarterSecond            = 250
.NAME   HalfSecond               = 500
.NAME   TwoSeconds               = 2000

; LEDR light constants
.NAME   LightsUpper = 992  ; Binary: 1111100000
.NAME   LightsLower = 31   ; Binary: 0000011111        
.NAME   LightsOff   = 0    ; Binary: 0000000000

; State constants
.NAME   UpperState = 0  ; Binary: 00
.NAME   LowerState = 1  ; Binary: 01
.NAME   AltState   = 2  ; Binary: 10

; Cycle constants
.NAME   Cycle1 = 0  ; Binary: 00
.NAME   Cycle2 = 1  ; Binary: 01
.NAME   Cycle3 = 2  ; Binary: 10

; Device ID numbers
.NAME   TimerIDN  = 1
.NAME   KeyIDN    = 2
.NAME   SwitchIDN = 3

; Key constants
.NAME   Key0 = 1
.NAME   Key1 = 2


; -----------------------------------------------------------------
; Initialization and Main Loop

    .ORG 0x100

    XOR   zero, zero, zero  ; zero out zero register

    SW    zero, CurrStateVal(zero)  ; initialize state machine variables
    SW    zero, CurrCycleVal(zero)

    ADDI  zero, a0, 0x300    ; put 0x300 in IHA      
    WSR   IHA, a0

    ADDI  zero, a0, HalfSecond  ; put 1/2 second in TLIM
    SW    a0, TLIM(zero)

    ADDI  zero, a0, 0x000002    ; also display 2 on HEX0
    SW    a0, HEX(zero)

    SW    zero, TCNT(zero)  ; reset TCNT

    ADDI  zero, a0, 0x01    ; put 0x01 in PCS (set IE bit)
    WSR   PCS, a0

InfiniteLoop:
    BR InfiniteLoop  ; loop here forever waiting for interrupts


; -----------------------------------------------------------------
; Interrupt Handler Address- From here, we read the IDN register
; to determine who caused the interrupt and then jump to the
; appropriate handler code

    .ORG 0x300

    RSR   t0, IDN    ; read the device ID of the device that
                     ; caused the interrupt

    ADDI  zero, t1, TimerIDN       ; check to see if it's the timer
    BEQ   t0, t1, TimerIntHandler

    ADDI  zero, t1, KeyIDN         ; check to see if it's the keys
    BEQ   t0, t1, KeyIntHandler

    ADDI  zero, t1, SwitchIDN
    BEQ   t0, t1, SwitchIntHandler ; check to see if it's the switches

    RETI


; -----------------------------------------------------------------
; Timer Interrupt Handler

TimerIntHandler:
    LW    t0, CurrStateVal(zero)  ; load state machine variables
    LW    t1, CurrCycleVal(zero)

    ADDI  zero, s0, Cycle3        ; if we haven't gone through all 3 cycles,
                                  ; stay in same state and keep blinking
    BLE   t1, s0, Blink

    ADDI  t0, t0, 1               ; else, increment the state variable and
    XOR   t1, t1, t1              ; and reset the cycle variable
    SW    t0, CurrStateVal(zero)
    SW    t1, CurrCycleVal(zero)

    ADDI  zero, s1, AltState      ; if we haven't gone through all 3 states,
                                  ; start blinking in next state
    BLE   t0, s1, Blink

    XOR   t0, t0, t0              ; else, reset the state variable before
    SW    t0, CurrStateVal(zero)  ; continuing
    BR    Blink

Blink:
    ADDI  zero, s0, UpperState   ; determine which state we are in
    BEQ   t0, s0, BlinkUpper     ; and branch to the right code

    ADDI  zero, s0, LowerState
    BEQ   t0, s0, BlinkLower

    ADDI  zero, s0, AltState
    BEQ   t0, s0, BlinkAlt

BlinkUpper:
    LW    a0, LEDR(zero)
    ADDI  zero, a1, LightsUpper
    BNE   a0, a1, BlinkUpperOn

BlinkUpperOff:
    SW    zero, LEDR(zero)  ; turn the lights off
    ADDI  t1, t1, 1              ; increment the cycle value and RETI
    SW    t1, CurrCycleVal(zero)
    RETI

BlinkUpperOn:
    ADDI  zero, a0, LightsUpper  ; turn the upper lights on
    SW    a0, LEDR(zero)
    RETI

BlinkLower:
    LW    a0, LEDR(zero)
    ADDI  zero, a1, LightsLower
    BNE   a0, a1, BlinkLowerOn

BlinkLowerOff:
    SW    zero, LEDR(zero)       ; turn the lights off
    ADDI  t1, t1, 1              ; increment the cycle value and RETI
    SW    t1, CurrCycleVal(zero)
    RETI

BlinkLowerOn:
    ADDI  zero, a0, LightsLower  ; turn the upper lights on
    SW    a0, LEDR(zero)
    RETI

BlinkAlt:
    LW    a0, LEDR(zero)
    ADDI  zero, a1, LightsUpper
    BNE   a0, a1, BlinkAltOn

BlinkAltOff:
    ADDI  zero, a0, LightsLower  ; turn the lower lights on
    SW    a0, LEDR(zero)
    ADDI  t1, t1, 1              ; increment the cycle value and RETI
    SW    t1, CurrCycleVal(zero)
    RETI

BlinkAltOn:
    ADDI  zero, a0, LightsUpper  ; turn the upper lights on
    SW    a0, LEDR(zero)
    RETI


; -----------------------------------------------------------------
; Key Interrupt Handler

KeyIntHandler:
    LW    t0, KEY(zero)      ; load the current key input into t0

    ADDI  zero, t1, Key0     ; see if it's key 0
    BEQ   t0, t1, Key0Press

    ADDI  zero, t1, Key1
    BEQ   t0, t1, Key1Press

    RETI                     ; if it was neither key 0 or key 1,
                             ; just ignore this interrupt and
                             ; RETI

Key0Press:
    LW    t0, TLIM(zero)                ; load current TLIM value and make
    ADDI  zero, t1, TwoSeconds          ; sure that it isn't already the
    BLE   t0, t1, Key0PressChangeSpeed  ; maximum value (2 seconds)
    RETI                                ; just RETI if maximum value 

Key0PressChangeSpeed:
    ADDI  t0, t0, QuarterSecond
    SW    t0, TLIM(zero)
    BR    KeyUpdateHex


Key1Press:
    LW    t0, TLIM(zero)                ; load current TLIM value and make
    ADDI  zero, t1, QuarterSecond       ; sure that it isn't already the
    BGT   t0, t1, Key1PressChangeSpeed  ; minimum value (1/4 second)
    RETI                                ; just RETI if minimum value 

Key1PressChangeSpeed:
    SUBI  t0, t0, 250                   ; just used 250 because
                                        ; using QuarterSecond
                                        ; made assembler not work
    SW    t0, TLIM(zero)
    BR    KeyUpdateHex

KeyUpdateHex:
    CALL  GetSpeed(zero)
    ADD   t0, rv, zero
    LW    t1, HexShiftAmtVal(zero)
    LSHF  t0, t0, t1
    SW    t0, HEX(zero)
    RETI


; -----------------------------------------------------------------
; Switch Interrupt Handler

SwitchIntHandler:
    LW    a0, SW(zero)  ; load the current switch input into t0

    ADD   t0, a0, zero
    BEQ   t0, zero, DisplayHex0  ; return to default if no switches
                                 ; are pressed

    ADD   t0, a0, zero
    ANDI  t0, t0, 0x1            ; if switch 0 is pressed, same thing
    ADDI  zero, t1, 0x1
    BEQ   t0, t1, DisplayHex0

    ADD   t0, a0, zero
    ANDI  t0, t0, 0x2            ; else if switch 1 is pressed,
    ADDI  zero, t1, 0x2          ; use HEX1
    BEQ   t0, t1, DisplayHex1

    ADD   t0, a0, zero
    ANDI  t0, t0, 0x4            ; else if switch 2 is pressed...
    ADDI  zero, t1, 0x4
    BEQ   t0, t1, DisplayHex2

    ADD   t0, a0, zero
    ANDI  t0, t0, 0x8            ; else if switch 3 is pressed...
    ADDI  zero, t1, 0x8
    BEQ   t0, t1, DisplayHex3

    ADD   t0, a0, zero
    ANDI  t0, t0, 0x10           ; else if switch 4 is pressed...
    ADDI  zero, t1, 0x10
    BEQ   t0, t1, DisplayHex4

    ADD   t0, a0, zero
    ANDI  t0, t0, 0x20           ; else if switch 5 is pressed...
    ADDI  zero, t1, 0x20
    BEQ   t0, t1, DisplayHex5

    RETI                         ; else, RETI

DisplayHex0:
    CALL  GetSpeed(zero)         ; display the number for current
    ADD   t0, rv, zero           ; speed on HEX0
    SW    zero, HexShiftAmtVal(zero)
    SW    t0, HEX(zero)
    RETI

DisplayHex1:
    CALL  GetSpeed(zero)         ; display the number for current
    ADD   t0, rv, zero           ; speed on HEX1
    ADDI  zero, t1, 4
    SW    t1, HexShiftAmtVal(zero)
    LSHF  t0, t0, t1
    SW    t0, HEX(zero)
    RETI

DisplayHex2:
    CALL  GetSpeed(zero)         ; display the number for current
    ADD   t0, rv, zero           ; speed on HEX2
    ADDI  zero, t1, 8
    SW    t1, HexShiftAmtVal(zero)
    LSHF  t0, t0, t1
    SW    t0, HEX(zero)
    RETI

DisplayHex3:
    CALL  GetSpeed(zero)         ; display the number for current
    ADD   t0, rv, zero           ; speed on HEX3
    ADDI  zero, t1, 12
    SW    t1, HexShiftAmtVal(zero)
    LSHF  t0, t0, t1
    SW    t0, HEX(zero)
    RETI

DisplayHex4:
    CALL  GetSpeed(zero)         ; display the number for current
    ADD   t0, rv, zero           ; speed on HEX4
    ADDI  zero, t1, 16
    SW    t1, HexShiftAmtVal(zero)
    LSHF  t0, t0, t1
    SW    t0, HEX(zero)
    RETI

DisplayHex5:
    CALL  GetSpeed(zero)         ; display the number for current
    ADD   t0, rv, zero           ; speed on HEX5
    ADDI  zero, t1, 20
    SW    t1, HexShiftAmtVal(zero)
    LSHF  t0, t0, t1
    SW    t0, HEX(zero)
    RETI


; A subroutine to determine what number to display on the hex
; based on the current blink speed; returns the number in a3/rv
GetSpeed:
    LW    t0, TLIM(zero)           ; load the current blink speed into t0

    ADDI  zero, t1, QuarterSecond  ; if it's 1/4 second, return 1
    BEQ   t0, t1, GetSpeed1

    ADDI  t1, t1, QuarterSecond    ; else if it's 1/2 second, return 2
    BEQ   t0, t1, GetSpeed2

    ADDI  t1, t1, QuarterSecond    ; else if it's 3/4 second, return 3
    BEQ   t0, t1, GetSpeed3

    ADDI  t1, t1, QuarterSecond    ; else if it's 1 second, return 4
    BEQ   t0, t1, GetSpeed4

    ADDI  t1, t1, QuarterSecond    ; else if it's 1 1/4 second, return 5
    BEQ   t0, t1, GetSpeed5

    ADDI  t1, t1, QuarterSecond    ; else if it's 1 1/2 second, return 6
    BEQ   t0, t1, GetSpeed6

    ADDI  t1, t1, QuarterSecond    ; else if it's 1 3/4 second, return 7
    BEQ   t0, t1, GetSpeed7

    ADDI  t1, t1, QuarterSecond    ; else if it's 2 seconds, return 8
    BEQ   t0, t1, GetSpeed8

    RETI                           ; should never get here, but
                                   ; RETI to be safe

GetSpeed1:
    ADDI  zero, rv, 1
    RET

GetSpeed2:
    ADDI  zero, rv, 2
    RET

GetSpeed3:
    ADDI  zero, rv, 3
    RET

GetSpeed4:
    ADDI  zero, rv, 4
    RET

GetSpeed5:
    ADDI  zero, rv, 5
    RET

GetSpeed6:
    ADDI  zero, rv, 6
    RET

GetSpeed7:
    ADDI  zero, rv, 7
    RET

GetSpeed8:
    ADDI  zero, rv, 8
    RET


; -----------------------------------------------------------------
; Variable Initialization

CurrStateVal:
    .WORD CurrState
CurrCycleVal:
    .WORD CurrCycle
HexShiftAmtVal:
    .WORD HexShiftAmt
