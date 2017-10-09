;========================================================================
;  Assembler key Pad handler
;========================================================================
;

;--- register definitions for ATMEGA32.
.include "c:\VMLab\include\M32DEF.INC"   ;**warning** you will ned to change this line

.DEF  ZERO   = R0;  ;register for zero value
.DEF  ONE  = R1;   ;register for one value

.DEF  TEMP  =R16  ;temp working register

.DEF  SER_REG = R17 ; register for storing the ASCII value to be sent
.DEF  SER_REG1 = R18; register for sending the ASCII value
.DEF  DELAYL = R19  ; 24 bit counter to generate long delays
.DEF  DELAYM = R20  ;
.DEF  DELAYH = R21  ;
.DEF  DIG_ONE =R22 ;storage for the first digit
.DEF  DIG_TWO = R23 ;storage for the second digit
.DEF  DIG_ANS = R24 ;storage for the answer
.DEF  TEN = R25 ; register for multiplying by 10
.DEF  COUNT=R26

.EQU  B192 = 205;

.CSEG

;------ Reset and interrupt vectors ------------------------------
;
.ORG  0x00
START:
   rjmp  RESET  ; Reset vector
   reti
   reti
   reti
   reti
   reti
   reti
   reti
   reti
   reti
   reti
   reti
   reti
   reti
   reti


;------ Device initialization -------------------------------------

.ORG  0x20
;------ initializations.
RESET:
	clr	R0	
	clr  COUNT
	clr   ZERO
   clr   ONE
   dec   ONE                     ;0xFF			  			; use as zero value
	clr   DIG_ONE               ;clear storage registers
   clr   DIG_TWO
   clr   DIG_ANS
   ldi   TEN, $0A              ;set register to 1010
	
   ldi   TEMP, low(RAMEND)   ; Init stack pointer
   out   SPL,  TEMP    			; RAMEND defined in M32DEF.INC
   ldi	TEMP, high(RAMEND)
   out	SPH,  TEMP

;**include code to do the following**
   ldi	TEMP, $FF	   ; set PORTB to output.
   out	DDRB, TEMP
   out	PortB, TEMP
   call  Delay
   call  Delay
   clr	TEMP
   out	PortB, TEMP
;  now set up the keypad port. This assumes that the keypad is configured with
; C0,2,4 are outputs to columns; R1, 3, 5, 6 are input, C7 not used
	ldi	TEMP, $15     ; 00010101
   out   DDRC, TEMP    ; set up keypad ports
   ldi	TEMP, $FF
   out	PORTC, temp   ;
;------ drops through to here - forever loop.
LOOP:
   ldi  DELAYH, 9    ; ~130mS delay
   call L_DELAY
   call ReadKP          ; go and look for a key - won't return until one is pressed
   call Convert         ; if pressed, convert to hex
   call STORE
   call CALC
   call Display         ; display it on the LEDs
   rjmp LOOP				; back to do it again


;----convert keypad presses into digits and stores it into registers---
STORE:
	cpi COUNT,5
	breq clear
	
	cpi TEMP,$A
	breq clear
	
	inc COUNT
	
	cpi COUNT,1
	breq digit_one
	
	cpi COUNT,2
	breq digit_two
	
	cpi COUNT,3
	breq digit_three
	
	cpi COUNT,4
	breq digit_four
	
	clr COUNT
	ret
	
digit_one:
	mov DIG_ONE,Temp
	mul DIG_ONE,TEN
	mov DIG_ONE,ZERO
	clr ZERO
	clr ONE
	dec ONE
	ret

digit_two:
	add  DIG_ONE, TEMP
	ret
	
digit_three:
	mov DIG_TWO, TEMP
	mul DIG_TWO, TEN
	mov DIG_TWO,ZERO
	clr ZERO
	clr ONE
	dec ONE
	ret
	
digit_four:
	add  DIG_TWO, TEMP
	ret

clear:
	clr COUNT
	out PORTB,ZERO
	cpi TEMP,$A
	breq jmp_loop
	cpi TEMP,$A
	brne jmp_store

jmp_store:
	rjmp STORE
jmp_loop:
	rjmp LOOP

CALC:
	cpi COUNT,4
	brsh sum
	;implement code here to display final sum?
	;or implement code inside LCD subroutine
	ret
	
sum:
	mov DIG_ANS, DIG_ONE
	add DIG_ANS, DIG_TWO

DISPLAY:
   cpi COUNT,4
   breq final
   out PORTB,Temp
   rjmp LOOP
final:
	out PORTB,DIG_ANS
	inc COUNT
	rjmp LOOP
; ************************************
; Key pad routine
; input: none
; returns: R16 (TEMP) Keypad Scan code
; assumes:
;	1. that a 12-key keypad is connected to Port C0..6 with line 1 -> C0 etc.
;	2. C0,2,4 are outputs to columns; R1, 3, 5, 6 are input, C7 not used
;	3. DDRC has already been set up in the initialisation routine and pull-ups are on
;*************************************
.EQU  col1=$FB          ;$FB: Bit 2 = 0; this is the 1st column
.EQU  col2=$FE          ;$FE: Bit 0 = 0; col 2
.EQU  col3=$EF          ;$EF: Bit 4 = 0; col 3
;*************************************
ReadKP:
   ldi	TEMP, col1     ;
	rcall ReadOne_Col    ; read one column
	cpi	TEMP, col1	   ; returns same value if no key pressed
	brne  ExitKP         ; if not equal then some key has been pressed
; column 2
   ldi	TEMP, col2     ;
	rcall ReadOne_Col    ; read one column
	cpi	TEMP, col2	   ; returns same value if no key pressed
	brne  ExitKP         ; if not equal then some key has been pressed
; column 3
   ldi	TEMP, col3     ;
	rcall ReadOne_Col    ; read one column
	cpi	TEMP, col3	   ; returns same value if no key pressed
	brne  ExitKP         ; if not equal then some key has been pressed
;
   rjmp	ReadKP        ; keep doing it until a key is pressed

ExitKP:
	ret

ReadOne_Col:
;**************************************
; routine reads one column
; input R16 (TEMP) with a zero value in column to be read
; returns R16 (TEMP) value from port C (keypad)
; assumes - if no key pressed, same value returned
; this is true as reading back a pin set as output
; will return the port register value set previously
;**************************************
;
   OUT	PORTC, TEMP
   NOP						;delay to let the outputs settle
   NOP                  ;250nS
   NOP
   IN	 TEMP, PINC
	ret

Convert:
;**************************************
; convert routine
; input: R16 (TEMP) scan code from keypad routine
; output R16 (TEMP) equivalent hex value
; uses: Z reg
; assume - we only get to here if a key has been pressed
; we'll use Z as a pointer to the entries in the convert table
;**************************************
;***last time we looked, this works.  Your job is to change it
; and come up with another routine that works the same.
   ldi	 ZH, high(Tble<<1)    ; set the base pointer to the table
   ldi	 ZL, low(Tble<<1)
   andi   TEMP, $7F 				; kill off bit 7, make sure it's zero
   add	 ZL,TEMP					; use TEMP as an offset to the table pointer
   adc	 ZH, R0					; R0=0 increment ZH if carry out of ZL
   lpm	 TEMP, Z	   			; now pick up the table entry
	ret                        ; returns with TEMP = hex value (or 0xFF if error)
	
; table to convert from scan code to whatever.  This time it's hex.
; There are 128 entries in the table, only 12 of which are actually used
; this is a fairly typical tradeoff - memory size for processing speed
; scan codes as follows:
; '0'->$76; '1'->$79; '2'->$7C; '3'->$6D; '4'->$3B; '5'->$3E; '6'->$2F; '7'->$5B
; '8'->$5E; '9'->$4F; '*'->$73; '#'->$67
; $F used as 'error' code (incorrect or unrecognized key code)

Tble:   ;0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
   .DB  $F, $1, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F      ;0x
   .DB  $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F      ;1x
   .DB  $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $6      ;2x
   .DB  $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $4, $F, $F, $5, $F      ;3x
   .DB  $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $9      ;4x
   .DB  $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $F, $7, $F, $F, $8, $F      ;5x
   .DB  $F, $F, $F, $F, $F, $F, $F, $A, $F, $F, $F, $F, $F, $3, $F, $F      ;6x
   .DB  $F, $F, $F, $A, $F, $F, $0, $F, $F, $1, $F, $F, $2, $F, $F, $F      ;7x

;*************************************************
;Display routine
; input: R16 (TEMP) = hex value to be displayed
; returns: nothing
; uses: R16
;*************************************************
;*************************************
;

;--------------- Baud-rate DELAY ------------------------------------------
;
; PURPOSE: 8 bit counter for very short delays.
;
; DETAILS:  total delay = 6+ 3x(Delay-1) x 83.3uS @ 12 MHz
;  Enters with delay value in DELAYL
; e.g 207 -> 5.2uS ->  19,230 baud
B_DELAY:
; 	ret                        ; uncomment this line for simulation to
   LDI   DELAYL, B192
BD_0:
   dec   DELAYL              ; Start at 0xFF
   brne  BD_0                 ; inner loop 3 cycles =
   LDI   DELAYL, B192
BD_1:
   dec   DELAYL              ; Start at 0xFF
   brne  BD_1                 ; inner loop 3 cycles =
   ret                       ; 4 cycles
;--------------- Short DELAY ------------------------------------------
;
; PURPOSE: 8 bit counter for very short delays.
;
; DETAILS:  total delay = 6+ 3x(Delay-1) x 83.3uS @ 12 MHz
;  Enters with delay value in DELAYL
; e.g 207 -> 5.2uS ->  19,230 baud
S_DELAY:
; ret                        ; uncomment this line for simulation to
SD_0:
   dec   DELAYL              ; Start at 0xFF
   brne  SD_0                 ; inner loop 3 cycles =
   ret                       ; 4 cycles

;--------------- Long DELAY ------------------------------------------
;
; PURPOSE: 24 bit counter for long delays.
;
; DETAILS:  L and M loops about 256 * 256 * 3 cycles / clock MHz.
;           very approx 16.4 ms @ 12 MHz
;
L_DELAY:
; ret                        ; uncomment this line for simulation to
                             ; eliminate very long delays.  Comment out
                             ; to run on real hardware to get 260ms delay.
   clr   DELAYM
   clr   DELAYL
D_0:
   dec   DELAYL              ; Start at 0xFF
   brne  D_0                 ; inner loop 3 cycles. ;branches once delayl=0
   dec   DELAYM              ;
   brne  D_0                 ; middle loop
   dec   DELAYH
   brne  D_0                 ;outer loop.

   ret

;--------------- Long DELAY ------------------------------------------
;
; PURPOSE: 24 bit counter for long delays.
;
; DETAILS:  L and M loops about 256 * 256 * 3 cycles / clock MHz.
;           very approx 16.4 ms @ 12 MHz
;
;
DELAY_96:
; ret                        ; uncomment this line for simulation to
                             ; eliminate very long delays.  Comment out
                             ; to run on real hardware to get 260ms delay.
   clr   DELAYL
   ldi   DELAYH, 5           ;about 9600 baud
D_96:
   dec   DELAYL              ; Start at 0xFF
   brne  D_96                 ; inner loop 3 cycles.
   dec   DELAYH
   brne  D_0                 ;outer loop.

   ret


;*************************************
;
; Delay routine
; Input: none
; outputs: none
; uses: nothing (saves registers and restores)
;
; this has an inner loop and an outer loop.  The delay is approximately
; equal to 256*256*number of inner loop instruction cycles (4) (~21mS)
; You can vary this by changing the initial values in the registers.
; If you need a much longer delay change one of the loop counters
; to a 16-bit register such as X or Y.
;
;*************************************
Delay:   ret
         PUSH R16			; save R16 and 17 as we're going to use them
         PUSH R17       ; as loop counters
         PUSH R0        ; we'll also use R0 as a zero value for compare
         CLR R0
         CLR R16        ; init inner counter
         CLR R17        ; and outer counter
L1:      DEC R16         ; counts down from 0 to FF to 0
			CPSE R16, R0    ; equal to zero?
			RJMP L1			 ; if not, do it again
			CLR R16			 ; reinit inner counter
L2:      DEC R17
         CPSE R17, R0    ; is it zero yet?
         RJMP L1			 ; back to inner counter
;
         POP R0          ; done, clean up and return
         POP R17
         POP R16
         RET

.exit

















