.include "m16def.inc"
; Program Description:
; 
; This program measures the motor rpm and input voltage
; while driving it using Phase Correct PWM
; The increments on the duty cycle are done AUTOMATICALLY
; every 3sec we increase the speed
; from 20% (starting value) up to 70% (final value)
; when we reack 70% we stay there for 10s
; and every 3sec after that we decrease the speed
; from 70% (starting value) all the way down to 0% (final value) 
; the samples are taken by measuring the number of pulses over a ~0.5s intervals
; the ~0.5 stopwatch is implemented using timer0 and overflow interrupts
; the pulse counting is implemented using timer1 and input capture interrupts (PD6)
; the pcpwm is implemented using timer2. (PD7)
; Therefore, the output pin is on PD7 instead of PD5.

; How to connect the pins:

; LEDS ON PORTB - ALL PORTS - OUTPUT
; SWS ON PORTC - ALL PORTS - INPUT
; PWM ON PD7 - OUTPUT
; 7414 ON PD6 - INPUT

.def intervalOf4Pulses = r15
.def outer_count = r19 ;for delay loop
.def mid_count = r20   ;for delay loop
.def inner_count = r21 ;for delay loop
.def samples = r22     ;for counting samples
.def pulses = r23 	   ;for counting pulse count overflows

; reset interrupt handler
.org 0x0000
 	rjmp RESET

; timer 1 event capture interrupt handler
.org 0x000A
	rjmp TIMER1_CAPT

; main program
.org 0x0100

RESET:

	ldi r16, high(RAMEND) 			; main program start
	out SPH, r16					; set stack pointer to top of RAM
	ldi r16, low(RAMEND)	
	out SPL, r16
	
	clr r16
	out DDRA, r16					; set PORTA as input for the adc
	out DDRC, r16					; set PORTC as sw0-sw7 INPUT			
	ldi r16, 0b10000000				; pin PD6-PD0 as input
	out DDRD, r16					; set PD7 as output
	ldi r16, 0xFF					; set PORTB as LEDS - OUTPUT
	out DDRB, r16	
	
	; set up adc 
	clr r16
	ldi r16, (1<<ADLAR)  ; left adjust the result
	out ADMUX, r16
	
	;set TCNT2 as PCPWM	
	;define compA timer value	
	out OCR2, r16
	ldi r16, 36 ;12 is 5% step -> start value 15% duty cycle
				; the value will be 20% duty cycle upon the first press of sw0
				; which will be before the press of the sw6 button to take the sample
	out OCR2, r16
	
	;Set TCCR2
	;for Phase Correct and Phase and Frequency Correct
	; Set output on compare match
	; Clear when up-counting, set when downcounting
	ldi r16, (1<<COM21) | (0<<COM20)
	
	; Set when up counting, clear when down counting
	;ldi r16, (1<<COM21) | (1<<COM20)
	;set PWM mode
	
	ldi r17, (0<<FOC2) | (0<<WGM21) |  (1<<WGM20)
	or r16,r17
	out TCCR2, r16
	
	;Set TCCR2
	;set PWM mode
	;set prescaler, start watch
	ldi r17, (1<<CS22) | (0<<CS21) | (1<<CS20)     ; prescaler used: 128
	or r16,r17
	out TCCR2, r16
	; enable interrupt on input capture event on timer1
	; disble interrupts on timer2. 
	ldi r16, (1<<TICIE1)
	out TIMSK, r16	
	
	;reset speed_up_steps counter
	ldi xl, 0x70
	clr xh
	ldi r16, 0 ;10 steps of 5% duty cycle increase
	st x, r16

	;reset slow_down_steps counter
	ldi xl, 0x72
	clr xh
	ldi r16, 0 ;14 steps of 5% duty cycle decrease
	st x, r16
	
	;set store sample pointer starting on 0x90 data memory address
	ldi yl, 0x90
	clr yh
	
	;reset sample counter
	clr samples
	
START:
	

SPEED_UP:
	ldi xl, 0x70
	clr xh
	ld r16 , x
	cpi r16 , 11				;10+1 steps of 5% duty cycle increase
	brsh KEEP_SPEED
	
	;increase duty cycle
	rcall  INC_SPEED
	rcall TIME_DELAY_3S
	rjmp SKIP           ; if we increased speed, skip slowing down for now
	
KEEP_SPEED:
	ldi xl, 0x72
	clr xh
	ld r16 , x			; check if we have ever decreased speed 
	cpi r16, 1			; if we have, this means we're on the braking process
	brsh SLOW_DOWN		; and we mustn't wait 10 sec
	rcall TIME_DELAY_10S
	
SLOW_DOWN:	
						; 14 steps of 5% duty cycle decrease 
	cpi r16 , 14		; (from 70% to 0% duty cycle)		
	brsh SKIP
	
	; decrease duty cycle	
	rcall DEC_SPEED
	rcall TIME_DELAY_3S

SKIP:

	; 4-Pulse Count
	;reset counted pulses from prev measurment
	clr pulses

	rcall SET_TIMER0_AND_COUNTER1 	;start timer-counter with special setup

	sei
CONT:
	cpi pulses, 4 					;count 4 pulses and then stop
	brlo CONT
	cli
	

	in intervalOf4Pulses, TCNT0 	;take speed measurment from timer0
	rcall STOP_TIMER0_AND_COUNTER1	;stop timer-counter
	
	rcall SAVE_SAMPLES				;save adcmax and intervalOf4Pulses
	cpi samples, 25
	brsh DISPLAY_SAMPLES
	rjmp START
	
DISPLAY_SAMPLES:
	
	cpi yl , 0x90				; check if we have values to display
	breq END_SEQUENCE
	
	rcall GET_SW7
	rcall WAIT_FOR_RELEASE
	rcall SHOW_VALUES
	rjmp DISPLAY_SAMPLES

END_SEQUENCE:
	ldi r16, 0x00				; if all leds light up, means we reached the end
	out PORTB, r16
ENDLOOP:
	in r16, PINB
	com r16
	out PORTB, r16
	rcall TIME_DELAY_1S
	rjmp ENDLOOP 				; The program ends here
	
	
; FUNCTIONS

ENABLE_ADC:
	push r16
	;set up adc parameters
	; aden -> enable adc
	; adsc -> start adc
	; adate -> auto trigger for free running mode
	; adie -> enable interrupts
	; adps2/1/0 -> set prescaler to 32 = 125kHz effective frequency of adc 

	;enable adc and clear ADIF 
	ldi r16, (1<<ADEN)	| (1<<ADSC) | (1<<ADATE) | (1<<ADIF) | (1<<ADIE)  | (1<<ADPS2) | (0<<ADPS1) | (1<<ADPS0)								
	out ADCSRA, r16
	pop r16

STOP_TIMER0_AND_COUNTER1:
	push r16
	;stop timer0-counter1
	;stop timer0
	ldi r16, (0<<CS02) | (0<<CS01) | (0<<CS00)		; prescaler used: 1
	out TCCR0, r16
	
	;stop counter1
	ldi r16, (0<<CS12) | (0<<CS11) | (0<<CS10) | (0 << ICES1)
	out TCCR1B, r16

	pop r16
	ret

SET_TIMER0_AND_COUNTER1:
	push r16
	push r17

	;clear flags by hand if they are set from before
	in r16, TIFR
	ldi r17 , 0b00100101 ;bit0, bit2, bit5
	or r16,r17
	out TIFR, r16

	;reset timer0 value
	clr r16
	out TCNT0, r16

	;reset timer1 value
	clr r16
	out TCNT1H, r16
	out TCNT1L, r16

	;set TCNT0 as stopwatch
	;set prescaler, start watch
	ldi r16, (0<<CS02) | (1<<CS01) | (1<<CS00)		; prescaler used: 64
	out TCCR0, r16									; we must count 132 interrupts 
															; for a ~0.5 s delay
	;set TCNT1 as counter
	;set counter mode on falling edge
	ldi r16, (1<<CS12) | (1<<CS11) | (0<<CS10) | (0 << ICES1)
	out TCCR1B, r16

	pop r17
	pop r16
	ret
	
; saves the values for speed and input voltage in motor
SAVE_SAMPLES:
	push r16
	
	st y+, intervalOf4Pulses
	
	;Com leds just for debug
	in r16, PINB
	com r16
	out PORTB, r16
	
	inc samples 
	pop r16
	ret

SHOW_VALUES:	
	push r16
	
	ld r16, -y					; load value from memory
	com r16						; adjust to inverse logic of leds
	out PORTB, r17				; display value via leds
	rcall TIME_DELAY_1S			; wait 2sec
	rcall TIME_DELAY_1S
	
	pop r16
	ret

; this just makes the microcontroller "wait" for 1 sec		
TIME_DELAY_1S:
	ldi  outer_count, 21
    ldi  mid_count, 75
    ldi  inner_count, 191
L2: dec  inner_count
    brne L2
    dec  mid_count
    brne L2
    dec  outer_count
    brne L2

	ret
	
; this just makes the microcontroller "wait" for 3 sec		
TIME_DELAY_3S:
	ldi  outer_count, 61
    ldi  mid_count, 225
    ldi  inner_count, 64
L23: dec  inner_count
    brne L23
    dec  mid_count
    brne L23
    dec  outer_count
    brne L23

	ret
	
; this just makes the microcontroller "wait" for 10 sec		
TIME_DELAY_10S:
	ldi  outer_count, 203
    ldi  mid_count, 236
    ldi  inner_count, 133
L21: dec  inner_count
    brne L21
    dec  mid_count
    brne L21
    dec  outer_count
    brne L21

	ret
	
; function that waits for the release of a button to avoid bounce effects
WAIT_FOR_RELEASE:
    push r16
WAIT:
    in r16, PINC				; wait untill all pinc are at the default value
	cpi  r16, 0b11000011		; pd2-pd6 are on zero cause of JTAG
	brne WAIT					; this is irrelevant to the program though
	pop r16			    		; beacause we never actually use any of these pins
	ret

; sw7 press function
GET_SW7:
	 push r16
PRESS7:
	in r16,PINC 				; in order to isolate bit we want,for the check
    andi r16, 0b10000000   		; if sw7 is pressed then R16 with have zero value
	cpi r16, 0
	brne PRESS7 				; Branch if SW7 is pressed
    pop r16 					; If Wrong switch is pressed
	ret

; increase duty cycle by 5%
INC_SPEED:
	push r16
	push r17
	cli

	in r16, PINB
	com r16
	out PORTB, r16							; invert leds

	;increase by 12 -> 5% duty cycle OCR2 timer value
	in r16, OCR2
	ldi r17, 12
	add r16, r17
	out OCR2, r16
	
	; increase increments counter
	ldi xl, 0x70
	clr xh
	ld r16, x
	inc r16
	st x, r16

	sei
	pop r17
	pop r16
	ret

; decrease duty cycle by 5%
DEC_SPEED:
	push r16
	cli
	
	in r16, PINB
	com r16
	out PORTB, r16							; invert leds

	;decrease by 12 OCR2 timer value
	in r16, OCR2
	subi r16, 12
	out OCR2, r16

	;increase by 1 counter
	ldi xl, 0x72
	clr xh
	ld r16, x
	inc r16
	st x, r16

	sei
	pop r16
	ret
	
	
; INTERRUPT HANDLERS

; Timer1 Event Capture Interrupt Handler
; Practiacally this handler just counts pulses
; timer 1 capture interrupt handler
TIMER1_CAPT:

	push r16
	in r16, SREG 							; save SREG in stack
	push r16
	cli

	inc pulses
	
	sei
	pop r16
	out SREG, r16
	pop r16
	reti