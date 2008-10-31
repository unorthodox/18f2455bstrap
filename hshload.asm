;Basic USB Bootload program for 18f2455
;Copyright 2008 Harvey Harrison <harvey.harrison@gmail.com>

	processor 18f2455
	include <p18f2455.inc>
	__config	_CONFIG1L, _USBDIV_2_1L & _CPUDIV_OSC3_PLL4_1L & _PLLDIV_6_1L
;
;Ugly and stupid as hell, but can't get __CONFIG to work
;
	ORG	_CONFIG2L
;	DATA	0x32f0
	DATA	0xfeef
	DATA	0xf8ff
	DATA	0xff9b
	DATA	0xffff
	DATA	0xffff
	DATA	0xffff

;
;Constant declarations, reserve memeory for subroutine use
;
USB_MAX_PACKET_SIZE	EQU	8
NUM_CONFIGURATIONS	EQU	1
NUM_INTERFACES		EQU	1
TOKEN_SETUP		EQU	0x00
TOKEN_IN		EQU	0x01
TOKEN_OUT		EQU	0x02

SUB_TEMP0		EQU	0x50
SUB_TEMP1		EQU	0x51

;
;Main program area
;
	ORG 0x0000
INIT		CLRF	PORTA, 0		;clear PORTA State
		MOVLW	0x0F			;set up PORTA to be digital I/Os
		MOVWF	ADCON1, 0
		CLRF	TRISA, 0		;set up all PORTA pins to be digital outputs
USB_INIT	CLRF	UIE, 0			;mask all USB interrupts
		CLRF	UIR, 0			;clear all USB interrupt flags
		SETF	UEIE, 0			;enable all error interrupts
		MOVLW	0x10			;configure USB for low-speed transfers and to use the on-chip transciever and pull-up resistor
		MOVWF	UCFG, 0
		MOVLW	0x08			;enable the USB module and its supporting circuitry
		MOVWF	UCON, 0
SET_LIT		MOVLW	0x34
		MOVWF	TOKEN_SETUP, 0
		MOVLW	0x24
		MOVWF	TOKEN_IN, 0
		MOVLW	0x04
		MOVWF	TOKEN_OUT, 0
WT_SE0		BTFSC	UCON, SE0, 0		;Wait for the initial SE0 to end
		BRA	-2			;Back to WT_SE0
		CALL	USB_RST			;It's useful to have USB_RST as a subroutine, so call rather than branch
MAIN_LOOP	CALL	USB_INTR
		BRA	-3			;call is a two-word instruction, back to MAIN_LOOP
;
;USB_RST subroutine
;
USB_RST		BCF	UIR, TRNIF, 0		;clear out the USB transmit FIFO (4 entries)
		BCF	UIR, TRNIF, 0
		BCF	UIR, TRNIF, 0
		BCF	UIR, TRNIF, 0
		MOVLW	UEP0 & 0xff		;disable all USB endpoints, ie. zero UEP0-UEP15
		MOVWF	FSR0L, 0
		CLRF	FSR0H, 0
		MOVLW	(UEP15 + 1) & 0xff
		CALL	SUB_CLEAR_RAM
		CLRF	FSR0L, 0		;clear all USB Block descriptors 0x0400 - 0x04FF
		BSF	FSR0H, 2, 0
		MOVLW	0x00
		CALL	SUB_CLEAR_RAM
		CLRF	UADDR, 0		;clear the USB address
		CLRF	UIR, 0			;clear all USB interrupt flags
BD0_INIT	MOVLB	0x04			;bank select USB buffer selector area
		MOVLW	0x88			;set BD0O as USB-owned, DTS enabled
		MOVWF	0x00, 1
		MOVLW	USB_MAX_PACKET_SIZE	;set BD0O packet size
		MOVWF	0x01, 1
		MOVLW	0x80			;set BD0O buffer address to 0x0480 (little endian)
		MOVWF	0x02, 1
		MOVLW	0x04
		MOVWF	0x03, 1
		MOVLW	0x08			;set BD0I as CPU-owned, DTS enabled
		MOVWF	0x04, 1
		MOVLW	USB_MAX_PACKET_SIZE	;set BD0I packet size
		MOVWF	0x05, 1
		MOVLW	0xa0			;set BD0O buffer address to 0x04a0 (little endian)
		MOVWF	0x06, 1
		MOVLW	0x04
		MOVWF	0x07, 1
		MOVLW	0x16			;enable IN, OUT and CONTROL packets for EP0
		MOVWF	UEP0
		RETURN
;
;USB Interrupt Handling subroutine
;
USB_INTR	BTFSC	UIR, UERRIF, 0		;test for USB error bits, clear the detailed error register
		CLRF	UEIR, 0
		BTFSC	UIR, SOFIF, 0		;test for start-of-frame received, clear if so, we currently ignore all SOF tokens
		BCF	UIR, SOFIF, 0
		BTFSS	UIR, IDLEIF, 0		;test for idle case, suspend USB hardware if so
		BRA	3			;skip past suspend setting and the return
		BSF	UCON, SUSPND, 0
		BCF	UIR, IDLEIF, 0
		RETURN
		BTFSS	UIR, ACTVIF, 0		;test for activity, resume USB if so
		BRA	3			;skip resume set and the return
		BCF	UCON, SUSPND, 0
		BCF	UIR, ACTVIF, 0
		RETURN
		BTFSC	UIR, STALLIF, 0		;ignore stall token sent condition
		BCF	UIR, STALLIF, 0
		BTFSS	UIR, URSTIF, 0		;check for USB bus reset
		BRA	3			;skip USB bus reset and the return (Call is two words)
		CALL 	USB_RST
		RETURN
		BTFSS	UIR, TRNIF, 0		;check for token completed in FIFO (Call is two words)
		BRA	2
		CALL	USB_PKT
		RETURN
USB_PKT		MOVLW	0x3c			;Mask the packet ID of the received packet
		ANDWF	USTAT, 0, 0		;and leave it in W
		CPFSEQ	TOKEN_SETUP, 0
		BRA	2
;		CALL	USB_TOK_SETUP
		CPFSEQ	TOKEN_IN, 0
		BRA	2
		CALL	USB_TOK_IN
		CPFSEQ	TOKEN_OUT, 0
		BRA	2
		CALL	USB_TOK_OUT
		RETURN
USB_TOK_OUT	MOVLB	0x04			;bank select USB buffer selector area
		MOVLW	0x88			;set BD0O as USB-owned, DTS enabled
		MOVWF	0x00, 1
		MOVLW	USB_MAX_PACKET_SIZE	;set BD0O packet size
		MOVWF	0x01, 1
		MOVLW	0xc8			;set BD0I as USB-owned, DATA1, DTS enabled
		MOVWF	0x04, 1
		MOVLW	0x00			;set BD0I packet size
		MOVWF	0x05, 1
USB_TOK_IN	
;
;Handle a GET_STATUS sent to the device
;
DEV_GET_STATUS	

include sub_clear_ram.inc
include sub_read_prog.inc
include sub_write_prog.inc

END

