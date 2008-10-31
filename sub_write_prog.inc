;
;PIC18f2455 subroutine for reading program to data mem
;Copyright 2008, Harvey Harrison <harvey.harrison@gmail.com>
;
;Address of program memory must be already placed in the tabptr registers
;and be 64-byte aligned
;Address of data memory to write to must be in FSR0
;
;Due to constraintes in the PIC, 64 bytes is the minimum block write
;size (from erase)
;
SUB_ERASE_ROW	BCF	INTCON, GIE		;disable interrupts
		BSF	EECON1, EEPGD		;point to Flash program memory
		BCF	EECON1, CFGS		;access Flash program memory
		BSF	EECON1, WREN		;enable write to memory
		BSF	EECON1, FREE		;enable Row Erase operation
		MOVLW	0x55			;begin required EECON2 erase sequence
		MOVWF	EECON2
		MOVLW	0xAA
		MOVWF	EECON2
		BSF	EECON1, WR		;end required EECON2 erase sequence
		MOVLW	0x02			;We must write 2 rows of data
		MOVWF	SUB_TEMP0, 0
		TBLRD*-				;Dummy decrement to ensure we stay in the correct row
SUB_WRITE_HREG	MOVLW	0x20			;We write 32 bytes at a time to the holding registers
		MOVWF	SUB_TEMP1, 0
SUB_WRITE_BYTE	MOVF	POSTINC0, 0, 0		;Read a byte of data to be written to flash
		MOVWF	TABLAT, 0		;Write to the table latch
		TBLWT+*				;Write the latch, preinc into a holding reg
		DECFSZ	SUB_TEMP1, 1, 0		;dec the count of bytes in the row
		BRA	SUB_WRITE_BYTE
SUB_WRITE_PROG	BSF	EECON1, EEPGD		;point to Flash program memory
		BCF	EECON1, CFGS		;access Flash program memory
		BSF	EECON1, WREN		;enable write to memory
		MOVLW	0x55			;begin required EECON2 write sequence
		MOVWF	EECON2
		MOVLW	0xAA
		MOVWF	EECON2
		BSF	EECON1, WR		;end required EECON2 write sequence
		DECFSZ	SUB_TEMP0, 1, 0		;decrement the number of rows written
		BRA	SUB_WRITE_HREG
		BSF	INTCON, GIE		;re-enable interrupts
		RETURN				;done row erase and reprog, return
