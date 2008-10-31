;
;PIC18f2455 subroutine for reading program to data mem
;Copyright 2008, Harvey Harrison <harvey.harrison@gmail.com>
;
;Address of program memory must be already placed in the tabptr registers
;Address of data memory to write to must be in FSR0
;
;Two entry points defined:
;SUB_READ_PRG64 which reads 64 bytes
;SUB_READ_PRG which reads however many bytes the value at SUB_TEMP0 says (0 = 256)
;
SUB_READ_PRG	CLRF	SUB_TEMP0, 0		;SUB_TEMP0 is reserved for subroutine use
		BSF	SUB_TEMP0, 6, 0		;SUB_TEMP0 = 64 (the loop counter)
SUB_READ_PRG64	TBLRD *+			;Read/Post increment tblptr to latch
		MOVFF	TABLAT, POSTINC0	;Move the value to wherever FSR0 was set, and postinc
		DECFSZ	SUB_TEMP0, 1, 0		;decrement counter, skip branch if zero
		BRA	SUB_READ_PRG
		RETURN

