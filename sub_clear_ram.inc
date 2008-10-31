;
;PIC18f2455 RAM Clearing Routine
;Copyright 2008, Harvey Harrison <harvey.harrison@gmail.com>
;
;Calling convention:
;Set the data address to erase (12bit) in FSR0
;Set SUB_TEMP0 to the number of bytes to clear (0 = 256)
;
;Side effects:
;SUB_TEMP0 is cleared upon return
;
SUB_CLEAR_RAM	CLRF	POSTINC0, 0		;Indirect clear of memory location, post increment
		DECFSZ	SUB_TEMP0, 1, 0		;decrement counter, skip branch if zero
		BRA	SUB_CLEAR_RAM
		RETURN

