; i960SxChipset_Type103
; Copyright (c) 2022-2024, Joshua Scoggins
; All rights reserved.
; 
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:
;     * Redistributions of source code must retain the above copyright
;       notice, this list of conditions and the following disclaimer.
;     * Redistributions in binary form must reproduce the above copyright
;       notice, this list of conditions and the following disclaimer in the
;       documentation and/or other materials provided with the distribution.
; 
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
; ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
.file "ExecutionBody.s"
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__SREG__ = 0x3f
__RAMPZ__ = 0x3b
__tmp_reg__ = 0
__zero_reg__ = 1
; declarations
.macro ReadFromAddress addr, reg 
.if \addr<=0x3f
	in \reg, \addr
.else
	lds \reg, \addr
.endif
.endm
.macro WriteToAddress addr, reg
.if \addr<=0x3f
	out \addr, \reg
.else
	sts \addr, \reg
.endif
.endm

.macro DefineWriteFunction name, addr
.macro Write_\name reg
WriteToAddress \addr, \reg
.endm
.endm

.macro DefineReadFunction name, addr
.macro Read_\name reg
ReadFromAddress \addr, \reg
.endm
.endm

.macro DefineReadWriteFunctions name, addr
DefineReadFunction \name, \addr
DefineWriteFunction \name, \addr
.endm


.macro DefinePort letter, base
PIN\letter = \base
DDR\letter = \base + 1
PORT\letter = \base + 2
Port\letter\()_Output = PORT\letter
Port\letter\()_Input = PIN\letter
Port\letter\()_Direction = DDR\letter
Port\letter\()_BaseAddress = \base
DefineReadWriteFunctions Port\letter\()_Output, Port\letter\()_Output
DefineReadWriteFunctions Port\letter\()_Input, Port\letter\()_Input
DefineReadWriteFunctions Port\letter\()_Direction, Port\letter\()_Direction
.endm

.macro DefinePinFunction func, port, index
\func\()_Output = Port\port\()_Output
\func\()_Input = Port\port\()_Input
\func\()_Direction = Port\port\()_Direction
\func\()_BitIndex = \index
.if Port\port\()_BaseAddress<=0x3f
.macro \func\()_Output_Toggle
	sbi \func\()_Input, \func\()_BitIndex
.endm
.macro \func\()_Output_SkipNextIfClear
	sbic \func\()_Output, \func\()_BitIndex
.endm
.macro \func\()_Output_SkipNextIfSet
	sbis \func\()_Output, \func\()_BitIndex
.endm
.macro \func\()_Input_SkipNextIfClear
	sbic \func\()_Input, \func\()_BitIndex
.endm
.macro \func\()_Input_SkipNextIfSet
	sbis \func\()_Input, \func\()_BitIndex
.endm

.macro \func\()_Direction_SkipNextIfClear
	sbic \func\()_Direction, \func\()_BitIndex
.endm
.macro \func\()_Direction_SkipNextIfSet
	sbis \func\()_Direction, \func\()_BitIndex
.endm

; general purpose versions for input pins
.macro \func\()_SkipNextIfClear
	\func\()_Input_SkipNextIfClear
.endm
.macro \func\()_SkipNextIfSet
	\func\()_Input_SkipNextIfSet
.endm
.macro \func\()_SkipNextIfLow
	\func\()_SkipNextIfClear
.endm
.macro \func\()_SkipNextIfHigh
	\func\()_SkipNextIfSet
.endm

.macro \func\()_IfBitIsSetGoto dest
	\func\()_SkipNextIfClear
	rjmp \dest
.endm

.macro \func\()_IfBitIsClearGoto dest
	\func\()_SkipNextIfSet
	rjmp \dest
.endm

.endif
.endm

.macro DefinePortFunction func, port
\func\()_Output = Port\port\()_Output
\func\()_Input = Port\port\()_Input
\func\()_Direction = Port\port\()_Direction
\func\()_BaseAddress = Port\port\()_BaseAddress
DefineReadWriteFunctions \func\()_Output, \func\()_Output
DefineReadWriteFunctions \func\()_Input, \func\()_Input
DefineReadWriteFunctions \func\()_Direction, \func\()_Direction
.macro \func\()_Read dest
	Read_\()\func\()_Input \dest
.endm
.macro \func\()_Write dest
	Write_\()\func\()_Output \dest
.endm

.endm


.macro signalReady
	call displaySignalReady
	Ready_Output_Toggle
.endm
.macro sbisrj a, b, dest ; 3 cycles when branch taken, 2 cycles when skipped
	sbis \a, \b 	; 1 cycle if false, 2 cycles if true
	rjmp \dest		; 2 cycles
.endm
.macro sbicrj a, b, dest ; 3 cycles when branch taken, 2 cycles when skipped
	sbic \a, \b		; 1 cycle if false, 2 cycles if true
	rjmp \dest		; 2 cycles
.endm
.macro computeTransactionWindow ; 2 cycles
	AddressLinesLowest_Read r28
.endm
.macro setupRegisterConstants
.endm
.macro delay2cycles ; 2 cycles
	rjmp 1f
1:
.endm

.macro clearEIFR ; 1 cycle
	sbi EIFR, ADS_BitIndex
.endm

.macro StoreToDataPort lo=__low_data_byte960__,hi=__high_data_byte960__ ; 3 cycles
	DataLinesLower_Write \lo
	DataLinesUpper_Write \hi
	call displayDataPortValue
.endm

.macro WhenBlastIsLowGoto dest ; 3 cycles when branch taken, 2 cycles when skipped
	BLAST_SkipNextIfSet
	rjmp \dest
.endm
.macro WhenBlastIsHighGoto dest ; 3 cycles when branch taken, 2 cycles when skipped
	BLAST_SkipNextIfClear
	rjmp \dest
.endm

.macro getLowDataByte960
	DataLinesLower_Read __low_data_byte960__
.endm
.macro getHighDataByte960 
	DataLinesUpper_Read __high_data_byte960__
.endm
.macro justWaitForTransaction ; three cycles each iteration, three cycles leaving, we need some more lead time 
1: sbisrj EIFR, ADS_BitIndex, 1b
clearEIFR
.endm
.macro waitForTransaction ; 3 cycles per iteration waiting, 2 cycles when condition met
	justWaitForTransaction
.endm
.macro SkipNextIfBE0High  ; 1 cycle when false, 2 cycles when true
	BE0_SkipNextIfSet
.endm
.macro SkipNextIfBE1High  ; 1 cycle when false, 2 cycles when true
	BE1_SkipNextIfSet
.endm
.macro setDataLinesDirection dir
	Write_DataLinesLower_Direction \dir
	Write_DataLinesUpper_Direction \dir
.endm
.macro getDataWord960 ; 3 cycles
	getLowDataByte960 ; 1 cycle
	getHighDataByte960 ; 2 cycles
.endm
.macro StoreToMemoryWindow reg, offset
	.if offset == 0
		st Y, \reg
	.else
		std Y+\offset , \reg
	.endif
.endm
.macro LoadFromMemoryWindow reg, offset
	.if offset == 0
		ld \reg, Y
	.else
		ldd \reg, Y+\offset
	.endif
.endm
.macro Load16FromMemoryWindow offset, lo=__low_data_byte960__, hi=__high_data_byte960__
	LoadFromMemoryWindow \lo, \offset
	LoadFromMemoryWindow \hi, \offset\()+1
.endm
.macro Store16ToMemoryWindow offset, lo=__low_data_byte960__, hi=__high_data_byte960__
	StoreToMemoryWindow \lo, \offset
	StoreToMemoryWindow \hi , \offset\()+1
.endm
.macro StoreLowByteToMemoryWindow offset
	StoreToMemoryWindow __low_data_byte960__, \offset
.endm
.macro StoreHighByteToMemoryWindow offset
	StoreToMemoryWindow __high_data_byte960__, \offset
.endm

.macro StoreHighByteIfBE1Low offset ; 2 cycles when BE1 is HIGH, 5 cycles when BE1 is LOW
	SkipNextIfBE1High	; 1 cycle when false, 2 cycles when skipping
	StoreHighByteToMemoryWindow \offset ; 4 cycles
.endm
.macro HandleLastTwoBytesInWriteTransaction offset
	StoreHighByteIfBE1Low \offset\()+1
	StoreLowByteToMemoryWindow \offset ; save it without checking BE0 since we flowed into this part of the transaction
.endm
.macro FallthroughExecutionBody_WriteOperation
	signalReady 

	waitForTransaction
	WR_IfBitIsClearGoto .LXB_ShiftFromWriteToRead
	IsIOOperation_IfBitIsClearGoto .LXB_Write_DoIO_Nothing
	computeTransactionWindow
	getDataWord960
	SkipNextIfBE0High
	StoreLowByteToMemoryWindow 0 								; Yes, so store to the EBI
	WhenBlastIsLowGoto .LXB_do16BitWriteOperation 				; Is blast high? then keep going, otherwise it is a 8/16-bit operations
	signalReady 												; first word down, onto the next one
	StoreHighByteToMemoryWindow 1 								; Store the upper byte to the EBI
	delay2cycles												; wait for the next cycle to start
	WhenBlastIsHighGoto .L642                                   ; this is checking blast for the second set of 16-bits not the first
																; this is a 32-bit write operation so we want to check BE1 and then fallthrough to the execution body itself
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 2
	rjmp .LXB_SignalReady_ThenWriteTransactionStart
.endm
.macro ReadBodyPrimary
	call displayReadTransaction
	computeTransactionWindow
	Load16FromMemoryWindow 0
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart 
	signalReady 
	Load16FromMemoryWindow 2
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart 
	signalReady 
	Load16FromMemoryWindow 4
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart
	signalReady 
	Load16FromMemoryWindow 6
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart
	signalReady 
	Load16FromMemoryWindow 8
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart
	signalReady 
	Load16FromMemoryWindow 10
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart
	signalReady 
	Load16FromMemoryWindow 12
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart
	signalReady 
	Load16FromMemoryWindow 14
	StoreToDataPort 
.endm

.macro FallthroughExecution_ReadBody
	signalReady
	waitForTransaction 
	WR_IfBitIsSetGoto .LXB_ShiftFromReadToWrite
	IsIOOperation_IfBitIsClearGoto .LXB_readOperation_CheckIO_Nothing
	ReadBodyPrimary
	rjmp .LXB_FirstSignalReady_ThenReadTransactionStart
.endm
.macro sbrsrj a, b, dest 
	sbrs \a, \b
	rjmp \dest
.endm
.macro sbrcrj a, b, dest 
	sbrc \a, \b
	rjmp \dest
.endm
EIFR = 0x1c
GPIOR0 = 0x1e

DefinePort A, 0x00
DefinePort B, 0x03
DefinePort C, 0x06
DefinePort D, 0x09
DefinePort E, 0x0c
DefinePort F, 0x0f
DefinePort G, 0x12
DefinePort H, 0x100
DefinePort J, 0x103
DefinePort K, 0x106
DefinePort L, 0x109
DefinePinFunction ADS, D, 2
DefinePinFunction BLAST, D, 3
DefinePinFunction WR, D, 4
DefinePinFunction BE0, D, 5
DefinePinFunction BE1, D, 6
DefinePinFunction IsIOOperation, D, 7
DefinePinFunction HLDA, E, 4
DefinePinFunction Ready, G, 4

DefinePortFunction DataLinesLower, F
DefinePortFunction DataLinesUpper, C
DefinePortFunction AddressLinesLowest, K
DefinePortFunction AddressLinesLower, H
DefinePortFunction AddressLinesHigher, J
DefinePortFunction AddressLinesHighest, L

MemoryWindowUpper = 0xfc
__high_data_byte960__ = 4
__low_data_byte960__ = 3
__direction_ff_reg__ = 2
.global ExecutionBody
.global ExecutionBody_5MHz
.global doIOReadOperation
.global doIOWriteOperation
.text

ExecutionBody:
	; use the lowest registers to make sure that we have Y, r16, and r17 free for usage
	ser r16
	mov __direction_ff_reg__, r16 ; 0xff
; setup index register Y to be the actual memory window, that way we can only update the lower half
	clr r28
	ldi r29, MemoryWindowUpper
	rjmp .LXB_ReadTransactionStart ; jump into the top of the invocation loop
.LXB_readOperation_CheckIO_Nothing:
	call doIOReadOperation			      ; It is so call doIOReadOperation, back to c++
	rjmp .LXB_ReadTransactionStart
.LXB_FirstSignalReady_ThenReadTransactionStart:
	signalReady
.LXB_ReadTransactionStart:
	waitForTransaction 
	WR_IfBitIsSetGoto .LXB_ShiftFromReadToWrite
	IsIOOperation_IfBitIsClearGoto .LXB_readOperation_CheckIO_Nothing
	call displayAddress
	ReadBodyPrimary
	FallthroughExecution_ReadBody
.LXB_ShiftFromWriteToRead:
	setDataLinesDirection __direction_ff_reg__ ; change the direction to output
	IsIOOperation_IfBitIsClearGoto .LXB_readOperation_CheckIO_Nothing
	ReadBodyPrimary
	FallthroughExecution_ReadBody
.LXB_Write_DoIO_Nothing:
	call doIOWriteOperation 			 ; perform the write operation
	rjmp .LXB_WriteTransactionStart		 ; restart execution

.LXB_SignalReady_ThenWriteTransactionStart:
	signalReady 
.LXB_WriteTransactionStart:
	waitForTransaction
	WR_IfBitIsClearGoto .LXB_ShiftFromWriteToRead
	IsIOOperation_IfBitIsClearGoto .LXB_Write_DoIO_Nothing
	computeTransactionWindow
	getDataWord960
	SkipNextIfBE0High
	StoreLowByteToMemoryWindow 0 								; Yes, so store to the EBI
	WhenBlastIsLowGoto .LXB_do16BitWriteOperation 				; Is blast high? then keep going, otherwise it is a 8/16-bit operations
	signalReady 												; first word down, onto the next one
	StoreHighByteToMemoryWindow 1 								; Store the upper byte to the EBI
	delay2cycles												; wait for the next cycle to start
	WhenBlastIsHighGoto .L642                                   ; this is checking blast for the second set of 16-bits not the first
																; this is a 32-bit write operation so we want to check BE1 and then fallthrough to the execution body itself
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 2
	FallthroughExecutionBody_WriteOperation
.L642:
	getDataWord960				
	signalReady 				  ; start the next word signal and we can use the 6 cycles to get ready
	Store16ToMemoryWindow 2       ; use the time to store into memory while the ready signal counter is doing its thing
	WhenBlastIsLowGoto .LXB_WriteBytes4_and_5_End	; We can now safely check if we should terminate execution
	getDataWord960				
	signalReady					  ; Start the process for the next word ( at this point we will be at a 64-bit number once this ready goes through)
	Store16ToMemoryWindow 4 	  ; save the word (bits 32-47)
	WhenBlastIsHighGoto .L649	  ; we have more data to transfer
	getDataWord960				
	HandleLastTwoBytesInWriteTransaction 6
	FallthroughExecutionBody_WriteOperation
.L649:
	getDataWord960
	signalReady
	Store16ToMemoryWindow 6
	WhenBlastIsLowGoto .LXB_WriteBytes8_and_9_End
	getDataWord960
	signalReady
	Store16ToMemoryWindow 8
	WhenBlastIsHighGoto .L657
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 10
	FallthroughExecutionBody_WriteOperation
.L657:
	getDataWord960
	signalReady
	Store16ToMemoryWindow 10
	WhenBlastIsHighGoto .L661
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 12
	FallthroughExecutionBody_WriteOperation
.L661:
	getDataWord960
	signalReady
	Store16ToMemoryWindow 12
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 14
	FallthroughExecutionBody_WriteOperation
.LXB_WriteBytes4_and_5_End:
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 4
	FallthroughExecutionBody_WriteOperation
.LXB_WriteBytes8_and_9_End:
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 8
	FallthroughExecutionBody_WriteOperation
.LXB_do16BitWriteOperation:
	StoreHighByteIfBE1Low 1
	FallthroughExecutionBody_WriteOperation

.LXB_ShiftFromReadToWrite:
; start setting up for a write operation here
	setDataLinesDirection __zero_reg__ 	
	IsIOOperation_IfBitIsClearGoto .LXB_Write_DoIO_Nothing
	computeTransactionWindow
	getDataWord960
	SkipNextIfBE0High
	StoreLowByteToMemoryWindow 0
	WhenBlastIsLowGoto	.LXB_do16BitWriteOperation
	signalReady 
	StoreHighByteToMemoryWindow 1
	delay2cycles
	WhenBlastIsHighGoto .L642 ; this is checking blast for the second set of 16-bits not the first
	; this is a 32-bit write operation so we want to check BE1 and then fallthrough to the execution body itself
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 2
	FallthroughExecutionBody_WriteOperation

.macro waitForTransaction_5MHz 
	justWaitForTransaction ; 3 cycles per iteration waiting, 3 cycles when condition met
	; according to the SA/SB manual, W/R is setup at the same time as AS is asserted
	; so by the time we detect the event it should be safe :)
.endm
.macro delay4cycles
	lpm
	nop
.endm
.macro delay8cycles 
	delay4cycles
	delay4cycles
.endm

ExecutionBody_5MHz:
	; use the lowest registers to make sure that we have Y, r16, and r17 free for usage
	ser r16
	mov __direction_ff_reg__, r16 ; 0xff
; setup index register Y to be the actual memory window, that way we can only update the lower half
	clr r28
	ldi r29, MemoryWindowUpper
	rjmp .LXB_ReadTransactionStart_5MHz ; jump into the top of the invocation loop
.LXB_readOperation_CheckIO_Nothing_5MHz:
	call doIOReadOperation			      ; It is so call doIOReadOperation, back to c++
	rjmp .LXB_ReadTransactionStart_5MHz
.LXB_FirstSignalReady_ThenReadTransactionStart_5MHz:
	signalReady
.LXB_ReadTransactionStart_5MHz:
	waitForTransaction_5MHz
	delay4cycles
	WR_IfBitIsSetGoto .LXB_ShiftFromReadToWrite_5MHz 
	IsIOOperation_IfBitIsClearGoto .LXB_readOperation_CheckIO_Nothing_5MHz ; we have enough time to detect things here
.LXB_ReadBodyPrimary_5MHz:
	call displayAddress
	computeTransactionWindow ; at this point things are solidified so start as normal
	Load16FromMemoryWindow 0 ; this will take 8 cycles avr
	StoreToDataPort  ; this will take 2 cycles avr
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz ; up to this point everything should be normal
	signalReady ; this will take 1 cycle avr
	; this is where it starts to get a little different, we have 16 cycles AVR before we can do things again (it should be 12 but lets make sure of that)
	Load16FromMemoryWindow 2 ; this will take 8 cycles avr
	delay8cycles			 ; wait another 8 cycles avr, otherwise we will corrupt the data port
	; at this point we have four more cycles before we are ready for the next step
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz
	signalReady 
	Load16FromMemoryWindow 4
	delay8cycles			 ; wait another 8 cycles avr to prevent data port corruption
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz
	signalReady 
	Load16FromMemoryWindow 6
	delay8cycles			 ; wait another 8 cycles avr to prevent data port corruption
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz
	signalReady 
	Load16FromMemoryWindow 8
	delay8cycles			 ; wait another 8 cycles avr to prevent data port corruption
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz
	signalReady 
	Load16FromMemoryWindow 10
	delay8cycles			 ; wait another 8 cycles avr to prevent data port corruption
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz
	signalReady 
	Load16FromMemoryWindow 12
	delay8cycles			 ; wait another 8 cycles avr to prevent data port corruption
	StoreToDataPort 
	WhenBlastIsLowGoto .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz
	signalReady 
	Load16FromMemoryWindow 14
	delay8cycles			 ; wait another 8 cycles avr to prevent data port corruption
	StoreToDataPort 
	rjmp .LXB_FirstSignalReady_ThenReadTransactionStart_5MHz
.LXB_ShiftFromWriteToRead_5MHz:
	setDataLinesDirection __direction_ff_reg__ ; change the direction to output
	IsIOOperation_IfBitIsClearGoto .LXB_readOperation_CheckIO_Nothing_5MHz
	rjmp .LXB_ReadBodyPrimary_5MHz
.LXB_Write_DoIO_Nothing_5MHz:
	call doIOWriteOperation 			 ; perform the write operation
	rjmp .LXB_WriteTransactionStart_5MHz		 ; restart execution

.LXB_SignalReady_ThenWriteTransactionStart_5MHz:
	signalReady 
	; at this point we can safely wait :)
.LXB_WriteTransactionStart_5MHz:
	waitForTransaction_5MHz
	WR_IfBitIsClearGoto .LXB_ShiftFromWriteToRead_5MHz
	IsIOOperation_IfBitIsClearGoto .LXB_Write_DoIO_Nothing_5MHz
.LXB_WriteTransactionBody_Start_5MHz:
	computeTransactionWindow
	getDataWord960
	SkipNextIfBE0High
	StoreLowByteToMemoryWindow 0 								; Yes, so store to the EBI
	WhenBlastIsLowGoto .LXB_do16BitWriteOperation_5MHz 				; Is blast high? then keep going, otherwise it is a 8/16-bit operations
	signalReady 												; first word down, onto the next one
	StoreHighByteToMemoryWindow 1 								; Store the upper byte to the EBI, this takes 4 cycles
	delay2cycles												; wait for the next cycle to start
	delay2cycles
	delay8cycles			 ; wait another 8 cycles avr to prevent data corruption
	WhenBlastIsHighGoto .L642_5MHz                              ; this is checking blast for the second set of 16-bits not the first
																; this is a 32-bit write operation so we want to check BE1 and then fallthrough to the execution body itself
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 2
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.L642_5MHz:
	getDataWord960				
	signalReady 				  ; start the next word signal and we can use the 6 cycles to get ready
	Store16ToMemoryWindow 2       ; use the time to store into memory while the ready signal counter is doing its thing
	delay8cycles
	WhenBlastIsLowGoto .LXB_WriteBytes4_and_5_End_5MHz	; We can now safely check if we should terminate execution
	getDataWord960				
	signalReady					  ; Start the process for the next word ( at this point we will be at a 64-bit number once this ready goes through)
	Store16ToMemoryWindow 4 	  ; save the word (bits 32-47)
	delay8cycles
	WhenBlastIsHighGoto .L649_5MHz	  ; we have more data to transfer
	getDataWord960				
	HandleLastTwoBytesInWriteTransaction 6
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.L649_5MHz:
	getDataWord960
	signalReady
	Store16ToMemoryWindow 6
	delay8cycles
	WhenBlastIsLowGoto .LXB_WriteBytes8_and_9_End_5MHz
	getDataWord960
	signalReady
	Store16ToMemoryWindow 8
	delay8cycles
	WhenBlastIsHighGoto .L657_5MHz
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 10
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.L657_5MHz:
	getDataWord960
	signalReady
	Store16ToMemoryWindow 10
	delay8cycles
	WhenBlastIsHighGoto .L661_5MHz
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 12
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.L661_5MHz:
	getDataWord960
	signalReady
	Store16ToMemoryWindow 12
	delay8cycles
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 14
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.LXB_WriteBytes4_and_5_End_5MHz:
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 4
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.LXB_WriteBytes8_and_9_End_5MHz:
	getDataWord960
	HandleLastTwoBytesInWriteTransaction 8
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.LXB_do16BitWriteOperation_5MHz:
	StoreHighByteIfBE1Low 1
	rjmp .LXB_SignalReady_ThenWriteTransactionStart_5MHz
.LXB_ShiftFromReadToWrite_5MHz:
; start setting up for a write operation here
	setDataLinesDirection __zero_reg__ 	 ; this will take two cycles
	IsIOOperation_IfBitIsClearGoto .LXB_Write_DoIO_Nothing_5MHz 
	rjmp .LXB_WriteTransactionBody_Start_5MHz

