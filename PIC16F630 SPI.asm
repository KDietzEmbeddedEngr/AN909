;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;                                                                     
;                     Software License Agreement                      
;                                                                     
; The software supplied herewith by Microchip Technology Incorporated 
; (the "Company") for its PICmicro® Microcontroller is intended and   
; supplied to you, the Company’s customer, for use solely and         
; exclusively on Microchip PICmicro Microcontroller products.         
;                                                                     
; The software is owned by the Company and/or its supplier, and is     
; protected under applicable copyright laws. All rights are reserved.  
; Any use in violation of the foregoing restrictions may subject the  
; user to criminal sanctions under applicable laws, as well as to     
; civil liability for the breach of the terms and conditions of this  
; license.                                                             
;                                                                      
; THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,   
; WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED   
; TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A         
; PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,   
; IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR          
; CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.                    
;                                                                     
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;	Filename:				PIC16F630 SPI.asm
;	Date:						November 3, 2003
;	File Version:			1.0
;	Assembled using:		MPLAB 6.40.00.0
;
;	Author:					Ken Dietz
;	Company:					Microchip Technology, Inc.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;   Files required:		p16f630.inc or p16f876.inc
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;	Purpose:
;
;	Besides serving as an update to AN648, the firmware for this app note is 
;	being written in such a way as to describe the program flow between a 
;	microcontroller and an SPI EEPROM.  That is, when connecting a 
;	microcontroller to an SPI EEPROM, the sequence of commands sent to the 
;	memory is vitally important, so this serves to enhance the information 
;	that we deliver in our datasheets.  As a side benefit, a small assembly 
;	library will be written for the standard SPI EEPROM instruction set in
;  bit-banging format (write, read, write enable, etc.).
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;	Program Description:
;
;	After using MPLAB to write code for the PIC16F630, designers can download 
;  their hex file into the target processor for testing using the PICkit or 
;  some other programming tool.  This code will initialize the chip then wait 
;  for user input in the form of a pushbutton switch.  After entering state 2, 
;  128 bytes of information will be transferred from the microcontroller to 
;  the external EEPROM (25LC160B).  As the program advances through its 
;  respective states, the LEDs on the development board are continually 
;  updated.  In state 4, the contents of the external EEPROM are compared to 
;  the data sent to the memory in the form of a checksum.  The program then 
;  starts over by returning to state 1.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;	Notes:
;	
;	1. Utilized PIC16F630~676 Assembly Language Programming Template.asm as 
;		the file template.  Also using snippets of code from PICkit examples.
;
;	2.	The program was developed for two processors to show a simple
;     migration from one mid-range device to another (PIC16F630 & PIC16F876).
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	list        p=16f630        ; list directive to define processor
	#include    <p16f630.inc>   ; processor specific variable definitions
ver	equ	630

;	list        p=16f876        ; list directive to define processor
;	#include    <p16f876.inc>   ; processor specific variable definitions
;ver	equ	876

	errorlevel  -302            ; suppress message 302 from list file

; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if	ver == 630
    __CONFIG	_CPD_OFF & _CP_OFF & _BODEN_OFF & _MCLRE_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT
#endif

#if	ver == 876
;	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _RC_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF
	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_ON & _CPD_OFF
#endif

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Defines
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define RA0		PORTA, 0            ; (Input/Output)
#define RA1		PORTA, 1            ; (Input/Output)
#define RA2		PORTA, 2            ; (Input/Output)
#define RA3		PORTA, 3            ; (Input Only)
#define RA4		PORTA, 4            ; (Input/Output)
#define RA5		PORTA, 5            ; (Input/Output)

#define RC0		PORTC, 0            ; (Input/Output)
#define RC1		PORTC, 1            ; (Input/Output)
#define RC2		PORTC, 2            ; (Input/Output)
#define RC3		PORTC, 3            ; (Input/Output)
#define RC4		PORTC, 4            ; (Input/Output)
#define RC5		PORTC, 5            ; (Input/Output)

;#define	SW1TRIS		b'11111111'

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;SPI Defines
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define	SCK		PORTC, 0			;SPI Clock Output
#define	SO			PORTC, 1			;Serial output
#define	SI			PORTC, 2			;Serial input
#define	CS			PORTC, 3			;Chip select output
#define	WP			PORTC, 4			;Write protect output
#define	HOLD		PORTC, 5			;Hold SPI operations output
#define	NumberOfPages	.4			;25LC640, 25LC160B
#define	PageSize	.32				;25LC640, 25LC160B size of page write capability
;#define	NumberOfPages	.2			;25AA256
;#define	PageSize	.64				;25AA256, size of page write capability
#define	AddrHCon	0x00				;Locations where data will be stored in external EEPROM
#define	AddrLCon	0x00				;Locations where data will be stored in external EEPROM
;#define	AddrHCon	0x1F				;Locations where data will be stored in external EEPROM
;#define	AddrLCon	0x80				;Locations where data will be stored in external EEPROM
#define	SPITRIS	0x04				;TRIS value

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED Defines
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if	ver == 630
#define	LED1TRIS		b'11001111'
#define	LED2TRIS		b'11001111'
#define	LED3TRIS		b'11101011'
#define	LED4TRIS		b'11101011'
#define	LED5TRIS		b'11011011'
#define	LED6TRIS		b'11011011'
#define	LED7TRIS		b'11111001'
#define	LED8TRIS		b'11111001'
#define	LEDOFFTRIS	b'00000000'
#define	LED1ON		b'00010000'
#define	LED2ON		b'00100000'
#define	LED3ON		b'00010000'
#define	LED4ON		b'00000100'
#define	LED5ON		b'00100000'
#define	LED6ON		b'00000100'
#define	LED7ON		b'00000100'
#define	LED8ON		b'00000010'
#define	LEDOFF		b'00000000'

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;Bank Select Defines
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define	Bank0		bcf	STATUS, RP0
#define	Bank1		bsf	STATUS, RP0
#endif

#if	ver == 876
Bank0		MACRO
		bcf	STATUS,	RP0
		bcf	STATUS,	RP1
		ENDM

Bank1		MACRO
		bsf	STATUS,	RP0
		bcf	STATUS,	RP1
		ENDM

Bank2		MACRO
		bcf	STATUS,	RP0
		bsf	STATUS,	RP1
		ENDM

Bank3		MACRO
		bsf	STATUS,	RP0
		bsf	STATUS,	RP1
		ENDM
#endif

; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Register locations 0x20 to 0x5F (64 bytes) are General Purpose 
; registers, implemented as static RAM and are mapped across both Banks. 
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	cblock  0x20
	WTemp
	StatusTemp
	State					;Current state value
	PreviousState		;Stores previous state value when entering a state
	SPIBuf				;Main buffer variable
	ByteCount			;Counts number of bytes sent
	PageCount			;Counts number of pages sent
	AddrMSB				;Holds MSB of SPI EEPROM address
	AddrLSB				;Holds LSB of SPI EEPROM address
	Count00				;Used in various locations
	Count01				;Used in MegaDelay
	Count02				;Used in MegaDelay
	OUTER					;Used in delay routine
	INNER					;Used in delay routine
	CheckSumH			;Stores checksum high byte
	CheckSumL			;Stores checksum low byte
	NewCheckSumH		;Used to compare data read from EEPROM
	NewCheckSumL		;Used to compare data read from EEPROM
	endc

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Begin Program Memory
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	ORG		0x000				;RESET Vector
	nop							;for ICD use
	goto		S0					;Initialize the chip in State 0

	ORG		0x004				;Interrupt Vector
	movwf		WTemp				;Save W register
	swapf		STATUS, W		;Swap status to be saved into W
	movwf		StatusTemp		;Save STATUS register

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;   < place ISR code here, not used in this application >
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	swapf		StatusTemp, W	; swap status_temp into W, sets Bank to original state
	movwf		STATUS			; restore STATUS register
	swapf		WTemp, F
	swapf		WTemp, W			; restore W register
	retfie

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;Subroutine: S0, State 0
;
;Description: Initialize the chip
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S0
	Bank0
	movf		State, W
	movwf		PreviousState
	clrf		State
	clrf		INTCON
	call		LEDSOFF
	call		Led1
	call		MegaDelay

#if	ver == 630
	Bank1
	call		0x3FF				;Retrieve factory calibration value
	movwf		OSCCAL			;Update register with factory cal value
	movlw		0xC0				;1100 0000
	movwf		OPTION_REG
	clrf		TRISA				; Write to TRISA register
	movlw		SPITRIS
	movwf		TRISC				; Write to TRISC register
	clrf		PIE1
	clrf		WPUA
	clrf		IOCA
	clrf     VRCON				;CVref circuit powered down, no Idd drain
	clrf		EECON1
	Bank0
	movlw		0xFF
	movwf		PORTC
	clrf		PORTA
	clrf		PIR1
	movlw		0x07
	movwf		CMCON
#endif

#if	ver == 876
	Bank1
	movlw	0xC0
	movwf	OPTION_REG
	movlw	0xFF
	movwf	TRISA
	clrf	TRISB
	movlw	SPITRIS
	movwf	TRISC
	clrf	PIE1
	movlw	0x06
	movwf	ADCON1
	Bank0
	clrf	SSPCON
	clrf	RCSTA
	clrf	ADCON0
	clrf	INTCON
#endif

	bsf		CS
	bsf		WP
	bsf		HOLD
	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	;Wait for user input to leave State 0
	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BP1d
	call		Led1
	call		Led2
	call		Led3
	call		Led4
	call		Led5
	call		Led6
	call		Led7
	call		Led8
#if	ver == 630
	btfsc		RA3				;User input is on RA3
#endif
#if	ver == 876
	btfsc		RA4				;User input is on RA4
#endif
	goto		BP1d
	movlw		0x00
	call		Delay
BP1u
#if	ver == 630
	btfss		RA3				;User input is on RA3
#endif
#if	ver == 876
	btfss		RA4				;User input is on RA4
#endif
	goto		BP1u
	goto		S1
S0End

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;Subroutine:	S1, State 1
;
;Description:	Main routine for program.
;
;Notes:
;1. This state is more about making decisions rather than taking action.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S1
	Bank0
	movf		State, W
	movwf		PreviousState
	movlw		0x01
	movwf		State
	call		LEDSOFF
	movlw		AddrHCon
	movwf		AddrMSB
	movlw		AddrLCon
	movwf		AddrLSB
S1Loop
	call		Led2
	call		MegaDelay
	bcf		CS
	call		ReadStatusSPI
	bsf		CS
	movlw		0x02
	subwf		SPIBuf, 0			;Compare value read from EEPROM status register with expected value.
	btfsc		STATUS, Z
	goto		S2
	btfss		SPIBuf, 1			;Begin testing and corrective procedures
	goto		S6
	btfsc		SPIBuf, 0
	goto		S1Loop
	movlw		0x8C
	xorwf		SPIBuf, 0
	btfss		STATUS, Z
	goto		S5
	goto		S1Loop
S1End

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;Subroutine:	S2, State 2 
;
;Description:
;In this state, the microcontroller will write the data EE contents from
;the microcontroller to the EEPROM buffer.  After filling the buffer,
;the microcontroller will initiate a write cycle on the EEPROM then poll
;the WIP bit of the EEPROM.  State 2 contains decision making code similar 
;to the code at the end of S1.
;
;Notes:
;1. It's assumed from the FSM diagram that SR=0x02 upon entering this routine.
;
;Algorithm for S2:
;Load page count and page size
;DoLoopOuter
;	Drop chip select
;	Write SPI header
;	DoLoopInner
;		Get byte from data EE
;		Send byte to SPI bus
;		Decrement ByteCount, skip if zero
;			goto DoLoopInner
;	Raise chip select
;	Poll status register
;	Decrement PageCount, skip if zero
;		goto DoLoopOuter
;goto S3
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S2
	Bank0
	movf		State, W
	movwf		PreviousState
	movlw		0x02
	movwf		State
	call		LEDSOFF
	call		Led3
	call		MegaDelay
;#if	ver == 630
;	Bank1
;#endif
;#if	ver == 876;
;	Bank2
;#endif
	banksel	EEADR					;banksel macro fine to use here
	clrf		EEADR
	Bank0
	clrf		CheckSumH			;Track checksum of data sent to EEPROM
	clrf		CheckSumL
	movlw		NumberOfPages
	sublw		0xFF
	movwf		PageCount
S2DoLoopOuter
	incf		PageCount, f
	btfsc		STATUS, Z
	goto		S3
	movlw		PageSize
	movwf		ByteCount
	bcf		CS
	call		WriteSPIHeader
S2DoLoopInner
	call		DATA_EEPROM_READ
	addwf		CheckSumL, f
	btfsc		STATUS, C
	incf		CheckSumH, f
	call		TransferSPIData
;#if	ver == 630
;	Bank1
;#endif
;#if	ver == 876;
;	Bank2
;#endif
	banksel	EEADR					;banksel macro fine to use here
	incf		EEADR, f
S2Continue
	Bank0
	decfsz	ByteCount, f
	goto		S2DoLoopInner
NextPage
	bsf		CS
	movlw		PageSize
	addwf		AddrLSB, f
	btfsc		STATUS, C
	incf		AddrMSB, f
S2StatusPoll						;Begin decision tree
	bcf		CS
	call		ReadStatusSPI
	bsf		CS
	movlw		0x02
	subwf		SPIBuf, 0
	btfsc		STATUS, Z
	goto		S2DoLoopOuter
	btfss		SPIBuf, 1
	goto		S6
	btfsc		SPIBuf, 0
	goto		S2StatusPoll
	movlw		0x8C
	xorwf		SPIBuf, 0
	btfss		STATUS, Z
	goto		S5
	goto		S2StatusPoll
S2End

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;Subroutine:	S3, State 3
;
;Description:
;State 3 is basically a hold state waiting for user input.  As soon as
;the user depresses the switch for a second time and lets go, then
;the program advances to State 4.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S3
	Bank0
	movf		State, W
	movwf		PreviousState
	movlw		0x03
	movwf		State
	call		LEDSOFF
	call		Led4
	call		MegaDelay
	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	;Wait for user input to leave State 3
	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BP2d
#if	ver == 630
	btfsc		RA3				;User input is on RA3
#endif
#if	ver == 876
	btfsc		RA4				;User input is on RA4
#endif
	goto		BP2d
	movlw		0x00
	call		Delay
BP2u
#if	ver == 630
	btfss		RA3				;User let go of the switch
#endif
#if	ver == 876
	btfss		RA4				;User input is on RA4
#endif
	goto		BP2u
	goto		S4
S3End

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;Subroutine:	S4, State 4
;
;Description:
;At this point in the program, the microcontroller reads back the contents
;of the EEPROM where it wrote, then verifies the checksum.  After all the 
;values have been read from the EEPROM and verified, the program loops back
;to State 1, the main loop.  If verify fails, all LED's are lit up and 
;the program is held in an infinite loop.  The board requires a hardware
;reset if verify fails.
;
;Notes:
;
;1. CheckSumH and CheckSumL already hold checksum from State 2 that was
;   written to SPI bus.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S4
	Bank0
	movf		State, W
	movwf		PreviousState
	movlw		0x04
	movwf		State
	call		LEDSOFF
	call		Led5
	call		MegaDelay
	movlw		NumberOfPages
	sublw		0x00
	movwf		PageCount
	movlw		AddrHCon
	movwf		AddrMSB
	movlw		AddrLCon
	movwf		AddrLSB
	clrf		NewCheckSumH
	clrf		NewCheckSumL
	bcf		CS
	call		ReadSPIHeader
	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	;Get checksum from 128 locations of the external EEPROM
	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S4DoLoopOuter
	movlw		PageSize
	movwf		ByteCount
S4DoLoopInner
	clrf		SPIBuf
	call		TransferSPIData
	movf		SPIBuf, W
	addwf		NewCheckSumL, f
	btfsc		STATUS, C
	incf		NewCheckSumH, f
	decfsz	ByteCount, f
	goto		S4DoLoopInner
	incfsz	PageCount, f
	goto		S4DoLoopOuter
	bsf		CS

	movf		NewCheckSumL, W
	subwf		CheckSumL, W
	btfss		STATUS, Z		;Checksums should be the same, result is zero
	goto		ErrorHandler

	movf		NewCheckSumH, W
	subwf		CheckSumH, W
	btfss		STATUS, Z		;Checksums should be the same, result is zero
	goto		ErrorHandler

	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	;Wait for user input to leave State 4
	;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BP3d
#if	ver == 630
	btfsc		RA3				;User input is on RA3
#endif
#if	ver == 876
	btfsc		RA4				;User input is on RA4
#endif
	goto		BP3d
	movlw		0x00
	call		Delay
BP3u
#if	ver == 630
	btfss		RA3				;User input is on RA3
#endif
#if	ver == 876
	btfss		RA4				;User input is on RA4
#endif
	goto		BP3u
	goto		S1
S4End

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;Subroutine:	S5, State 5
;
;Description:
;S1 can enter S5 three different ways--if either of the block protect
;bits are high in the status register or if the write-protect enable bit
;is high.  In any of these three cases, state 5 issues a write status
;register command with the value of 0x02.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S5
	Bank0
	movf		State, W
	movwf		PreviousState
	movlw		0x05
	movwf		State
	call		LEDSOFF
	call		Led6
	call		MegaDelay
	bcf		CS
	call		WriteStatusSPI
	bsf		CS
	btfss		PreviousState, 1
	goto		S1
	goto		S2StatusPoll
S5End

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;Subroutine:	S6, State 6
;
;Description:
;EEPROM is not write enabled, send the write enable command.  Previous
;state must be known when entering this state in order to return to 
;the correct location in the program.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
S6
	Bank0
	movf		State, W
	movwf		PreviousState
	movlw		0x06
	movwf		State
	call		LEDSOFF
	call		Led7
	call		MegaDelay
	bcf		CS
	call		WriteEnableSPI
	bsf		CS
	btfss		PreviousState, 1
	goto		S1
	goto		S2StatusPoll
S6End

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;Subroutine:	WriteSPIHeader
;
;Description:	Sends the Write command to the SPI bus in the following
;					form:
;
;					<0x02><16 bit address>
;
;Notes:
;1.	PageSize must be predefined going into this function.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
WriteSPIHeader
	movlw		0x02
	movwf		SPIBuf
	call		ShiftSPIBuffer
	movf		AddrMSB, W
	movwf		SPIBuf
	call		ShiftSPIBuffer
	movf		AddrLSB, W
	movwf		SPIBuf
	call		ShiftSPIBuffer
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Subroutine: ReadSPIHeader
;   
; Description: Sends the Read command to the SPI bus in the following
;					form:
;
;					<0x03><16 bit address>
;
;					Number of bytes must be specified and can be less than
;					or equal to 128 total bytes for this application.
;
;Notes:
;1.	The 128 byte cap comes from the PIC16F630, not the external EEPROM.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ReadSPIHeader
	movlw		0x03
	movwf		SPIBuf
	call		ShiftSPIBuffer
	movf		AddrMSB, W
	movwf		SPIBuf
	call		ShiftSPIBuffer
	movf		AddrLSB, W
	movwf		SPIBuf
	call		ShiftSPIBuffer
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;Subroutine:	TransferSPIData
;
;Description:	Sends data to the SPI bus in the following form:
;
;					<Data 0><Data 1>...<Data n-1>, where n is the number of 
;					bytes in a page.
;
;					Number of bytes must be specified and can be less than
;					or equal to total bytes in a page for the EEPROM.
;
;Notes:
;1.	PageSize must be predefined going into this function.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TransferSPIData
	Bank0
	movwf		SPIBuf
	call		ShiftSPIBuffer
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Subroutine: WriteEnableSPI
;   
; Description: Sends the Write Enable command to the SPI bus in the following
;					form:
;
;					<0x06>
;
;Notes:
;1.	
;   
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
WriteEnableSPI
	movlw		0x06
	movwf		SPIBuf
	call		ShiftSPIBuffer
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Subroutine: WriteDisableSPI
;   
; Description: Sends the Write Disable command to the SPI bus in the following
;					form:
;
;					<0x04>
;
;Notes:
;1.	
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
WriteDisableSPI
	movlw		0x04
	movwf		SPIBuf
	call		ShiftSPIBuffer
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Subroutine: ReadStatusSPI
;   
; Description: Sends the Read Status Register command to the SPI bus in the 
;					following form:
;
;					<0x05><1 byte input>
;
;Notes:
;1.	
;   
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ReadStatusSPI
	movlw		0x05
	movwf		SPIBuf
	call		ShiftSPIBuffer
	clrf		SPIBuf				;Dummy byte
	call		ShiftSPIBuffer
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Subroutine: WriteStatusSPI
;   
; Description: Sends the Read Status Register command to the SPI bus in the 
;					following form:
;
;					<0x01><1 byte output>
;
;Notes:
;1.	
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
WriteStatusSPI
	movlw		0x01
	movwf		SPIBuf
	call		ShiftSPIBuffer
	movlw		0x02
	movwf		SPIBuf
	call		ShiftSPIBuffer
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Subroutine: ShiftSPIBuffer
;   
; Description: This routine is used to shift data out of the microcontroller
;					onto the SPI bus.
;
;Notes:
;1.	This routine assumes data is in SSPBuf already.
;2.	Number of bytes must be specified in ByteCount prior to entry.	
;3.	Basically, control the ports as follows:
;		Clear chip select
;		Clear clock
;Loop
;		Set or clear output pin.
;		Set clock
;		Read input pin
;		Clear clock
;		Goto Loop 8 times
;		Set chip select
;Count00
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ShiftSPIBuffer
	clrf		Count00
	bcf		SCK
ShiftLoop
	bcf		SCK
	rlf		SPIBuf, f
	btfss		STATUS, C
	bcf		SO
	btfsc		STATUS, C
	bsf		SO
	bsf		SCK
	btfss		SI
	bcf		STATUS, C
	btfsc		SI
	bsf		STATUS, C
	bcf		SCK
	incf		Count00, f
	btfss		Count00, 3
	goto		ShiftLoop
	rlf		SPIBuf, f
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Subroutine: DATA_EEPROM_READ
;
; Description: To read an EEPROM data memory location, the address is
;   written to the EEADR register and set control bit RD (EECON1<0>) to
;   initiate a read. Data is available in the EEDATA register the next
;   clock cycle.
;
; Constants: none
;   
; Global Variables: none
;   
; Initialization: W contains EEPROM address to be read
;   
; Output: W contains EEPROM data
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
DATA_EEPROM_READ
#if	ver == 876
	Bank3
#endif
#if	ver == 630
	Bank1
#endif
	bsf		EECON1, RD          ; initiate EEPROM read
#if	ver == 876
	Bank2
#endif
	movf		EEDATA, W           ; move data to W
	Bank0
	return

#if	ver == 630
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED1 ~ D0
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led1
	Bank1						
	movlw		LED1TRIS
	movwf		TRISA
	Bank0			
	movlw		LED1ON
	movwf		PORTA
	return
	
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED2 ~ D1
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led2	
	Bank1				
	movlw		LED2TRIS
	movwf		TRISA
	Bank0				
	movlw		LED2ON
	movwf		PORTA
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED3 ~ D2
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led3	
	Bank1				
	movlw		LED3TRIS
	movwf		TRISA
	Bank0				
	movlw		LED3ON
	movwf		PORTA
	return
	
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED4 ~ D3
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led4
	Bank1				
	movlw		LED4TRIS
	movwf		TRISA	
	Bank0				
	movlw		LED4ON
	movwf		PORTA
	return
	
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED5 ~ D4
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led5
	Bank1				
	movlw		LED5TRIS
	movwf		TRISA				
	Bank0				
	movlw		LED5ON
	movwf		PORTA
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED6 ~ D5
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led6
	Bank1				
	movlw		LED6TRIS
	movwf		TRISA			
	Bank0				
	movlw		LED6ON
	movwf		PORTA
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED7 ~ D6
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led7
	Bank1				
	movlw		LED7TRIS
	movwf		TRISA		
	Bank0				
	movlw		LED7ON
	movwf		PORTA
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED8 ~ D7
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led8
	Bank1				
	movlw		LED8TRIS
	movwf		TRISA	
	Bank0				
	movlw		LED8ON
	movwf		PORTA
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;All LED's will be turned off when this is called.
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
LEDSOFF
	Bank1
	movlw		LEDOFFTRIS
	movwf		TRISA
	Bank0
	movlw		LEDOFF
	movwf		PORTA
	return
#endif

#if	ver == 876
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED1 ~ D0
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led1
	Bank1
	clrf		TRISB
	Bank0			
	movlw		0x01
	movwf		PORTB
	return
	
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED2 ~ D1
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led2
	Bank1
	clrf		TRISB
	Bank0			
	movlw		0x02
	movwf		PORTB
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED3 ~ D2
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led3
	Bank1
	clrf		TRISB
	Bank0				
	movlw		0x03
	movwf		PORTB
	return
	
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED4 ~ D3
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led4
	Bank1
	clrf		TRISB
	Bank0				
	movlw		0x04
	movwf		PORTB
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED5 ~ D4
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led5
	Bank1
	clrf		TRISB
	Bank0				
	movlw		0x05
	movwf		PORTB
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED6 ~ D5
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led6
	Bank1
	clrf		TRISB
	Bank0				
	movlw		0x06
	movwf		PORTB
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED7 ~ D6
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led7
	Bank1
	clrf		TRISB
	Bank0				
	movlw		0x07
	movwf		PORTB
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;LED8 ~ D7
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Led8
	Bank1
	clrf		TRISB
	Bank0				
	movlw		0x08
	movwf		PORTB
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;All LED's will be turned off when this is called.
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
LEDSOFF
	Bank1
	clrf		TRISB
	Bank0
	clrf		PORTB
	return
#endif

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;Delay(W) ~ DelayTime = [ (1)+(2)+(2)+(W*768~W)+(W*3~1)+(2) ]*(OSC/4)cycles
;           (This includes the call & The movlw)
;         ~ Max Time When W=0xFF, [ 196356 Cycles * (OSC/4) ]
;         ~ Must Declare INNER & OUTER AS GPR'S
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Delay
	movwf		OUTER
	clrf		INNER
D1	decfsz	INNER,f
	goto		D1
D2	decfsz	OUTER,f
	goto		D1
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;MegaDelay
;Holds processor up in very long loop for visual indicators.
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
MegaDelay
	Bank0
	movlw		0x02
	movwf		Count01
OuterLoop
	movlw		0x03
	movwf		Count02
InnerLoop
	movlw		0x00
	call		Delay
	decfsz	Count02, f
	goto		InnerLoop
	decfsz	Count01, f
	goto		OuterLoop
	return

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;ErrorHandler
;Holds processor up in infinite loop due to erroneous checksum calculation.
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ErrorHandler
	call		Led1
	call		Led2
	call		Led3
	call		Led4
	call		Led5
	call		Led6
	call		Led7
	call		Led8
	goto		ErrorHandler

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Data EEPROM Memory (Section 8.0)
;
; PIC16F630 devices have 128 bytes of data EEPROM with address
; range 0x00 to 0x7F.
; Initialize Data EEPROM Memory locations
	ORG     0x2100
	DE      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
	DE      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
	DE      0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70
	DE      0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0

	DE      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
	DE      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
	DE      0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70
	DE      0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0

	DE      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
	DE      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
	DE      0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70
	DE      0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0

	DE      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
	DE      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
	DE      0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70
	DE      0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; Calibrating the Internal Oscillator (Section 9.2.5.1)
; Oscillator Calibration Register (OSCCAL) (Section 2.2.2.7)
;
; The below statements are placed here so that the program can be
; simulated with MPLAB SIM or emulated with the ICD2 or ICE~2000.  
;
; The programmer (PICkit or PROMATE II) will save the actual OSCCAL 
; value in the device and restore it. The value below WILL NOT be
; programmed into the device.
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	org		0x3ff
	retlw		0x80                ; Center Frequency

	end                         ; end of program directive
	
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
