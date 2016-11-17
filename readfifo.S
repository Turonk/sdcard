@
@ Ported to GNU arm assembler (arm-elf-as)
@ and added comments by Martin Thomas
@

@@		AREA MCIFIFO, CODE, READONLY
		.section .text2, "ax"

@@		EXPORT MCI_ReadFifo
@@		EXPORT MCI_WriteFifo
		.global MCI_ReadFifo
		.global MCI_WriteFifo
		.global mci_fifo
@@		; int MCI_ReadFifo(unsigned int * dest);
@@		; int MCI_WriteFifo(unsigned int * src);

@@src     RN 4
@@dst     RN 5
src .req R4
dst .req R5

@ C-Interface:
@ int MCI_ReadFifo(unsigned int * dest);
@ Read 2*4*8=32 bytes from src = MCI Fifo Address to
@ dest passed as parameter - take care of alignment of dst
		.func MCI_ReadFifo
		MCI_ReadFifo:
		STMFD   sp!,{src,dst,r12,lr}
		LDR     src, =0xE008C080
		MOV     dst, r0
		LDMIA   src, {r0-r3}
		STMIA   dst!, {r0-r3}
		LDMIA   src, {r0-r3}
		STMIA   dst!, {r0-r3}
		MOV     r0, dst
		LDMFD   sp!,{src,dst,r12,pc}
		.endfunc

@ C-Interface:
@ int MCI_WriteFifo(unsigned int * src);
@ Write 2*4*8=32 bytes to dst = MCI Fifo Address from
@ src passed as parameter - take care of alignment of src
		.func MCI_WriteFifo
		MCI_WriteFifo:
		STMFD   sp!,{src,dst,r12,lr}
		LDR     dst, =0xE008C080
		MOV     src, r0
		LDMIA   src!, {r0-r3}
		STMIA   dst, {r0-r3}
		LDMIA   src!, {r0-r3}
		STMIA   dst, {r0-r3}
		MOV     r0, src
		LDMFD   sp!,{src,dst,r12,pc}
		.endfunc

@@mci_fifo
@@		DCD     0xE008C080
mci_fifo: .word  0xE008C080

@@		END
		.end
