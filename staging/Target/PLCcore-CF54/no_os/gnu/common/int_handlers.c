/*
 * File:		int_handlers.c
 * Purpose:		Interrupt handlers
 *
 */

#include "../include/common.h"
#include "../include/io.h"

/********************************************************************/

/* Called by asm_exception_handler */
void 
exception_handler (void *framep) 
{
	/*
	 * This is the exception handler for all defined exceptions.  Most
	 * exceptions do nothing, but some of the more important ones are
	 * handled to some extent.
	 */

	switch (MCF5XXX_RD_SF_FORMAT(framep))
	{
		case 4:
		case 5:
		case 6:
		case 7:
			break;
		default:
			printf("\nIllegal stack type! -- PC = %#08X\n", \
				MCF5XXX_SF_PC(framep));
			break;
	}

	switch (MCF5XXX_RD_SF_VECTOR(framep))
	{
		case 2:
			printf("Access Error -- PC = %#08X\n", MCF5XXX_SF_PC(framep));
			switch (MCF5XXX_RD_SF_FS(framep))
			{
				case 4:
					printf("Error on instruction fetch\n");
					break;
				case 8:
					printf("Error on operand write\n");
					break;
				case 9:
					printf("Attempted write to write-protected space\n");
					break;
				case 12:
					printf("Error on operand read\n");
					break;
				default:
					printf("Reserved Fault Status Encoding\n");
					break;
			}
			break;
		case 3:
			printf("Address Error -- PC = %#08X\n", MCF5XXX_SF_PC(framep));
			switch (MCF5XXX_RD_SF_FS(framep))
			{
				case 4:
					printf("Error on instruction fetch\n");
					break;
				case 8:
					printf("Error on operand write\n");
					break;
				case 9:
					printf("Attempted write to write-protected space\n");
					break;
				case 12:
					printf("Error on operand read\n");
					break;
				default:
					printf("Reserved Fault Status Encoding\n");
					break;
			}
			break;
		case 4:
			printf("Illegal instruction -- PC = %#08X\n", MCF5XXX_SF_PC(framep));
			break;
		case 8:
			printf("Privilege violation -- PC = %#08X\n", MCF5XXX_SF_PC(framep));
			break;
		case 9:
			printf("Trace Exception -- PC = %#08X\n", MCF5XXX_SF_PC(framep));
			break;
		case 10:
			printf("Unimplemented A-Line Instruction -- PC = %#08X\n", \
				MCF5XXX_SF_PC(framep));
			break;
		case 11:
			printf("Unimplemented F-Line Instruction -- PC = %#08X\n", \
				MCF5XXX_SF_PC(framep));
			break;
		case 12:
			printf("Debug Interrupt -- PC = %#08X\n", MCF5XXX_SF_PC(framep));
			break;
		case 14:
			printf("Format Error -- PC = %#08X\n", MCF5XXX_SF_PC(framep));
			break;
		case 15:
			printf("Unitialized Interrupt -- PC = %#08X\n", \
				MCF5XXX_SF_PC(framep));
			break;
		case 24:
			printf("Spurious Interrupt -- PC = %#08X\n", \
				MCF5XXX_SF_PC(framep));
			break;
		case 25:
		case 26:
		case 27:
		case 28:
		case 29:
		case 30:
		case 31:
			printf("Autovector interrupt level %d\n",
				MCF5XXX_RD_SF_VECTOR(framep) - 24);
			break;
		case 32:
		case 33:
		case 34:
		case 35:
		case 36:
		case 37:
		case 38:
		case 39:
		case 40:
		case 41:
		case 42:
		case 43:
		case 44:
		case 45:
		case 46:
		case 47:
			printf("TRAP #%d\n", MCF5XXX_RD_SF_VECTOR(framep) - 32);
			break;
		case 5:
		case 6:
		case 7:
		case 13:
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 48:
		case 49:
		case 50:
		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
		case 56:
		case 57:
		case 58:
		case 59:
		case 60:
		case 61:
		case 62:
		case 63:
			printf("Reserved: #%d\n",
				MCF5XXX_RD_SF_VECTOR(framep));
			break;
		case 65: /* Eport Interrupt 1 */
		case 66: /* Eport Interrupt 2 */
		case 67: /* Eport Interrupt 3 */
		case 68: /* Eport Interrupt 4 */
		case 69: /* Eport Interrupt 5 */
		case 70: /* Eport Interrupt 6 */
		case 71: /* Eport Interrupt 7 */

			/* 
			 * Clear the interrupt source 
			 * This clears the flag for edge triggered interrupts
			 */
			MCF_EPORT_EPFR = (uint8)(0x01 << (MCF5XXX_RD_SF_VECTOR(framep) - 64));
			printf("Edge Port Interrupt #%d\n",MCF5XXX_RD_SF_VECTOR(framep) - 64);
			break;	
		default:
			printf("User Defined Vector #%d\n",
				MCF5XXX_RD_SF_VECTOR(framep));
			break;
	}
}

/********************************************************************/

void irq_handler (void) 
{
	/* 
	 * This is the catch-all interrupt handler for all user defined
	 * interrupts.  To create specific handlers, create a new interrupt
	 * handler and change vectors.s to point to the new handler.
	 */
	printf("irq_handler\n");
}

/********************************************************************/
