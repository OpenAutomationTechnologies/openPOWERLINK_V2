/* File Name: m5485_reg.h
*************************************************************************
*                                                                       *
*   Project:    Coldfire core			                        *
*   Purpose:    A structure of the M5407's register map                 *
*                                                                       *
************************************************************************/
#ifndef __M5485_REG_H__
#define __M5485_REG_H__

typedef unsigned char	uint8_t;
typedef unsigned short	uint16_t;
typedef unsigned long	uint32_t;

/* there are a lot of places where this is usefull */
typedef struct
{
	uint8_t a;
	uint16_t b;
} __attribute ((packed)) uint24_t;

/* The chipselect configuration registers for a single chipselect */
struct m5407cs
{
	volatile uint16_t	csar;
	volatile uint16_t	unused0;
	volatile uint32_t	csmr;
	volatile uint16_t	unused1;
	volatile uint16_t	cscr;
} __attribute((packed));

/* The timer configuration registers for a single timer */
struct m5407tmr
{
	volatile uint16_t	tmr;
	volatile uint16_t	u0; /* unused 0 */
	volatile uint16_t	trr;
	volatile uint16_t	u1;
	volatile uint16_t	tcr;
	volatile uint16_t	u2;
	volatile uint16_t	tcn;
	volatile uint24_t	u3;
	volatile uint8_t	ter;
	volatile uint16_t	u4[23];
} __attribute((packed));

/* The UART registers */
struct m5407uart
{
	volatile uint8_t	umr;
	volatile uint24_t	x0; /* unused */
	volatile uint8_t 	usr;
	volatile uint24_t	x1;
	volatile uint8_t	ucr;
	volatile uint24_t	x2;
	volatile uint8_t	urb;
	volatile uint24_t	x3;
	volatile uint8_t	uipcr;
	volatile uint24_t	x4;
	volatile uint8_t	uisr;
	volatile uint24_t	x5;
	volatile uint8_t	udu;
	volatile uint24_t	x6;
	volatile uint8_t	udl;
	volatile uint24_t	x7;
	volatile uint32_t	x8[5];
	volatile const uint8_t	uip;
	volatile uint24_t	x10;
	volatile uint8_t	uop1;
	volatile uint24_t	x11;
	volatile uint8_t	uop0;
	volatile uint24_t	x12;
} __attribute((packed));

struct m5407dma
{
	volatile uint32_t	sar;
	volatile uint32_t	dar;
	volatile uint32_t	dcr;
	volatile uint32_t	bcr;
	volatile uint8_t	dsr;
	volatile uint24_t	u0;
	volatile uint8_t	divr;
	volatile uint8_t	u1[43];
} __attribute((packed));

struct m5407reg
{
	volatile uint8_t	rsr;
	volatile uint8_t	sypcr;
	volatile uint8_t	swivr;
	volatile uint8_t	swsr;
	volatile uint16_t	par;
	volatile uint8_t	irqpar;
	volatile uint8_t	unused0;
	volatile uint8_t	pllcr;
	volatile uint24_t	unused1;
	volatile uint8_t	mpark;
	volatile uint24_t	unused2;

	volatile uint32_t	unused3[12];

	volatile uint32_t	ipr;
	volatile uint32_t	imr;
	volatile uint24_t	unused4;
	volatile uint8_t	avr;
	volatile uint8_t	icr[10];

	volatile uint16_t	unused5[21];

	struct m5407cs		csr[8];

	volatile uint32_t	unused6[8];

	volatile uint16_t	dcr;
	volatile uint16_t	unused7;
	volatile uint32_t	unused8;
	volatile uint32_t	dacr0;
	volatile uint32_t	dmr0;
	volatile uint32_t	dacr1;
	volatile uint32_t	dmr1;

	volatile uint32_t	unused9[10];

	struct m5407tmr		timer[2];

	struct m5407uart	uart[2];

	volatile uint32_t	unused10;

	volatile uint16_t	paddr;
	volatile uint16_t	unused11;
	volatile uint16_t	padat;

	volatile uint16_t	unused12[27];

	volatile uint8_t	iadr;
	volatile uint24_t	unused13;
	volatile uint8_t	ifdr;
	volatile uint24_t	unused14;
	volatile uint8_t	i2cr;
	volatile uint24_t	unused15;
	volatile uint8_t	i2sr;
	volatile uint24_t	unused16;
	volatile uint8_t	i2dr;

	volatile uint8_t	unused17[111];

	struct m5407dma		dma[4];
} __attribute((packed));
#endif
