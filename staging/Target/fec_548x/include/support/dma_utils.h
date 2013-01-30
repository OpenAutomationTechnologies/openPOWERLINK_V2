/*
 * File:		dma_utils.h
 * Purpose:		
 *
 * Notes:
 */

#ifndef _DMA_UTILS_H_
#define _DMA_UTILS_H_

extern volatile  int dma_flag;

/********************************************************************/

void   dma_irq_enable(uint8, uint8);
void   dma_irq_disable(void);
int    dma_set_initiator(int);
unsigned int dma_get_initiator(int);
void   dma_remove_initiator(int);
int    dma_set_channel(int);
int    dma_get_channel(int);
void   dma_remove_channel(int);
void   dma_interrupt_handler_real(void);
void   dma_interrupt_handler(void);
void   dma_handler(void) ;

int    dma_connect(int channel, int address);
int    dma_disconnect(int channel);
void   dma_remove_channel_by_number(int channel);

/********************************************************************/

/*
 * Create identifiers for each initiator/requestor
 */
#define DMA_ALWAYS      (0)
#define DMA_DSPI_RX     (1)
#define DMA_DSPI_TX     (2)
#define DMA_DREQ0       (3)
#define DMA_PSC0_RX     (4)
#define DMA_PSC0_TX     (5)
#define DMA_USBEP0      (6)
#define DMA_USBEP1      (7)
#define DMA_USBEP2      (8)
#define DMA_USBEP3      (9)
#define DMA_PCI_TX      (10)
#define DMA_PCI_RX      (11)
#define DMA_PSC1_RX     (12)
#define DMA_PSC1_TX     (13)
#define DMA_I2C_RX      (14)
#define DMA_I2C_TX      (15)
#define DMA_FEC0_RX     (16)
#define DMA_FEC0_TX     (17)
#define DMA_FEC1_RX     (18)
#define DMA_FEC1_TX     (19)
#define DMA_DREQ1       (20)
#define DMA_CTM0        (21)
#define DMA_CTM1        (22)
#define DMA_CTM2        (23)
#define DMA_CTM3        (24)
#define DMA_CTM4        (25)
#define DMA_CTM5        (26)
#define DMA_CTM6        (27)
#define DMA_CTM7        (28)
#define DMA_USBEP4      (29)
#define DMA_USBEP5      (30)
#define DMA_USBEP6      (31)
#define DMA_PSC2_RX     (32)
#define DMA_PSC2_TX     (33)
#define DMA_PSC3_RX     (34)
#define DMA_PSC3_TX     (35)
#define DMA_FEC_RX(x)   ((x == 0) ? DMA_FEC0_RX : DMA_FEC1_RX)
#define DMA_FEC_TX(x)   ((x == 0) ? DMA_FEC0_TX : DMA_FEC1_TX)

/********************************************************************/

#endif /* _DMA_UTILS_H_ */
