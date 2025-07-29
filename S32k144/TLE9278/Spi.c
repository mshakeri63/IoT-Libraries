/*
 * Spi.c
 *
 *  Created on: Jan 6, 2025
 *      Author: MohammadShakeri
 */

#include "S32K144.h" /* include peripheral declarations S32K144 */
#include "Spi.h"
uint8_t addressspi = 0x81;  // Example 8-bit value 1
uint8_t value = 0x23;  // Example 8-bit value 2
uint16_t tx_16bits=0x8124;
uint16_t LPSPI1_16bits_read;  /* Returned data in to SPI */

 void WDOG_disable (void){
	 WDOG->CNT=0xD928C520;     /*Unlock watchdog*/
	 WDOG->TOVAL=0x0000FFFF;   /*Maximum timeout value*/
	 WDOG->CS = 0x00002100;    /*Disable watchdog*/
 }
 void PORT_init (void) {
	 //tx_16bits =((uint16_t)addressspi << 8) | value;
	 PCC->PCCn[PCC_PORTB_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTB */
	 PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTB */
	 PCC->PCCn[PCC_PORTE_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTB */
	 PORTB->PCR[14]|=PORT_PCR_MUX(3); /* Port B14: MUX = ALT3, LPSPI1_SCK */
	 PORTB->PCR[15]|=PORT_PCR_MUX(3); /* Port B15: MUX = ALT3, LPSPI1_SIN */
	 PORTB->PCR[16]|=PORT_PCR_MUX(3); /* Port B16: MUX = ALT3, LPSPI1_SOUT */
	 //PORTA->PCR[6]|=PORT_PCR_MUX(3); /* Port E1: MUX = ALT3, LPSPI1_PCS3 */
	 PORTB->PCR[17]|=PORT_PCR_MUX(3); /* Port B17: MUX = ALT3, LPSPI1_PCS3 */
	 //PORTE->PCR[1]|=PORT_PCR_MUX(3); /* Port B17: MUX = ALT3, LPSPI1_PCS3 */
	// PORTD->PCR[1]|=PORT_PCR_MUX(3); /* Port B15: MUX = ALT3, LPSPI1_SIN */
	 //PORTE->PCR[0]|=PORT_PCR_MUX(3); /* Port B16: MUX = ALT3, LPSPI1_SOUT */
	// PORTD->PCR[2]|=PORT_PCR_MUX(3); /* Port B16: MUX = ALT3, LPSPI1_SOUT */
 }

void LPSPI1_init_master(void) {
PCC->PCCn[PCC_LPSPI1_INDEX] = 0;          /* Disable clocks to modify PCS ( default) */
PCC->PCCn[PCC_LPSPI1_INDEX] = 0xC6000000; /* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) */
LPSPI1->CR    = 0x00000000;   /* Disable module for configuration */
LPSPI1->IER   = 0x00000000;   /* Interrupts not used */
LPSPI1->DER   = 0x00000000;   /* DMA not used */
LPSPI1->CFGR0 = 0x00000000;   /* Defaults: */
/* RDM0=0: rec'd data to FIFO as normal */
/* CIRFIFO=0; Circular FIFO is disabled */
/* HRSEL, HRPOL, HREN=0: Host request disabled */
LPSPI1->CFGR1 = 0x00000001;   /* Configurations: master mode*/
/* PCSCFG=0: PCS[3:2] are enabled */
/* OUTCFG=0: Output data retains last value when CS negated */
/* PINCFG=0: SIN is input, SOUT is output */
/* MATCFG=0: Match disabled */
/* PCSPOL=0: PCS is active low */
/* NOSTALL=0: Stall if Tx FIFO empty or Rx FIFO full */
/* AUTOPCS=0: does not apply for master mode */
/* SAMPLE=0: input data sampled on SCK edge */
/* MASTER=1: Master mode */
//LPSPI1->TCR   = 0x5300000F;   /* Transmit cmd: PCS3, 16bits, prescale func'l clk by 4. */
LPSPI1->TCR   = 0x5380000F;//| (1 << 24);
/* CPOL=0: SCK inactive state is low */
/* CPHA=1: Change data on SCK lead'g, capture on trail'g edge*/
/* PRESCALE=2: Functional clock divided by 2**2 = 4 */
/* PCS=3: Transfer using PCS3 */
/* LSBF=0: Data is transferred MSB first */
/* BYSW=0: Byte swap disabled */
/* CONT, CONTC=0: Continuous transfer disabled */
/* RXMSK=0: Normal transfer: rx data stored in rx FIFO */
/* TXMSK=0: Normal transfer: data loaded from tx FIFO */
/* WIDTH=0: Single bit transfer */
/* FRAMESZ=15: # bits in frame = 15+1=16 */
LPSPI1->CCR   = 0x04090808;   /* Clk dividers based on prescaled func'l clk of 100 nsec */
/* SCKPCS=4: SCK to PCS delay = 4+1 = 5 (500 nsec) */
/* PCSSCK=4: PCS to SCK delay = 9+1 = 10 (1 usec) */
/* DBT=8: Delay between Transfers = 8+2 = 10 (1 usec) */
/* SCKDIV=8: SCK divider =8+2 = 10 (1 usec: 1 MHz baud rate) */
LPSPI1->FCR   = 0x00000003;   /* RXWATER=0: Rx flags set when Rx FIFO >0 */
/* TXWATER=3: Tx flags set when Tx FIFO <= 3 */
LPSPI1->CR    = 0x00000009;
/* Enable module for operation */
/* DBGEN=1: module enabled in debug mode */
/* DOZEN=0: module enabled in Doze mode */
/* RST=0: Master logic not reset */
/* MEN=1: Module is enabled */ }
void LPSPI1_transmit_16bits (uint16_t send) {
	while((LPSPI1->SR & LPSPI_SR_TDF_MASK)>>LPSPI_SR_TDF_SHIFT==0);                                   /* Wait for Tx FIFO available */
	LPSPI1->TDR = send;              /* Transmit data */
	LPSPI1->SR |= LPSPI_SR_TDF_MASK; /* Clear TDF flag */ }

uint16_t LPSPI1_receive_16bits (void) {  uint16_t receive = 0;
while((LPSPI1->SR & LPSPI_SR_RDF_MASK)>>LPSPI_SR_RDF_SHIFT==0);                                   /* Wait at least one RxFIFO entry */
receive= LPSPI1->RDR;            /* Read received data */
LPSPI1->SR |= LPSPI_SR_RDF_MASK; /* Clear RDF flag */
return receive;                  /* Return received data */ }




void SOSC_init_8MHz(void) {
	SCG->SOSCDIV=0x00000101;  /* SOSCDIV1 & SOSCDIV2 =1: divide by 1 */
	SCG->SOSCCFG=0x00000024;  /* Range=2: Medium freq (SOSC between 1MHz-8MHz)*/
	/* HGO=0:   Config xtal osc for low power */
	/* EREFS=1: Input is external XTAL */
	while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK); /* Ensure SOSCCSR unlocked */
	SCG->SOSCCSR=0x00000001;  /* LK=0:          SOSCCSR can be written */
	/* SOSCCMRE=0:    OSC CLK monitor IRQ if enabled */
	/* SOSCCM=0:      OSC CLK monitor disabled */
	/* SOSCERCLKEN=0: Sys OSC 3V ERCLK output clk disabled */
	/* SOSCLPEN=0:    Sys OSC disabled in VLP modes */
	/* SOSCSTEN=0:    Sys OSC disabled in Stop modes */
	/* SOSCEN=1:      Enable oscillator */
	while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK)); /* Wait for sys OSC clk valid */
}
void SPLL_init_160MHz(void) {
	while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); /* Ensure SPLLCSR unlocked */
	SCG->SPLLCSR = 0x00000000;  /* SPLLEN=0: SPLL is disabled (default) */
	SCG->SPLLDIV = 0x00000302;  /* SPLLDIV1 divide by 2; SPLLDIV2 divide by 4 */
	SCG->SPLLCFG = 0x00180000;  /* PREDIV=0: Divide SOSC_CLK by 0+1=1 */
	/* MULT=24:  Multiply sys pll by 4+24=40 */
	/* SPLL_CLK = 8MHz / 1 * 40 / 2 = 160 MHz */
	while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); /* Ensure SPLLCSR unlocked */
	SCG->SPLLCSR = 0x00000001; /* LK=0:        SPLLCSR can be written */
	/* SPLLCMRE=0:  SPLL CLK monitor IRQ if enabled */
	/* SPLLCM=0:    SPLL CLK monitor disabled */
	/* SPLLSTEN=0:  SPLL disabled in Stop modes */
	/* SPLLEN=1:    Enable SPLL */
	while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)); /* Wait for SPLL valid */
}
void NormalRUNmode_80MHz (void) {  /* Change to normal RUN mode with 8MHz SOSC, 80 MHz PLL*/
	SCG->RCCR=SCG_RCCR_SCS(6)      /* PLL as clock source*/
			|SCG_RCCR_DIVCORE(0b01)      /* DIVCORE=1, div. by 2: Core clock = 160/2 MHz = 80 MHz*/
			|SCG_RCCR_DIVBUS(0b01)       /* DIVBUS=1, div. by 2: bus clock = 40 MHz*/
			|SCG_RCCR_DIVSLOW(0b10);     /* DIVSLOW=2, div. by 3: SCG slow, flash clock= 26 2/3 MHz*/
	while (((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT ) != 6) {}                                 /* Wait for sys clk src = SPLL */
}

void TLE_Control(uint16_t input) {
    LPSPI1_transmit_16bits(input);
    LPSPI1_16bits_read = LPSPI1_receive_16bits();
}
