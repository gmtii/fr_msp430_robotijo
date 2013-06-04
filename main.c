#include "msp430fr5739.h"
#include "i2cmaster.h"
#include "10dof.h"
#include "lcd.h"
#include "comun.h"
#include "Fraunchpad.h"
   
unsigned char *TXData;                    // Pointer to TX data
unsigned char *RXData;
unsigned char TXByteCtr,RXByteCtr;
unsigned char RXbuffer[6];
unsigned char TXbuffer[6];

const unsigned char LED_Menu[] = {0x00,0x80,0xC0,0xE0,0xF0,0xF8};
volatile unsigned char modo=0;
volatile unsigned int ADCResult = 0;
volatile unsigned char cuenta=0,borra;

void main(void)

{
    WDTCTL = WDTPW + WDTHOLD;

    SystemInit();
    i2c_init();
    
    lcd_init();
    lcd_clear(0);
    lcd_print("Iniciando...",0,0);
    StartUpSequence(); 
	serialstring("Hola mundo!\n");
    //init_sensors();
    //lcd_print("I2C OK",0,3);

    

while(1) 
{
    
     if (cuenta==50)
     {
     P1OUT |= BIT0;
     PJOUT &= ~(BIT0 +BIT1+BIT2+BIT3);                  
     P3OUT &= ~(BIT4 +BIT5+BIT6+BIT7);
     } else if (cuenta>50 && !modo)
     {
     StartDebounceTimer(1); 
     __bis_SR_register(LPM0_bits + GIE);
     __no_operation();
     StartDebounceTimer(1); 
     __bis_SR_register(LPM0_bits + GIE);
     __no_operation();
     } 
     if (borra)
     {
     lcd_clear(0);
     borra&=0;
     }
	modo=3;

    switch (modo) {
  
                  case 0:
                    lcd_print("Temperatura:",0,0);    
                    i2c_init(BMP085_ADDR);
                    print_int(temperatura(),1);
                    lcd_print("Presion (Pa)",0,3);
                    print_int(presion(),4);
                    cuenta++;
                    break;

                  case 1:           
                    get_hmc5883();
                    StartDebounceTimer(0);                       
                   __bis_SR_register(LPM0_bits + GIE);
                   __no_operation();
                    cuenta++;
                    break;

                  case 2:
                    get_adxl345();
                    cuenta++;    
                    break;
                  
                  case 3:      
                    lcd_print("ADXL 335",0,0);
                    adxl335();
                    cuenta++;
                    break;
                    
                  case 4:      
                    get_l3g4200d();
                    cuenta++;
                    
                    break;
                                      
                  case 5:       
                    P1OUT |= BIT0;
                    lcd_print("Menu 5",0,0);
                    __bis_SR_register(LPM0_bits + GIE);
                    __no_operation();
                    break;
                                                                       
                  default:
                    modo=0;
                    break;
                }
      
}           

}
    


//*****************************************************************************
//
// I2C RX/TX
//*****************************************************************************

__attribute__((interrupt(USCI_B0_VECTOR )))
void USCIB0_ISR (void)
{	
  switch(UCB0IV) 
  {
        case 0x00: break;                    // Vector 0: No interrupts break;
        case 0x02: break;
        case 0x04:
          UCB0CTLW0 |= UCTXSTT;             //resend start if NACK
          break;                            // Vector 4: NACKIFG break;
        case 0x16:   
          RXByteCtr--;                            // Decrement RX byte counter
          if (RXByteCtr)
            *RXData++ = UCB0RXBUF;               // Move RX data to address PRxData
          else
          {
            *RXData = UCB0RXBUF;               // Move RX data to address PRxData
            UCB0CTLW0 |= UCTXSTP;
            UCB0IFG &= ~UCRXIFG0;
          __bic_SR_register_on_exit(LPM0_bits);// Exit LPM0
          }
          break;

        case 0x18: 
	  if (TXByteCtr)                    // Check TX byte counter
          {
            UCB0TXBUF = *TXData++;             // Load TX buffer
            TXByteCtr--;                    // Decrement TX byte counter
          }
          else
          {
            UCB0CTLW0 |= UCTXSTP;           // I2C stop condition
            UCB0IFG &= ~UCTXIFG;            // Clear USCI_B0 TX int flag
           __bic_SR_register_on_exit(LPM0_bits);// Exit LPM0
          }
          break;                            // Vector 26: TXIFG0 break;
          
          case 0x1a: P1OUT ^= BIT0;
            break; // Vector 28: BCNTIFG break;

          default: break;
  }
}
	
//*****************************************************************************
//
// Pulsación 
//*****************************************************************************

__attribute__((interrupt(PORT4_VECTOR))) 
void Port_4(void)

{
  // Clear all LEDs
  
  P1OUT &= ~BIT0;                               // enciende la pantalla 
  PJOUT &= ~(BIT0 +BIT1+BIT2+BIT3);                  
  P3OUT &= ~(BIT4 +BIT5+BIT6+BIT7);
  
  switch(P4IV)
    {
      case P4IV_P4IFG0:   
        cuenta&=0;
        borra|=1;
        DisableSwitches();
        StartDebounceTimer(0);
        if (modo<5)
        {
          modo++;
          P3OUT = LED_Menu[modo];
        }
        P4IFG &= ~BIT0;                         // Clear P4.0 IFG
        break;
          
      case P4IV_P4IFG1:
        cuenta&=0;
        borra|=1;
        DisableSwitches();
        StartDebounceTimer(0);
        if (modo)
        {
          modo--;
          P3OUT = LED_Menu[modo];
        }
        P4IFG &= ~BIT1;                     // Clear P4.1 IFG      
        break;
  
      default:
        break;
    }  
}

//*****************************************************************************
//
// Pulsación 
//*****************************************************************************

__attribute__((interrupt(TIMER0_A0_VECTOR)))
void TIMER0_A0_ISR(void)
{
  TA1CCTL0 = 0;
  TA1CTL = 0;
  EnableSwitches();
  __bic_SR_register_on_exit(LPM0_bits);
}

//*****************************************************************************
//
// Pulsación 
//*****************************************************************************

__attribute__((interrupt(ADC10_VECTOR)))
void ADC10_ISR (void)
{
  switch(ADC10IV)
  {
    case ADC10IV_NONE: break;               // No interrupt
    case ADC10IV_ADC10OVIFG: break;         // conversion result overflow
    case ADC10IV_ADC10TOVIFG: break;        // conversion time overflow
    case ADC10IV_ADC10HIIFG: break;         // ADC10HI
    case ADC10IV_ADC10LOIFG: break;         // ADC10LO
    case ADC10IV_ADC10INIFG: break;         // ADC10IN
    case ADC10IV_ADC10IFG: 
             ADCResult = ADC10MEM0;
             __bic_SR_register_on_exit(LPM0_bits);                                              
             break;                          // Clear CPUOFF bit from 0(SR)                         
    default: break; 
  }  
}

// Echo back RXed character, confirm TX buffer is ready first
__attribute__((interrupt(USCI_A0_VECTOR)))
void USCI_A0_ISR(void)
{
	switch(UCA0IV)
	{
		case 0:		break; // Vector 0 - no interrupt
		case 2: 	while (!(UCA0IFG&UCTXIFG)); // USCI_A0 TX buffer ready?
				UCA0TXBUF = UCA0RXBUF; // TX -> RXed character
				break;
		case 4:		break; // Vector 4 - TXIFG
		default:	break;
	}
}
