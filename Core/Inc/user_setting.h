#ifndef USER_SETTING_H_
#define USER_SETTING_H_

#define RD_PORT GPIOA
#define RD_PIN  GPIO_PIN_0
#define WR_PORT GPIOA
#define WR_PIN  GPIO_PIN_1
#define CD_PORT GPIOA          // RS PORT
#define CD_PIN  GPIO_PIN_4     // RS PIN
#define CS_PORT GPIOB
#define CS_PIN  GPIO_PIN_0
#define RESET_PORT GPIOC
#define RESET_PIN  GPIO_PIN_1

#define D0_PORT GPIOA
#define D0_PIN GPIO_PIN_9
#define D1_PORT GPIOC
#define D1_PIN GPIO_PIN_7
#define D2_PORT GPIOA
#define D2_PIN GPIO_PIN_10
#define D3_PORT GPIOB
#define D3_PIN GPIO_PIN_3
#define D4_PORT GPIOB
#define D4_PIN GPIO_PIN_5
#define D5_PORT GPIOB
#define D5_PIN GPIO_PIN_4
#define D6_PORT GPIOB
#define D6_PIN GPIO_PIN_10
#define D7_PORT GPIOA
#define D7_PIN GPIO_PIN_8

#define  WIDTH    ((uint16_t)320)
#define  HEIGHT   ((uint16_t)480)

/****************** delay in microseconds ***********************/
extern TIM_HandleTypeDef htim1;
void delay (uint32_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}

// configure macros for the data pins.

/* First of all clear all the LCD_DATA pins i.e. LCD_D0 to LCD_D7
 * We do that by writing the HIGHER bits in BSRR Register
 *
 * For example :- To clear Pins B3, B4 , B8, B9, we have to write GPIOB->BSRR = 0b0000001100011000 <<16
 *
 *
 *
 * To write the data to the respective Pins, we have to write the lower bits of BSRR :-
 *
 * For example say the PIN LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
 *
 * GPIOB->BSRR = (data & (1<<4)) << 3.  Here first select 4th bit of data (LCD_D4), and than again shift left by 3 (Total 4+3 =7 i.e. PB7)
 *
 * GPIOB->BSRR = (data & (1<<6)) >> 4.  Here first select 6th bit of data (LCD_D6), and than again shift Right by 4 (Total 6-4 =2 i.e. PB2)
 *
 *
 */

#define AMASK ((1<<9)|(1<<10)|(1<<8))        //#0, #2, #7
#define BMASK ((1<<3)|(1<<5)|(1<<4)|(1<<10)) //#3, #4, #5, #6
#define CMASK ((1<<7))

#define write_8(d) { \
        GPIOA->REGS(BSRR) = AMASK << 16; \
        GPIOB->REGS(BSRR) = BMASK << 16; \
        GPIOC->REGS(BSRR) = CMASK << 16; \
        GPIOA->REGS(BSRR) = (  ((d) & (1<<0)) << 9) \
                            | (((d) & (1<<2)) << 8) \
                            | (((d) & (1<<7)) << 1); \
        GPIOB->REGS(BSRR) = (  ((d) & (1<<3)) << 0) \
                            | (((d) & (1<<4)) << 1) \
                            | (((d) & (1<<5)) >> 1) \
                            | (((d) & (1<<6)) << 4); \
        GPIOC->REGS(BSRR) = (  ((d) & (1<<1)) << 6); \
    }

  /* To read the data from the Pins, we have to read the IDR Register
   *
   * Take the same example say LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
   *
   * To read data we have to do the following
   *
   * GPIOB->IDR & (1<<7) >> 3. First read the PIN (1<<7 means we are reading PB7) than shift it to the position, where it is connected to
   * and in this example, that would be 4 (LCD_D4). (i.e. 7-3=4)
   *
   * GPIOB->IDR & (1<<2) << 4. First read the PIN (1<<2 means we are reading PB2) than shift it to the position, where it is connected to
   * and in this case, that would be 6 (LCD_D6). (i.e. 2+4= 6). Shifting in the same direction
   *
   */
#define read_8() (       (  (  (GPIOA->REGS(IDR) & (1<<9)) >> 9) \
                            | ((GPIOC->REGS(IDR) & (1<<7)) >> 6) \
                            | ((GPIOA->REGS(IDR) & (1<<10)) >> 8) \
                            | ((GPIOB->REGS(IDR) & (1<<3)) >> 0) \
                            | ((GPIOB->REGS(IDR) & (1<<5)) >> 1) \
                            | ((GPIOB->REGS(IDR) & (1<<4)) << 1) \
                            | ((GPIOB->REGS(IDR) & (1<<10)) >> 4) \
                            | ((GPIOA->REGS(IDR) & (1<<8))  >> 1)))

/************************** For 48 MHZ ****************************/
#define WRITE_DELAY { }
#define READ_DELAY  { }

#endif /* USER_SETTING_H_ */
