#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>



#define BAUD 115200
#include <util/setbaud.h>

/*
 * Poner un unico bit a 1 sin afectar a a ningun otro del registro
 * registro = registro OR (1 << Bit_name Registro)
 * Registro |= ( 1 << Bit_Name del registro)
 * 
 * Poner un unico bit a 0 sin afectar a resto de bits del registro
 * registro = registro AND NOT(1 << Bit_name Registro)
 * Registro &= ~( 1 << Bit_Name del registro)
 *
*/
#define sbi(REG,BIT) (REG |= (1 << BIT))
#define cbi(REG,BIT) (REG &= ~(1 << BIT))

#define setOutput(ddr, pin) ((ddr) |= (1 << (pin)))
#define setLow(port, pin) ((port) &= ~(1 << (pin)))
#define setHigh(port, pin) ((port) |= (1 << (pin)))
#define pulse(port, pin) do { setHigh((port), (pin)); setLow((port), (pin)); } while (0)
#define outputState(port, pin) ((port) & (1 << (pin)))

#define ADC_BUFFER_SIZE 1024

volatile uint16_t adc_counter;
volatile uint8_t adc_data[ADC_BUFFER_SIZE];

volatile int16_t stop_index = -1;
volatile bool freeze = false;
volatile uint16_t timer = 0;
volatile uint8_t comp = 0xFF;

void init_adc(void);
void init_comparator(void);
void start_adc(void);
void stop_adc(void);

void init_adc(void) {
/**
 **** ADMUX(ADC Multiplexer selection register)
 *       REFS1:0 Bit: 7:6 Reference selecion
 *          these  bits select the reference voltaje for the adc
 *       ADLAR Bit 5 ADC  Left Adjust Results
 *          these bit dictates either the left bits or the right bits of the result 
 *          register ADCH:ADCL that are used to store the result. if we write a one
 *          to ADLAR, the result will be left adjusted; otherwise, the result is
 *          right adjusted.
 *       MUX4:0 Bit 4:0 Analog Chanel and Gain selection Bits 
 *          the value of these bits selects the gain for the differential channels 
 *          and also selects which combination of analog inputs are connected to the
 *          ADC.
 *   
 **** ADCRSA( Control and status register A) 
 *      ADEN bit 7: ADC enable
 *          these bit in one enable the ADC, by writing 0, the ADC its turned
 *          of
 *      ADSC bit 6: ADC start conversion
 *          these bit in one by Single, and free running mode starts the conversion.
 *          the first conversion performs the initialization of the adc.
 *      ADATE bit 5: ADC auto trigger enable
 *          in one enables the auto triggering of the adc, the adc will start 
 *          the conversion on a positive edge, of the select trigger signal,
 *          the trigger source select from the bits ADTS in ADCRSB
 *      ADIF bit 4: ADC interrupt flag
 *          these bit is set when an adc conversion completes and data registers
 *          are update. ADIF its cleared by hardware if ADIE bit and the I-bit
 *          in sreg are set, alternative ADIF its cleared by writing a logical
 *          one to the flag. 
 *      ADIE bit 3: ADC interrupt Enable
 *          When this bit is set one, and the I-bit in SREG is set, the ADC 
 *          conversion complete interrupt is activated.
 *      ADPSn bit 2:0 : ADC Prescaler select[n=2:0]
 *          these bit determine the division factor between the system clock 
 *          frequency and the input clock to the ADC
 *
 *          ADPS[2:0]   |   Division Factor
 *          000         |    2
 *          001         |    2
 *          010         |    4
 *          011         |    8
 *          100         |    16
 *          101         |    32
 *          110         |    64 
 *          111         |    128
 *    
*/
    cbi(ADCSRA, ADEN); // disable adc
    cbi(ADCSRA, ADSC); // stop conversion

    cbi(ADMUX, REFS1); // choose AVcc with external cap
    sbi(ADMUX, REFS0); // for adc voltage reference
    sbi(ADMUX, ADLAR); // left adjust adc readings for 8 bit
    ADMUX |= ( 0 & 0x07 ); // choose analog 0 pin

    sbi(ADCSRA, ADATE); // enable auto trigger of adc
    sbi(ADCSRA, ADIE); // enable adc interrupt

    cbi(ADCSRA, ADPS2); // set prescaler to 4
    sbi(ADCSRA, ADPS1); //
    cbi(ADCSRA, ADPS0); //

    cbi(ADCSRB, ACME); // choose AIN1 for comparator

    cbi(ADCSRB, ADTS2); // choose free running mode


    cbi(ADCSRB, ADTS1); // for auto trigger adc
    cbi(ADCSRB, ADTS0);

    
}

void start_adc(void)
{
    sbi( ADCSRA, ADEN );
    sbi( ADCSRA, ADSC );
}

void stop_adc(void)
{
    cbi( ADCSRA, ADEN );
    cbi( ADCSRA, ADSC );
}

void init_comparator(void)
{
    cbi( ACSR, ACD ); // turn on comparator
    cbi( ACSR, ACBG ); // choose digital pin 7 for comparator
    cbi( ACSR, ACIE ); // disable interrupt

    cbi( ACSR, ACIC ); // disable input capture interrupt
}


ISR(ADC_vect)
{
    adc_data[adc_counter] = ADCH; // read adc
    if (freeze) {
        freeze = false;
        stop_index = -1;
    } else if (adc_counter == stop_index) {
        freeze = true;
    } else if ( comp == 0 && (ACSR & (1 << ACO)) && stop_index < 0) {
        stop_index = ((adc_counter + (ADC_BUFFER_SIZE >> 1) ) & 0x03FF);
    }
    adc_counter = (( adc_counter + 1 ) & 0x03FF); // increment adc counter
    timer = timer + 1;
    comp = (comp << 1); // push next comparator value
    comp += (ACSR & (1 << ACO));
}

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8 bit
    UCSR0B = _BV(RXEN0) | _BV(TXEN0); // enable rx and tx

}

void uart_putchar(char c) {
    loop_until_bit_is_set(UCSR0A, UDRE0); // wait until reg empty
    UDR0 = c;
}

char uart_getchar(void) {
    loop_until_bit_is_set(UCSR0A, RXC0); // wait for byte
    return UDR0;
}

void init_pwm(void)
{
    /**
     * 
     *  TCCR2A(TC2 Control register A) : 
     *      COM2An bits 7:6 : Compare Ouput Mode for Chanel A[n=1:0]
     *          these bits control the ouput compare pin(OC2A) behavior. If one
     *          both of the COM2A[1:0] bits are set to one.
     *          Normal PWM mode
     *          COM2A1  |    COM2A0 |   Description
     *          0       |    0      |     Normal Port operation, OC2A disconnected
     *          0       |    1      |     Toggle OC2A on compare match
     *          1       |    0      |     Clear OC2A on compare match
     *          1       |    1      |     set OC2A on compare match
     *          Fast PWM mode set by WGM2[1:0] bits 
     *          COM2A1  |    COM2A0 |   Description
     *          0       |    0      |    Normal Port operation, OC2A disconnected
     *          0       |    1      |    WGM22  = 0: Normal Port operation, OC2A disconnected 
     *                  |           |    WGM22  = 1: Toggle OC2A on compare match
     *          1       |    0      |    Clear OC2A on compare match, Set OC2A at bottom(non-inverting mode)
     *          1       |    1      |    set OC2A on compare match, Clear OC2A at bottom(inverting mode)
     *          Phase correct PWM mode set by WGM2[2:0] bits
     *          COM2A1  |    COM2A0 |   Description
     *          0       |    0      |    Normal Port operation, OC2A disconnected
     *          0       |    1      |    WGM22  = 0: Normal Port operation, OC2A disconnected 
     *                  |           |    WGM22  = 1: Toggle OC2A on compare match
     *          1       |    0      |    Clear OC2A on compare match when up-counting. Set OC2A on compare match when down-counting
     *          1       |    1      |    set OC2A on compare match, Clear OC2A on compare match when down counting 
     *      
     *      COM2Bn bits 5:4 : compare output mode for channel b[n=1:0]
     *          these bits control the output compare pin (OC2B) behavior, if
     *          one or both of the COM2B[1:0] bit are set, the OC2B output
     *          overrides the normal port functionality of the i/0 pint it is connected to.
     *          ** The DATA Direction register corresponding to the OC2B pin must be set 
     *          in order to enable the output  driver.
     *          Behavior tables:  
     *          COM2B1  |   COM2B0  | Description
     *          Non-pwm: 
     *          0       |   0       | Normal port Operation, OC2A disconnected
     *          0       |   1       | Toggle OC2A on compare match
     *          1       |   0       | clear OC2A on compare match 
     *          1       |   1       | Set OC2A on compare match 
     *          Fast-PWM
     *          0       |   0       | Normal port Operation, OC2A disconnected
     *          0       |   1       | WGM22  = 0: Normal Port operation, OC2A disconnected 
     *                                WGM22  = 1: Toggle OC2A on compare match
     *          1       |   0       | clear OC2A on compare match, Set OC2A at Bottom (non-iverting modE)
     *          1       |   1       | Set OC2A on compare match, clear OC2A at Bottom (iverting modE)
     *          Phase Correct-PWM
     *          0       |   0       | Normal port Operation, OC2A disconnected
     *          0       |   1       | WGM22  = 0: Normal Port operation, OC2A disconnected 
     *                                WGM22  = 1: Toggle OC2A on compare match
     *          1       |   0       | Clear OC2A on compare match when up-counting. Set OC2A on compare match when down-counting
     *          1       |   1       | set OC2A on compare match, Clear OC2A on compare match when down counting
     *     WGM2n bit 1:0: waveform generation modE[n = 1:0]
     *          Combined with the WGM22 on TCCR2B register, these bits control the 
     *          counting sequence of the counter, the source for maximum (TOP) counter
     *          value, and what type of waveform generation to be used.
     *          the modes are Normal Mode(counter), Clear Timer one compare Match CTC,
     *          and two types of pulse width modulation(PWM) modes.
     *
     *          **refer table atmega328 avr micro datashet at 205p.
     *
     *
     *          
     *
     *  TCCR2B( TC2 control register B)
     *      FOC2A bit 7 : Force Ouput Compare A
     *          The FOC2A bit its only active when the WGM bits specify a non-PWM
     *          mode.
     *          these bit determine based on COM2A[1:0], a strobe function on OC2B
     *      FOC2B bit 6: Force output compare B
     *          The FOC2A bit its only active when the WGM bits specify a non-PWM
     *          mode.
     *      WGM22 bit  3: waveform generation mode
     *          these bit conf, needs to refer to TCCR2A
     *      CS2[2:0] bit 2:0: Clock select 2 [n = 0..2]
     *          the three clock select bits select the clock source to be used
     *          by the timer/counter.
     *      
     *
     */
       
    setOutput(DDRD, PORTD3);//set data direction register to  PORTD3
    sbi(TCCR2A, COM2B1);// Clear OC2B on compare match when up-counting, set OC2B on compare Match when down-counting
    sbi(TCCR2A, WGM20);// PWM, Phase correct mode
    sbi(TCCR2B, CS22);// 
    OCR2B = 128;
}

void read_cmd(void)
{
    timer = 0;
    while (!(freeze || timer > 1023)); // wait for the adc to freeze
    stop_adc();

    int i;
    for (i = 0; i < ADC_BUFFER_SIZE - adc_counter; i++)
        uart_putchar(((char *)adc_data)[adc_counter + i]);
    for (i = 0; i < adc_counter; i++)
        uart_putchar(((char *)adc_data)[i]);

    start_adc();
}

void trigger_val_cmd(void)
{
    uint8_t val = 0;
    val += ((uart_getchar() - 48) * 100);
    val += ((uart_getchar() - 48) * 10);
    val += ((uart_getchar() - 48) * 1);
    OCR2B = val;
}

int main(void) {

    uart_init();

    sei(); // enable interrupts

    init_adc();
    init_comparator();
    init_pwm();

    start_adc();
    while (true) {

        char command = uart_getchar();

        switch (command) {
            case 'p':
                uart_putchar('p');
                break;
            case 'r':
                read_cmd();
                break;
            case 't':
                trigger_val_cmd();
                break;
        }

    }

    return 0;

}
