
#define _DISABLE_ARDUINO_TIMER1_INTERRUPT_HANDLER_  // Disable the default Arduino TIMER1 ISR

#include <Arduino.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <EEPROM.h>
#include <TM1637Display.h>

// Module connection pins (Digital Pins)
#define CLK1 12
#define DIO1 2
#define CLK2 7
#define DIO2 4
#define CURR_INPUT A0
#define POT_INPUT A2

#define THREE_PH 0
#define ONE_PH 1
#define PWM_RUNNING 2
#define PWM_NOT_RUNNING 1
#define PWM_NOT_SET 0
#define DEADTIME_ADD  5
#define DEADTIME_SUB  2

// Millisecond timings for button and switch
#define SHORT_CLICK     10
#define LONG_CLICK      500
#define POT_SWITCH_SAMPLES  2
#define BOOT_CAP_CHARGE_TIME   160
#define RELAY_CHARGE_WAIT      2000000
#define DISPLAY_BLINK   100
#define MIN_PWM_VAL   6
#define BUTTON_IS_PRESSED      (!((PINC >> PINC4) & 1))
#define POT_SWITCH_IS_ON       (!((PINC >> PINC3) & 1))

const uint8_t ONE_PHASE[] = {SEG_B | SEG_C, SEG_A | SEG_B | SEG_E | SEG_F | SEG_G, SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, 0};
const uint8_t THREE_PHASE[] = {SEG_A | SEG_B | SEG_C | SEG_D | SEG_G, SEG_A | SEG_B | SEG_E | SEG_F | SEG_G, SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, 0};
const uint8_t SPACE[] = {0};

const float Sine[] = {125.0, 179.0, 223.0, 246.0, 246.0, 223.0, 179.0, 125.0, 71.0, 27.0, 6.0, 6.0, 27.0, 71.0, 125.0};
const uint8_t Sine_Len = 15;              // Sine table length
const uint8_t Min_Freq = 20;              // Minimal demanded sine wave frequency
const uint8_t Max_Freq = 120;             // Maximal demanded sine wave frequency
const uint16_t Base_Freq = 1041;          // [Hz] Maximal frequency if the sine wave array index is incremented every OVF occurrence
const float Min_Amp = 0.1;                // Minimal allowed sine wave amplitude
const float Max_Amp = 1.0;                // Maximal allowed sine wave amplitude
const float V_f = 3.8333;                 // V/f value. ~230[VAC] w/ 60[Hz]
const float VBus = 230.0;                 // AC voltage [VAC]
bool Phase_Config = 0;                    // 0: 3 phase, 1: 1 phase
bool Config_Editable = 0;                 // Is the configuration editable or not (Between 2 long clicks). 0: No, 1: Yes
int16_t Sine_Used[] = {125, 179, 223, 246, 246, 223, 179, 125, 71, 27, 6, 6, 27, 71, 125};
uint8_t Click_Type = 0;                   // 1: Short, 2: Long
uint8_t PWM_Running = PWM_NOT_SET;        // Indicates if the PWM is operating or not. 2 is running, 1 is not, initialized to 0 to indicate not yet set.
uint32_t Timer = 0;                       // Timer, increments every loop
uint32_t Click_Timer = 0;                 // Increments while button is clicked
uint32_t Pot_Switch_Off_Timer = 0;        // Increments while potentiometer switch is OFF
uint32_t Pot_Switch_On_Timer = 0;         // Increments while potentiometer switch is ON
uint32_t Display_Timer = 0;               // Used for delay of the blinking display (When configurable)
uint32_t Timer_Temp = 0;                  // Used to make sure of consecutive executions
uint32_t Timer_Temp1 = 0;                 // Used to make sure of consecutive executions
uint32_t Init_PWM_Counter = 0;            // Used for charging the bootstrap capacitors, source below
uint16_t Curr_Value = 0;                  // Current value measured in [mA]
uint8_t Sine_Index = 0;                   // 3 sine wave indices are used to allow for phase shifted sine waves.
uint8_t Sine_Index_120 = Sine_Len / 3;
uint8_t Sine_Index_240 = (Sine_Len * 2) / 3;       // Sine_Len must be lower than 128, otherwise, change eq.
uint8_t OVF_Counter = 0;                  // Increments every Timer1 overflow                                          
uint8_t OVF_Counter_Compare = 0;          // Compare OVF_Counter to this value (Base freq / desired freq).

TM1637Display Display1(CLK1, DIO1);
TM1637Display Display2(CLK2, DIO2);

void setup() {
  // Load the latest chosen phase configuration, unless this is the first time.
  if (EEPROM.read(256) == 123) {
    EEPROM.get(0, Phase_Config);             // Set Phase_Config to first byte in EEPROM.
  }
  cli();                                      // Disable interrupts
  CLKPR = (1 << CLKPCE);                      // Enable change of the clock prescaler
  CLKPR = (1 << CLKPS0);                      // Set system clock prescaler to 2. Beforehand DT had to be increased to a large value due to IPM, and at low amplitudes distorted sine wave. When reducing the prescaler, this allows for the DT value to be small.
  sei();
  Display1.setBrightness(0x02);
  Display2.setBrightness(0x02);
  Display1.clear();
  Display2.clear();
  PORTC = (1 << PORTC3) | (1 << PORTC4);      // Activates internal pull up for PC3 (ADC3) and PC4 (ADC4). Default pin state is input. Potentiometer switch and button respectively   
  DDRD = (1 << DDD6) | (1 << DDD5) | (1 << DDD3); // Sets the OC1A, OC1B and OC2B pins to outputs (Default is LOW)
  DDRB = (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0); // Sets the OC2A, OC1B and OC1A pins to outputs (Default is LOW)
                                                                // And sets PB0 pin to output (Default is LOW). Commands the relay
  Wait_A_Bit(RELAY_CHARGE_WAIT);              // Using this function since delay() doesn't seem to work correctly when ISR is activated. Not needed to delay if interrupts are enabled.
                                              // Waiting this delay to let the capacitors charge up. Requires ~3 seconds.
  PORTB = (1 << PORTB0);                      // Set output pin to the relay high, to bypass the high power resistor after the caps were sufficiently charged
}

void loop() {   
  // Local variables
  uint8_t Desired_Freq;                                 // Desired sine wave freq [Hz]
  uint8_t OVF_Counter_Compare_Temp;                     // Used temporarily for OVF_Counter_Compare
  float Amp;                                            // Sine wave amplitude

  // Calculate variables
  Curr_Value = analogRead(CURR_INPUT) << 3;                 // A value of 1023 (5V) -> 8000[mA]. Multiply the analog reading by 8. Gives a resolution of ~8[mA]
  Desired_Freq = ((uint8_t)(analogRead(POT_INPUT) >> 3));   // A value of 1023 (5V) -> 128[Hz]. Divide result by 8 to get value in Hz. Gives resolution of 1[Hz]
  if (Desired_Freq < Min_Freq) Desired_Freq = Min_Freq;
  else if (Desired_Freq > Max_Freq) Desired_Freq = Max_Freq;
  OVF_Counter_Compare_Temp = (uint8_t)(Base_Freq / Desired_Freq); // Decides after how many interrupts should I increment the look-up table index.
  noInterrupts();
  OVF_Counter_Compare = OVF_Counter_Compare_Temp;        // If ISR interrupts in middle of calculation, may give completely false OVF_Counter_Compare value. May be problematic.
  interrupts();

  Amp = ((float)(Desired_Freq) * V_f) / VBus;                                                          
  if (Amp < Min_Amp) Amp = Min_Amp;      
  else if (Amp > Max_Amp) Amp = Max_Amp;   
  noInterrupts();
  for (int i = 0; i < Sine_Len; i++) {
    Sine_Used[i] = (int16_t)(Amp * Sine[i]);
  }
  interrupts();

  // Run functions
  Pot_Switch_State_Check();
  if (PWM_Running != PWM_RUNNING) Button_Click();
  if (PWM_Running != PWM_NOT_SET) Display(PWM_Running, Config_Editable, Desired_Freq);
  Timer++;
}

void Pot_Switch_State_Check() {    
  if (POT_SWITCH_IS_ON) {      
    if (PWM_Running != PWM_RUNNING) {    // If PWM isn't running (If it was already running, no need to update anything).
      if (Timer - Timer_Temp > 1) Pot_Switch_On_Timer = 0;    // To make sure these increments are consecutive
      else Pot_Switch_On_Timer++;      
      Timer_Temp = Timer;
      Pot_Switch_Off_Timer = 0;
      if (Pot_Switch_On_Timer > POT_SWITCH_SAMPLES) {      // If potentiometer switch was ON a sufficient amount of time
        Pwm_Config();      
        Pot_Switch_On_Timer = 0;
        Pot_Switch_Off_Timer = 0;      
        Display1.clear();
        Display2.clear();
      }         
    }
  } else {           // Potentiometer switch OFF
    if (PWM_Running != PWM_NOT_RUNNING) {     // If PWM is running (If it was already not running, no need to update anything).
      if (Timer - Timer_Temp > 1) Pot_Switch_Off_Timer = 0;      // To make sure these increments are consecutive
      else Pot_Switch_Off_Timer++;      
      Timer_Temp = Timer;
      Pot_Switch_On_Timer = 0;
      if (Pot_Switch_Off_Timer > POT_SWITCH_SAMPLES) {     // If potentiometer switch was OFF a sufficient amount of time
        Pwm_Disable();
        Pot_Switch_On_Timer = 0;
        Pot_Switch_Off_Timer = 0;
        Display1.clear();
        Display2.clear();
      } 
    }
  }    
}

void Button_Click() {
  if (BUTTON_IS_PRESSED) {
    if (Timer - Timer_Temp1 > 1) Click_Timer = 0;    // To make sure these increments are consecutive
    else Click_Timer++;
    Timer_Temp1 = Timer;
    if (Click_Timer == LONG_CLICK) {
      Config_Editable = !Config_Editable;    // Toggle
    }
  } else if (Click_Timer > SHORT_CLICK && Click_Timer < LONG_CLICK) {
    if (Config_Editable) {
      Phase_Config = !Phase_Config;        // Toggle
      EEPROM.write(0, Phase_Config);      // Save latest value to EEPROM 
      EEPROM.write(256, 123);              // Update that the Phase_Config was saved to EEPROM (Write a value of 123 to byte 256, arbitrary numbers)
    }
    Click_Timer = 0;
  } else Click_Timer = 0;
}

void Display(uint8_t PWM_Running, bool Blink, uint8_t Desired_Freq) {
  if (PWM_Running == PWM_RUNNING) {
    // Display 1, displays desired frequency in [Hz]
    if (Desired_Freq > 99) Display1.showNumberDec(Desired_Freq, false, 3, 1);
    else {
      Display1.setSegments(SPACE, 1, 1);
      Display1.showNumberDec(Desired_Freq, false, 2, 2);         
    }
    // Display 2, displays measured current in [mA]
    if (Curr_Value > 999) Display2.showNumberDec(Curr_Value, false, 4, 0);
    else {
      Display2.setSegments(SPACE, 1, 0);
      Display2.showNumberDec(Curr_Value, false, 3, 1);         
    }
  } else {
    Display_Timer++;
    if (Blink && (Display_Timer == DISPLAY_BLINK)) {
      Display1.clear();
      Display2.clear();
    } else if (Display_Timer > (2 * DISPLAY_BLINK)) {
      if (Phase_Config) Display1.setSegments(ONE_PHASE);
      else Display1.setSegments(THREE_PHASE); 
      Display_Timer = 0;   
    }
  }
}

void Wait_A_Bit(uint32_t Executions_To_Wait) {
  volatile uint32_t Timer_Temp2 = 0;
  while (Timer_Temp2 < Executions_To_Wait) {
    Timer_Temp2++;
  }
}

void Pwm_Disable() {
  PWM_Running = PWM_NOT_RUNNING;
  cli();
  Init_PWM_Counter = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  sei();    
}

void Pwm_Config() {
  if (Phase_Config == THREE_PH) {
    cli();                      // Disable interrupts
    PWM_Running = PWM_RUNNING;
    
    GTCCR = (1 << TSM) | (1 << PSRASY) | (1 << PSRSYNC); // Halt all timers
    // Timer 1
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10); // Clear OC1A and set OC1B counting up. Waveform mode 1 (Table 14-8)
    TCCR1B = (1 << CS10);       // No prescaler
    TIMSK1 = (1 << TOIE1);      // Timer/Counter1 Overflow Interrupt Enable
    OCR1A = 0;     // Sign determined by set or clear at count-up. High-side IGBT OFF.
    OCR1B = 127;   // Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
    // Timer 2
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20); // Clear OC2A and set OC2B counting up. Waveform mode 1 (Table 14-8)
    TCCR2B = (1 << CS20);       // No prescaler
    OCR2A = 0;     // Sign determined by set or clear at count-up. High-side IGBT OFF.
    OCR2B = 127;   // Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
    
    TCNT1 = 0;     // Set timer1 to 0
    TCNT2 = 0;     // Set timer2 to 0      
    GTCCR = 0;     // Release all timers
    Init_PWM_Counter = 0;
    Wait_A_Bit(BOOT_CAP_CHARGE_TIME);
    sei();      
  } else if (Phase_Config == ONE_PH) {
    PORTB &= 0xF5;                                   // Clear the 2nd (OC1A) and 4th (OC2A) bits. Set High-Side transistors LOW.
    PORTB |= (1 << PORTB2);                          // Set the 3rd (OC1B) bit. Set Low-Side transistor HIGH. Should be connected to GND.
    PORTD |= (1 << PORTD3);                          // Set the 4th bit (OC2B). Set Low-Side transistor HIGH. Should be connected to GND.
    cli();                      // Disable interrupts
    PWM_Running = PWM_RUNNING;

    GTCCR = (1 << TSM) | (1 << PSRASY) | (1 << PSRSYNC); // Halt all timers
    // Timer 1
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10); // Clear OC1A and set OC1B counting up. Waveform mode 1 (Table 14-8)
    TCCR1B = (1 << CS10);       // No prescaler
    TIMSK1 = (1 << TOIE1);      // Timer/Counter1 Overflow Interrupt Enable
    OCR1A = 0;     // Sign determined by set or clear at count-up. High-side IGBT OFF.
    OCR1B = 127;   // Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.
    // Timer 2
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20); // Clear OC2A and set OC2B counting up. Waveform mode 1 (Table 14-8)
    TCCR2B = (1 << CS20);       // No prescaler
       OCR2A = 0;     // Sign determined by set or clear at count-up. High-side IGBT OFF.
    OCR2B = 127;   // Sign determined by set or clear at count-up. Low-side IGBT 50% duty cycle to charge bootstrap cap.

    TCNT1 = 0;     // Set timer1 to 0
    TCNT2 = 0;     // Set timer2 to 0      
    GTCCR = 0;     // Release all timers
    Init_PWM_Counter = 0;
    Wait_A_Bit(BOOT_CAP_CHARGE_TIME);
    sei();
  }
}

ISR(TIMER1_OVF_vect) {   
  OVF_Counter++;   
  if (OVF_Counter >= OVF_Counter_Compare) {
    if (Sine_Index == Sine_Len) Sine_Index = 0;
    if (Sine_Index_120 == Sine_Len) Sine_Index_120 = 0;
    if (Sine_Index_240 == Sine_Len) Sine_Index_240 = 0;    

    if ((Sine_Used[Sine_Index] - DEADTIME_SUB) < MIN_PWM_VAL) {
      OCR1A = 0;
    } else {
      OCR1A = uint8_t(Sine_Used[Sine_Index] - DEADTIME_SUB);
    }
    OCR1B = OCR1A + DEADTIME_ADD;

    if ((Sine_Used[Sine_Index_120] - DEADTIME_SUB) < MIN_PWM_VAL) {
      OCR2A = 0;
    } else {
      OCR2A = uint8_t(Sine_Used[Sine_Index_120] - DEADTIME_SUB);    
    }
    OCR2B = OCR2A + DEADTIME_ADD;

    if ((Sine_Used[Sine_Index_240] - DEADTIME_SUB) < MIN_PWM_VAL) {
      OCR2A = 0;
    } else {
      OCR2A = uint8_t(Sine_Used[Sine_Index_240] - DEADTIME_SUB);
    }
    OCR2B = OCR2A + DEADTIME_ADD;

    OVF_Counter = 0;
    Sine_Index++;
    Sine_Index_120++;
    Sine_Index_240++;
  }
}

