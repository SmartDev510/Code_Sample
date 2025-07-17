// STM32F103 6-Channel PWM Signal Generation with 48 MHz System Clock
// Using Timer 2 (PA0-PA3) and Timer 3 (PA6-PA7)
// PWM frequency: ~1kHz, adjustable duty cycle
// System clock: 48 MHz with 8 MHz HSE, APB1 prescaler forced to /2

#include <Arduino.h>
#include <HardwareTimer.h>
#include <stm32f1xx.h>  // Include STM32F1 HAL for RCC registers

// Define PWM pins (adjust based on your board's pinout)
#define PWM_PIN_CH1 PA0  // Timer 2, Channel 1
#define PWM_PIN_CH2 PA1  // Timer 2, Channel 2
#define PWM_PIN_CH3 PA2  // Timer 2, Channel 3
#define PWM_PIN_CH4 PA3  // Timer 2, Channel 4
#define PWM_PIN_CH5 PA6  // Timer 3, Channel 1
#define PWM_PIN_CH6 PA7  // Timer 3, Channel 2

// PWM frequency in Hz
#define PWM_FREQUENCY 1000

// Timer prescaler and period calculations
#define SYS_CLK 48000000  // System clock: 48 MHz
#define APB1_CLK (SYS_CLK / 2)  // APB1 clock: 24 MHz
#define TIMER_CLK (APB1_CLK * 2)  // Timer clock: 48 MHz (APB1 * 2 when prescaler > 1)
#define PRESCALER 48      // Timer prescaler for 1 kHz PWM
#define PWM_PERIOD (TIMER_CLK / (PRESCALER * PWM_FREQUENCY))  // Period for 1 kHz

// Timer objects
HardwareTimer *Timer2 = new HardwareTimer(TIM2);
HardwareTimer *Timer3 = new HardwareTimer(TIM3);

// Function to configure system clock to 48 MHz using 8 MHz HSE
void SystemClock_Config() {
  // Enable HSE
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY));  // Wait for HSE to be ready

  // Configure PLL: HSE as source, multiplier = 6 (8 MHz * 6 = 48 MHz)
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);  // Clear PLL settings
  RCC->CFGR |= (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL6);  // HSE as PLL source, PLL * 6

  // Explicitly set APB1 prescaler to /2 (24 MHz), APB2 to /1 (48 MHz)
  RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 = 48 MHz / 2 = 24 MHz

  // Enable PLL
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait for PLL to be ready

  // Set PLL as system clock source
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait for switch

  // Update SystemCoreClock variable
  SystemCoreClockUpdate();
}

void setup() {
  // Configure system clock to 48 MHz
  SystemClock_Config();

  // Initialize serial for debugging (optional)
  Serial.begin(115200);

  // Configure PWM pins as outputs
  pinMode(PWM_PIN_CH1, OUTPUT);
  pinMode(PWM_PIN_CH2, OUTPUT);
  pinMode(PWM_PIN_CH3, OUTPUT);
  pinMode(PWM_PIN_CH4, OUTPUT);
  pinMode(PWM_PIN_CH5, OUTPUT);
  pinMode(PWM_PIN_CH6, OUTPUT);

  // Configure Timer 2 for PWM
  Timer2->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN_CH1);  // Channel 1
  Timer2->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN_CH2);  // Channel 2
  Timer2->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN_CH3);  // Channel 3
  Timer2->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN_CH4);  // Channel 4
  Timer2->setPrescaleFactor(PRESCALER);
  Timer2->setOverflow(PWM_PERIOD - 1, TICK_FORMAT);
  Timer2->resume();

  // Configure Timer 3 for PWM
  Timer3->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN_CH5);  // Channel 1
  Timer3->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN_CH6);  // Channel 2
  Timer3->setPrescaleFactor(PRESCALER);
  Timer3->setOverflow(PWM_PERIOD - 1, TICK_FORMAT);
  Timer3->resume();

  // Set initial duty cycles (0-100%)
  Timer2->setCaptureCompare(1, 25, PERCENT_COMPARE_FORMAT);  // 25% duty cycle
  Timer2->setCaptureCompare(2, 50, PERCENT_COMPARE_FORMAT);  // 50% duty cycle
  Timer2->setCaptureCompare(3, 75, PERCENT_COMPARE_FORMAT);  // 75% duty cycle
  Timer2->setCaptureCompare(4, 10, PERCENT_COMPARE_FORMAT);  // 10% duty cycle
  Timer3->setCaptureCompare(1, 30, PERCENT_COMPARE_FORMAT);  // 30% duty cycle
  Timer3->setCaptureCompare(2, 60, PERCENT_COMPARE_FORMAT);  // 60% duty cycle
}

void loop() {
  // Example: Dynamically change duty cycle for demonstration
  static uint32_t lastUpdate = 0;
  static uint8_t duty = 10;

  if (millis() - lastUpdate > 1000) {
    duty = (duty + 10) % 100;  // Increment duty cycle by 10%, wrap at 100
    Timer2->setCaptureCompare(1, duty, PERCENT_COMPARE_FORMAT);  // Update Channel 1
    Timer2->setCaptureCompare(2, 100 - duty, PERCENT_COMPARE_FORMAT);  // Inverse for Channel 2
    Timer2->setCaptureCompare(3, duty, PERCENT_COMPARE_FORMAT);  // Update Channel 3
    Timer2->setCaptureCompare(4, 100 - duty, PERCENT_COMPARE_FORMAT);  // Inverse for Channel 4
    Timer3->setCaptureCompare(1, duty, PERCENT_COMPARE_FORMAT);  // Update Channel 5
    Timer3->setCaptureCompare(2, 100 - duty, PERCENT_COMPARE_FORMAT);  // Inverse for Channel 6

    // Print current duty cycle for debugging
    Serial.print("Duty Cycle: ");
    Serial.print(duty);
    Serial.println("%");
    
    lastUpdate = millis();
  }
}