#include <STM32RTC.h>

#define LED_PIN PC13
#define SLEEP_INTERVAL 10 // Sleep interval in seconds

// Get RTC instance
STM32RTC& rtc = STM32RTC::getInstance();

void setup() {
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED off (active low on STM32F103C8)
  
  // Initialize RTC with LSI (internal oscillator)
  rtc.setClockSource(STM32RTC::LSI_CLOCK); // Use LSI_CLOCK; change to LSE_CLOCK if 32.768 kHz crystal present
  rtc.begin(STM32RTC::HOUR_24);
  
  // Set initial time (optional, for reference)
  rtc.setHours(0);
  rtc.setMinutes(0);
  rtc.setSeconds(0);
  
  // Set initial alarm
  rtc.setAlarmTime(0, 0, SLEEP_INTERVAL, STM32RTC::ALARM_A);
  rtc.attachInterrupt(alarmCallback, nullptr, STM32RTC::ALARM_A);
}

void loop() {
  // Blink LED 3 times
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);  // LED on
    delay(200);
    digitalWrite(LED_PIN, HIGH); // LED off
    delay(200);
  }
  
  // Set next alarm based on current RTC time
  uint8_t hour, min, sec;
  uint32_t subsec;
  rtc.getTime(&hour, &min, &sec, &subsec);
  rtc.setAlarmTime(hour, min, (sec + SLEEP_INTERVAL) % 60, STM32RTC::ALARM_A);
  
  // Clear wakeup flag
  PWR->CR |= PWR_CR_CWUF;
  // Enable low-power deep sleep (STOP mode)
  PWR->CR |= PWR_CR_LPDS;
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  // Wait for interrupt
  __WFI();
}

void alarmCallback(void* data) {
  // Called when RTC alarm triggers
  // No action needed; loop() resumes after wake-up
}