#include <ArduinoModbus.h>
#include <Adafruit_MAX31865.h>
#include <TM1637Display.h>
#include <QuickPID.h>
#include <SPI.h>

// Pin definitions
#define MAX31865_CS PA15
#define RS485_TX PB6    // UART1 TX (remapped)
#define RS485_RX PB7    // UART1 RX (remapped)
#define RS485_DE PB8
#define TM1637_CLK1 PC4 // PV display
#define TM1637_DIO1 PA7
#define TM1637_CLK2 PB15 // SV display
#define TM1637_DIO2 PB14
#define BUTTON_SET PA8
#define BUTTON_RS PB13  // AT/RS button
#define BUTTON_UP PB12
#define BUTTON_DW PB2
#define LED_RELAY PC9
#define LED_AT PC8
#define LED_ALARM1 PC3
#define LED_MODBUS PC5
#define RELAY_PIN PC9

// Modbus settings
#define MODBUS_SLAVE_ID 1
#define BAUD_RATE 9600

// Extended register map (2 registers per parameter, IEEE-754 float)
#define REG_COUNT 44
uint16_t holdingRegisters[REG_COUNT];
#define REG_SV 0
#define REG_AL1 2
#define REG_AL2 4
#define REG_AL3 6
#define REG_PS 8
#define REG_P 10
#define REG_I 12
#define REG_D 14
#define REG_HY 16
#define REG_DB 18
#define REG_HY1 20
#define REG_HY2 22
#define REG_HY3 24
#define REG_OLL 26
#define REG_OLH 28
#define REG_FL 30
#define REG_FH 32
#define REG_ANL 34
#define REG_ANH 36
#define REG_PV 38
#define REG_ADB 40
#define REG_BBD 42
#define REG_MODE 44    // Manual/Auto (nP)
#define REG_LOCK 46    // Password Lock (LC)
#define REG_CP 48      // Cooling Proportional (cP)
#define REG_CT 50      // OUT2 Control Cycle (c-t)
#define REG_AD1 52     // Alarm 1 Mode (Ad1)
#define REG_AD2 54     // Alarm 2 Mode (Ad2)
#define REG_AD3 56     // Alarm 3 Mode (Ad3)
#define REG_IP 58      // Input Signal Mode (i-P)
#define REG_DP 60      // Decimal Point (dP)
#define REG_FC 62      // Filter Constant (FC)
#define REG_TF 64      // Temperature Unit (t-F)

// Parameter menu structure
struct MenuItem {
  const char* name;
  uint16_t regAddress;
  float minValue;
  float maxValue;
  float defaultValue;
  bool isSubmenu;
};

MenuItem menuItems[] = {
  {"RL1 ", REG_AL1, -198.9, 850.0, 200.0, false},
  {"RL2 ", REG_AL2, -198.9, 850.0, 600.0, false},
  {"RL3 ", REG_AL3, -198.9, 850.0, 1000.0, false},
  {"PS  ", REG_PS, -100.0, 100.0, 0.0, false},
  {"nP  ", REG_MODE, 0.0, 1.0, 0.0, false},
  {"LC  ", REG_LOCK, 0.0, 9999.0, 0.0, false},
  {"P   ", REG_P, 0.0, 9999.0, 10.0, false},
  {"I   ", REG_I, 0.0, 3600.0, 240.0, false},
  {"d   ", REG_D, 0.0, 3600.0, 60.0, false},
  {"HY2 ", REG_HY, 0.0, 1000.0, 2.0, false},
  {"cP  ", REG_CP, 0.0, 9999.0, 0.0, false},
  {"c-t ", REG_CT, 0.0, 3600.0, 0.0, false},
  {"dB  ", REG_DB, -100.0, 100.0, 5.0, false},
  {"HY1 ", REG_HY1, 0.0, 1000.0, 1.0, false},
  {"Ad1 ", REG_AD1, 0.0, 7.0, 0.0, false},
  {"HY2 ", REG_HY2, 0.0, 1000.0, 1.0, false},
  {"Ad2 ", REG_AD2, 0.0, 7.0, 0.0, false},
  {"HY3 ", REG_HY3, 0.0, 1000.0, 1.0, false},
  {"Ad3 ", REG_AD3, 0.0, 7.0, 0.0, false},
  {"OLL ", REG_OLL, 0.0, 99.0, 0.0, false},
  {"OLH ", REG_OLH, 1.0, 100.0, 100.0, false},
  {"i-P ", REG_IP, 0.0, 3.0, 0.0, false},
  {"FL  ", REG_FL, -198.9, 850.0, -198.9, false},
  {"FH  ", REG_FH, -198.9, 850.0, 850.0, false},
  {"dP  ", REG_DP, 0.0, 3.0, 0.0, false},
  {"FC  ", REG_FC, 0.0, 10.0, 4.0, false},
  {"t-F ", REG_TF, 0.0, 1.0, 0.0, false},
  {"br-L", REG_ANL, -198.9, 850.0, 0.0, false},
  {"br-H", REG_ANH, -198.9, 850.0, 1300.0, false},
  {"bBd ", REG_BBD, 1200.0, 19200.0, 9600.0, false},
  {"AdB ", REG_ADB, 1.0, 247.0, 1.0, false}
};
#define MENU_ITEM_COUNT (sizeof(menuItems) / sizeof(menuItems[0]))

// Submenu indices
#define SUBMENU_OUTPUT 0
#define SUBMENU_CONTROL 5
#define SUBMENU_ALARM1 10
#define SUBMENU_ALARM2 12
#define SUBMENU_ALARM3 14
#define SUBMENU_OUTPUT_LIMIT_LOW 16
#define SUBMENU_OUTPUT_LIMIT_HIGH 17
#define SUBMENU_INPUT 18
#define SUBMENU_COMMUNICATION 27

// MAX31865 object (PT100, 3-wire, hardware SPI)
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(MAX31865_CS);

// TM1637 displays
TM1637Display pvDisplay(TM1637_CLK1, TM1637_DIO1);
TM1637Display svDisplay(TM1637_CLK2, TM1637_DIO2);

// PID variables
float setpoint, input, output;
float Kp = 10.0, Ki = 240.0, Kd = 60.0;
QuickPID myPID(&input, &output, &setpoint, Kp, Ki, Kd, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);

// Menu variables
enum MenuState { MEASURING, SETTING, SUBMENU };
MenuState menuState = MEASURING;
int menuIndex = 0;
bool editing = false;
unsigned long setPressStart = 0;
const unsigned long LONG_PRESS_DURATION = 2000; // 2 seconds
unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// Button states
volatile bool setPressed = false, rsPressed = false, upPressed = false, dwPressed = false;

// Timer
HardwareTimer timer(TIM2);

// Declare Serial1 with custom pins PB6 (TX) and PB7 (RX)
HardwareSerial Serial1(PB6, PB7);

extern "C" {
#include "stm32f1xx_hal.h" // Include STM32 HAL for register access
}

void setup() {
  // Enable AFIO clock and remap UART1 to PB6/PB7
  __HAL_RCC_AFIO_CLK_ENABLE();
  //MODIFY_REG(AFIO->MAPR, 0x00000004, 0x00000004); // Bit 2 = 1 for USART1 remap to PB6/PB7
   __HAL_AFIO_REMAP_USART1_ENABLE();

  // Initialize SPI1 for MAX31865
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  // Initialize pins
  pinMode(LED_RELAY, OUTPUT);
  pinMode(LED_AT, OUTPUT);
  pinMode(LED_ALARM1, OUTPUT);
  pinMode(LED_MODBUS, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_SET, INPUT_PULLUP);
  pinMode(BUTTON_RS, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DW, INPUT_PULLUP);
  pinMode(MAX31865_CS, OUTPUT);
  digitalWrite(MAX31865_CS, HIGH);
  pinMode(RS485_DE, OUTPUT); // Manual DE/RE control
  digitalWrite(RS485_DE, LOW); // Default to receive mode

  // Initialize LEDs
  digitalWrite(LED_RELAY, LOW);
  digitalWrite(LED_AT, HIGH); // Power on
  digitalWrite(LED_ALARM1, LOW);
  digitalWrite(LED_MODBUS, LOW);

  // Initialize MAX31865
  max31865.begin(MAX31865_3WIRE);

  // Initialize TM1637 displays
  pvDisplay.setBrightness(0x0f);
  svDisplay.setBrightness(0x0f);

  // Initialize Serial1 for Modbus (already configured with PB6/PB7)
  Serial1.begin(BAUD_RATE, SERIAL_8N1);

  // Initialize Modbus with Serial1
  if (!ModbusRTUServer.begin(Serial1, MODBUS_SLAVE_ID)) {
    while (1); // Halt if Modbus fails to start
  }
  // Configure holding registers
  for (int i = 0; i < REG_COUNT; i += 2) {
    ModbusRTUServer.configureHoldingRegisters(i / 2, 2); // 2 registers per float
  }

  // Initialize registers with default values
  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    setFloatRegister(menuItems[i].regAddress, menuItems[i].defaultValue);
  }
  setFloatRegister(REG_PV, 0.0); // PV is read-only

  // Initialize PID
  setpoint = getFloatRegister(REG_SV);
  myPID.SetMode(QuickPID::Control::automatic);
  myPID.SetSampleTimeUs(100000); // 100ms
  myPID.SetOutputLimits(getFloatRegister(REG_OLL), getFloatRegister(REG_OLH));

  // Setup button interrupts
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), setISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_RS), rsISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_UP), upISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_DW), dwISR, FALLING);

  // Setup timer interrupt (100ms)
  timer.setOverflow(10000, MICROSEC_FORMAT);
  timer.attachInterrupt(timerISR);
  timer.resume();
}

// Button ISRs
void setISR() {
  if (millis() - lastButtonTime > DEBOUNCE_DELAY) {
    if (digitalRead(BUTTON_SET) == LOW) {
      setPressStart = millis();
    } else if (setPressStart > 0) {
      unsigned long pressDuration = millis() - setPressStart;
      setPressed = (pressDuration < LONG_PRESS_DURATION);
      if (pressDuration >= LONG_PRESS_DURATION) {
        if (menuState == MEASURING) menuState = SETTING;
        else if (menuState == SETTING) {
          switch (menuIndex) {
            case SUBMENU_OUTPUT: menuIndex = 0; break;
            case SUBMENU_CONTROL: menuIndex = 5; break;
            case SUBMENU_ALARM1: menuIndex = 10; break;
            case SUBMENU_ALARM2: menuIndex = 12; break;
            case SUBMENU_ALARM3: menuIndex = 14; break;
            case SUBMENU_OUTPUT_LIMIT_LOW: menuIndex = 16; break;
            case SUBMENU_OUTPUT_LIMIT_HIGH: menuIndex = 17; break;
            case SUBMENU_INPUT: menuIndex = 18; break;
            case SUBMENU_COMMUNICATION: menuIndex = 27; break;
            default: menuState = MEASURING; menuIndex = 0; break;
          }
        } else {
          menuState = MEASURING;
          menuIndex = 0;
        }
      }
      lastButtonTime = millis();
    }
  }
}
void rsISR() {
  if (millis() - lastButtonTime > DEBOUNCE_DELAY) {
    rsPressed = true;
    lastButtonTime = millis();
  }
}
void upISR() {
  if (millis() - lastButtonTime > DEBOUNCE_DELAY) {
    upPressed = true;
    lastButtonTime = millis();
  }
}
void dwISR() {
  if (millis() - lastButtonTime > DEBOUNCE_DELAY) {
    dwPressed = true;
    lastButtonTime = millis();
  }
}

// Timer ISR
void timerISR() {
  updateSensorData();
  if (menuState != MEASURING) return; // Pause control in menu mode
  applyControlLogic();
  updateDisplay();
  // Update Modbus registers
  setFloatRegister(REG_PV, input); // Reflect current PV
  digitalWrite(RS485_DE, HIGH); // Set to transmit
  ModbusRTUServer.poll();
  digitalWrite(RS485_DE, LOW); // Return to receive
}

// Helper: Set float in two registers (IEEE-754)
void setFloatRegister(uint16_t address, float value) {
  uint32_t floatBits;
  memcpy(&floatBits, &value, sizeof(float));
  holdingRegisters[address] = (floatBits >> 16) & 0xFFFF;
  holdingRegisters[address + 1] = floatBits & 0xFFFF;
  ModbusRTUServer.holdingRegisterWrite(address / 2, floatBits); // Write to Modbus
}

// Helper: Get float from two registers
float getFloatRegister(uint16_t address) {
  uint32_t floatBits = ((uint32_t)holdingRegisters[address] << 16) | holdingRegisters[address + 1];
  float value;
  memcpy(&value, &floatBits, sizeof(float));
  return value;
}

// Update sensor data
void updateSensorData() {
  float temperature = max31865.temperature(100, 430); // PT100, Rref=430Ω
  if (max31865.readFault()) {
    max31865.clearFault();
    temperature = 0.0;
  }
  temperature += getFloatRegister(REG_PS);
  input = temperature;
}

// Apply PID control logic
void applyControlLogic() {
  setpoint = getFloatRegister(REG_SV);
  Kp = getFloatRegister(REG_P);
  Ki = getFloatRegister(REG_I);
  Kd = getFloatRegister(REG_D);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.Compute();
  digitalWrite(RELAY_PIN, output > 0 ? HIGH : LOW); // Adjusted to 0 threshold
  digitalWrite(LED_RELAY, output > 0 ? HIGH : LOW);

  // Alarm logic (simplified)
  float pv = getFloatRegister(REG_PV);
  float al1 = getFloatRegister(REG_AL1);
  float hy1 = getFloatRegister(REG_HY1);
  digitalWrite(LED_ALARM1, pv > al1 + hy1 ? HIGH : LOW);
}

// Update displays
void updateDisplay() {
  if (menuState == MEASURING) {
    pvDisplay.showNumberDecEx((int)(getFloatRegister(REG_PV) * 10), 0b01000000, false);
    svDisplay.showNumberDecEx((int)(getFloatRegister(REG_SV) * 10), 0b01000000, false);
  } else {
    uint8_t segments[] = {
      menuItems[menuIndex].name[0],
      menuItems[menuIndex].name[1],
      menuItems[menuIndex].name[2],
      menuItems[menuIndex].name[3]
    };
    pvDisplay.setSegments(segments);
    svDisplay.showNumberDecEx((int)(getFloatRegister(menuItems[menuIndex].regAddress) * 10), 0b01000000, false);
  }
}

// Handle button inputs
void handleButtons() {
  if (setPressed) {
    setPressed = false;
    if (menuState == SETTING) {
      menuIndex = (menuIndex + 1) % MENU_ITEM_COUNT;
      if (menuIndex == SUBMENU_OUTPUT || menuIndex == SUBMENU_CONTROL ||
          menuIndex == SUBMENU_ALARM1 || menuIndex == SUBMENU_ALARM2 ||
          menuIndex == SUBMENU_ALARM3 || menuIndex == SUBMENU_OUTPUT_LIMIT_LOW ||
          menuIndex == SUBMENU_OUTPUT_LIMIT_HIGH || menuIndex == SUBMENU_INPUT ||
          menuIndex == SUBMENU_COMMUNICATION) {
        menuIndex++; // Skip submenu headers
      }
    }
  }
  if (menuState == SETTING && upPressed) {
    upPressed = false;
    float value = getFloatRegister(menuItems[menuIndex].regAddress);
    value += 1.0;
    if (value <= menuItems[menuIndex].maxValue) {
      setFloatRegister(menuItems[menuIndex].regAddress, value);
    }
  }
  if (menuState == SETTING && dwPressed) {
    dwPressed = false;
    float value = getFloatRegister(menuItems[menuIndex].regAddress);
    value -= 1.0;
    if (value >= menuItems[menuIndex].minValue) {
      setFloatRegister(menuItems[menuIndex].regAddress, value);
    }
  }
  if (rsPressed) {
    rsPressed = false;
    if (menuState == MEASURING) {
      float mode = getFloatRegister(REG_MODE);
      setFloatRegister(REG_MODE, 1.0 - mode); // Toggle 0/1
    }
  }
}

void loop() {
  // Handle button inputs
  handleButtons();
}