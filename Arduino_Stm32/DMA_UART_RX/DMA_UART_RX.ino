#include <HardwareSerial.h>

// Define UART and DMA handles
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

// Receive buffer
#define RX_BUFFER_SIZE 32
uint8_t rxBuffer[RX_BUFFER_SIZE];

// Flag to indicate DMA buffer full
volatile bool rxCompleteFlag = false;

// Custom callback for UART RX completion
void My_UART_RxCompleteCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    rxCompleteFlag = true; // Set flag instead of printing directly
  }
}

void setup() {
  // Initialize Serial for debugging (uses USB Serial)
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to connect
  }

  // Initialize UART2
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Serial.println("UART Init Failed");
    while (1);
  }

  // Initialize DMA for UART RX
  __HAL_RCC_DMA1_CLK_ENABLE(); // Enable DMA1 clock
  hdma_usart2_rx.Instance = DMA1_Channel6; // DMA channel for USART2 RX
  hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_rx.Init.Mode = DMA_CIRCULAR; // Circular mode for continuous reception
  hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
    Serial.println("DMA Init Failed");
    while (1);
  }

  // Link DMA to UART
  __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

  // Enable DMA interrupt
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

  // Start DMA reception
  if (HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE) == HAL_OK) {
    Serial.println("DMA RX Started");
  } else {
    Serial.println("DMA RX Failed");
  }
}

void loop() {
  // Check for RX completion
  if (rxCompleteFlag) {
    Serial.println("\nDMA RX Buffer Full");
    rxCompleteFlag = false; // Reset flag
  }

  // Process received data
  for (int i = 0; i < RX_BUFFER_SIZE; i++) {
    if (rxBuffer[i] != 0) { // Check for non-zero data
      Serial.print((char)rxBuffer[i]);
      rxBuffer[i] = 0; // Clear buffer
    }
  }
  delay(100); // Small delay to avoid flooding Serial
}

// DMA interrupt handler
extern "C" void DMA1_Channel6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  // Manually check for transfer complete and call custom callback
  if (__HAL_DMA_GET_FLAG(&hdma_usart2_rx, DMA_FLAG_TC6)) {
    My_UART_RxCompleteCallback(&huart2);
  }
}