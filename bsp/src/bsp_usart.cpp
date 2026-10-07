#include "bsp_usart.hpp"

extern UART_HandleTypeDef huart1, huart6;       // 串口句柄
extern DMA_HandleTypeDef hdma_usart1_rx;        // 4 个 DMA 句柄
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx; 

void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

static uint8_t usart1_rx_buf[128],usart6_rx_buf[128];
void USART6_Init()
{
  // TODO: 配置 USART6 的 DMA 接收、发送及空闲中断，并启动接收。
  __HAL_LINKDMA(&huart6,hdmarx,hdma_usart6_rx);
  __HAL_LINKDMA(&huart6,hdmatx,hdma_usart6_tx);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart6, usart6_rx_buf, sizeof(usart6_rx_buf));
}

void USART1_Init()
{
  // TODO: 配置 USART1 的 DMA 接收、发送及空闲中断，并启动接收。
  __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);
  __HAL_LINKDMA(&huart1,hdmatx,hdma_usart1_tx);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, usart1_rx_buf, sizeof(usart1_rx_buf));
}

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  // TODO: 检查参数，并按 mode 选择阻塞、DMA 或中断发送。
  if(huart == nullptr || pData == nullptr || Size == 0)return;
  switch (mode){
    case USART_MODE_BLOCK:
      HAL_UART_Transmit(huart,pData,Size,100);
      break;
    case USART_MODE_DMA:
      HAL_UART_Transmit_DMA(huart,pData,Size);
      break;
    case USART_MODE_IT:
      HAL_UART_Transmit_IT(huart,pData,Size);
      break;
    default:
      break;
  }
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  // TODO: 选择接收方式并启动 HAL UART 接收。
  if(huart == nullptr || pData == nullptr || Size == 0)return;
  HAL_UART_Receive_DMA(huart,pData,Size);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size){
  if(huart == &huart1)
    HAL_UART_Receive_DMA(&huart1,usart1_rx_buf,sizeof(usart1_rx_buf));
  else if(huart == &huart6)
    HAL_UART_Receive_DMA(&huart6,usart6_rx_buf,sizeof(usart6_rx_buf));
}
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart == &huart1)
    HAL_UART_Receive_DMA(&huart1,usart1_rx_buf,sizeof(usart1_rx_buf));
  else if(huart == &huart6)
    HAL_UART_Receive_DMA(&huart6,usart6_rx_buf,sizeof(usart6_rx_buf));
}
