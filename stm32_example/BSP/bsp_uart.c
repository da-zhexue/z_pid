#include "bsp_uart.h"
#include "usart.h"
#include "bsp_imu.h"
#include "bsp_odom.h"
#include "control_task.h"

float PID_V[3] = {1.0, 0.0, 0.0}, IOUT_MAX_V = 500.0, OUT_MAX_V = 2000.0;
float PID_P[3] = {1.0, 0.0, 0.0}, IOUT_MAX_P = 500.0, OUT_MAX_P = 2000.0;

uint8_t uart3_rx_buffer[REC_MESSAGE_LEN] = 
{
	0xAA, 0x55, 0x0A,             // 帧头 数据长度
	0x04, 0x00, 0x00, 0x00, 0x00, // kp_p
	0x04, 0x00, 0x00, 0x00, 0x00, // ki_p
	0x04, 0x00, 0x00, 0x00, 0x00, // kd_p
	0x04, 0x00, 0x00, 0x00, 0x00, // maxout_p
	0x04, 0x00, 0x00, 0x00, 0x00, // imaxout_p
	0x04, 0x00, 0x00, 0x00, 0x00, // kp_v
	0x04, 0x00, 0x00, 0x00, 0x00, // ki_v
	0x04, 0x00, 0x00, 0x00, 0x00, // kd_v
	0x04, 0x00, 0x00, 0x00, 0x00, // maxout_v
	0x04, 0x00, 0x00, 0x00, 0x00, // imaxout_v
	0x00, 0x5D                    // 校验位 帧尾
};
uint8_t uart3_tx_buffer[SEND_MESSAGE_LEN] = 
{
	0xAA, 0x55, 0x20, 0x01,                        // 帧头 数据长度 running
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// motor_v1~4
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            // vx vy vz
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            // x y z
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,            // roll pitch yaw
	0xDE, 0x5D                                     // 校验位 帧尾
};

int running = 0;

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
  return ch;
}

void uart_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); 
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); 
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

}	

int Rec_decode(uint8_t *buffer, uint16_t size)
{
	if (buffer[0] != 0xAA || buffer[1] != 0x55) return -1;
	uint8_t checksum = 0x00;
	for(int i = 0; i < size-2; i++)
		checksum ^= buffer[i];
	if(checksum != buffer[size - 2]) return -2;
	uint8_t data_count = buffer[2];
	running = 1;
	custom_field_t received_fields[data_count];
	uint16_t offset = 3;
	
	for (int i = 0; i < data_count; i++) {
        if (offset >= size - 2) { 
            return -5; 
        }
        
        uint8_t type_id = buffer[offset];
        offset++; 
        
        received_fields[i].type_id = type_id; 

        switch (type_id) {
            case 0x00: // int16 (小端序)
                if (offset + 2 > size - 2) return -5;
                received_fields[i].value.i16_val = (int16_t)((buffer[offset+1] << 8) | buffer[offset]);
                offset += 2;
                break;

            case 0x01: // uint16 (小端序)
                if (offset + 2 > size - 2) return -5;
                received_fields[i].value.u16_val = (uint16_t)((buffer[offset+1] << 8) | buffer[offset]);
                offset += 2;
                break;

            case 0x02: // int32 (小端序)
                if (offset + 4 > size - 2) return -5;
                received_fields[i].value.i32_val = (int32_t)((buffer[offset+3] << 24) | (buffer[offset+2] << 16) | (buffer[offset+1] << 8) | buffer[offset]);
                offset += 4;
                break;

            case 0x03: // uint32 (小端序)
                if (offset + 4 > size - 2) return -5;
                received_fields[i].value.u32_val = (uint32_t)((buffer[offset+3] << 24) | (buffer[offset+2] << 16) | (buffer[offset+1] << 8) | buffer[offset]);
                offset += 4;
                break;

            case 0x04: // float (小端序)
                if (offset + 4 > size - 2) return -5;
                memcpy(&(received_fields[i].value.f_val), &buffer[offset], sizeof(float));
                offset += 4;
                break;
                
            case 0x05: // uint8
                if (offset + 1 > size - 2) return -5;
                received_fields[i].value.u8_val = buffer[offset];
                offset += 1;
                break;
                
            default:
                // 未知数据类型
                return -4; // 未知数据类型
        }
	}
	PID_P[0] = received_fields[0].value.f_val;
	PID_P[1] = received_fields[1].value.f_val;
	PID_P[2] = received_fields[2].value.f_val;
	IOUT_MAX_P = received_fields[3].value.f_val;
	OUT_MAX_P = received_fields[4].value.f_val;
	
	PID_V[0] = received_fields[5].value.f_val;
	PID_V[1] = received_fields[6].value.f_val;
	PID_V[2] = received_fields[7].value.f_val;
	IOUT_MAX_V = received_fields[8].value.f_val;
	OUT_MAX_V = received_fields[9].value.f_val;

	printf("P: %.2f %.2f %.2f %f %f\r\n", PID_P[0], PID_P[1], PID_P[2], IOUT_MAX_P, OUT_MAX_P);
	printf("V: %.2f %.2f %.2f %f %f\r\n", PID_V[0], PID_V[1], PID_V[2], IOUT_MAX_V, OUT_MAX_V);
	
	return 0;
}

void Send_code(uint8_t *buffer)
{
	buffer[0] = 0xAA;
	buffer[1] = 0x55;
	buffer[2] = 0x20;
	buffer[3] = 0x01;
	
	for(int i = 0; i < 4; i++){
		buffer[i*2 + 4] = vecdata.motor_v[i] & 0xFF;
		buffer[i*2 + 5] = (vecdata.motor_v[i] >> 8) & 0xFF;
	}
	
	buffer[12] = vecdata.vx & 0xFF;
	buffer[13] = (vecdata.vx >> 8) & 0xFF;
	buffer[14] = vecdata.vy & 0xFF;
	buffer[15] = (vecdata.vy >> 8) & 0xFF;
	buffer[16] = vecdata.vz & 0xFF;
	buffer[17] = (vecdata.vz >> 8) & 0xFF;
	
	buffer[18] = odomdata.x & 0xFF;
	buffer[19] = (odomdata.x >> 8) & 0xFF;
	buffer[20] = odomdata.y & 0xFF;
	buffer[21] = (odomdata.y >> 8) & 0xFF;
	buffer[22] = odomdata.z & 0xFF;
	buffer[23] = (odomdata.z >> 8) & 0xFF;
	
	buffer[24] = imudata.roll & 0xFF;
	buffer[25] = (imudata.roll >> 8) & 0xFF;
	buffer[26] = imudata.pitch& 0xFF;
	buffer[27] = (imudata.pitch >> 8) & 0xFF;
	buffer[28] = imudata.yaw& 0xFF;
	buffer[29] = (imudata.yaw >> 8) & 0xFF;
	
	buffer[30] = 0x00;
	for(int i = 0; i < 30 ;i++)
		buffer[30] ^= buffer[i];
	buffer[31] = 0x5D;
	
	HAL_UART_Transmit(&huart3, uart3_tx_buffer, 32, 100);
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //DMA
//{
//	if(huart == &huart3)
//	{
//		int rec = Rec_decode(uart3_rx_buffer, Size);
//		
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, sizeof(uart3_rx_buffer));
//		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
//	}
//}

void USART3_IRQHandler(void)
{
	volatile uint8_t receive;
	static uint8_t pdata[128];
	static uint8_t data_index = 0;
	//receive interrupt �����ж�
	if(huart3.Instance->SR & UART_FLAG_RXNE)
	{
			receive = huart3.Instance->DR;
			
			pdata[data_index++]=receive;
		
	}
	//idle interrupt �����ж�
	else if(huart3.Instance->SR & UART_FLAG_IDLE)
	{
			receive = huart3.Instance->DR;
			
			int rec = Rec_decode(pdata, data_index);
			for(int i=0;i<=data_index;i++)
					pdata[i]=0;
			data_index = 0;
	}
}
