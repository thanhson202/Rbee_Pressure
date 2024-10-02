#include "main.h"
#include "math.h"
#include "stdio.h"
#define Address 0x807D000
uint32_t read_flash;
int val_adc;
int distance;
float first_distance;
uint16_t array_ADC[100];
uint16_t array_distance[100];
int filter_adc;
float kalman_adc;
extern ADC_HandleTypeDef hadc1;


// hàm chuyển đổi giá trị tuyêns tính
float map_ss(float x, float In_Max, float In_Min, float Out_Max, float Out_Min)
{
    return (((x - In_Min) * (Out_Max - Out_Min) / (In_Max - In_Min)) + Out_Min);
}

// xóa trang bộ nhớ flash
void Flash_Erase(void) {
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef pEraseInit;
  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  pEraseInit.PageAddress = Address;
  pEraseInit.NbPages = 1;
  uint32_t PageError = 0;
  HAL_FLASHEx_Erase(&pEraseInit, &PageError);
  HAL_FLASH_Lock();
}

// viết dữ liệu vào tại 1 địa chỉ flash
void Flash_write(uint32_t Data) {
  HAL_FLASH_Unlock();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data);
  HAL_FLASH_Lock();
}

//đọc giá trị tại 1 địa chỉ trong flash
uint32_t Read_Page() {
  return *(uint32_t *)(Address);
}

// xử lý và trả giá trị cảm biến
float read_ss(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	read_flash=Read_Page();
	int count=0;
	do
		{
		for (int i = 0; i < 100; i++) {
			val_adc = HAL_ADC_GetValue(&hadc1);
			array_ADC[i] = val_adc;
		}

		for (int i = 0; i < 99; i++) {
			for (int j = i + 1; j < 100; j++) {
				if (array_ADC[i] < array_ADC[j]) {
					int temp = array_ADC[i];
					array_ADC[i] = array_ADC[j];
					array_ADC[j] = temp;
				}
			}
		}

		filter_adc = array_ADC[100 / 2];

		for(int x=0;x<=20000;x++)
		{
			kalman_adc = kalman_filter(filter_adc);
		}
		for(int i=0;i<=100;i++)
		{
			distance = map_ss(kalman_adc, 568, 3500, 0.0, 190.0);
			array_distance[i]=distance;
		}
		for(int i=0;i<99;i++)
		{
			for(int j=i+1;j<100;j++)
			{
				if(array_distance[i]<array_distance[j])
				{
					int virtual = array_distance[i];
					array_distance[i] = array_distance[j];
					array_distance[j] = virtual;
				}
			}
		}
		distance = array_distance[100/2];
		if(distance - read_flash >= 2 || distance - read_flash < -1)
		{
			count ++;
			continue;

		}
		else
		{
			Flash_Erase();
			Flash_write((uint32_t) distance);
			printf("data ok/n");
			read_flash = distance;
			break;

		}
	}
	while(count<=200);
	if((count ==200 && (distance - read_flash >= 1))|| (count ==200 && (distance - read_flash <= -1)))
	{
		Flash_Erase();
		Flash_write((uint32_t) distance);
		printf("data error/n");
		read_flash = distance;
	}
	HAL_ADC_Stop(&hadc1);
return distance;
}

float kalman_filter(unsigned long ADC_Value)
{
    static float x_k1_k1, x_k_k1;
    static float P_k1_k1;
    static float kalman_adc_old = 0;

    const float Q = 0.008;	// độ nhạy của giá trị trả về
    const float R = 0.008;	// độ tin tưởng của giá trị đo được
    float Z_k = ADC_Value;
    float P_k_k1;
    float Kg;

    x_k1_k1 = kalman_adc_old;
    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1 / (P_k_k1 + R);

    kalman_adc_old = x_k_k1 + Kg * (Z_k - x_k_k1);
    P_k1_k1 = (1 - Kg) * P_k_k1;

    return kalman_adc_old;
}
