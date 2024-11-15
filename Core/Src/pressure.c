#include "main.h"
#include "math.h"
#include "stdio.h"
#include "config.h"
#define int_address 0x807D000
#define float_address 0x807E000
int read_flash_int,read_flash_float;
int int_distance,float_distance;
float read_flash;
int val_adc;
float distance;
uint16_t array_ADC[100];
float array_distance[100];
int filter_adc;
float kalman_adc;


extern ADC_HandleTypeDef hadc1;


// hàm chuyển đổi giá trị tuyêns tính
float map_ss(float x, float In_Max, float In_Min, float Out_Max, float Out_Min)
{
    return (((x - In_Min) * (Out_Max - Out_Min) / (In_Max - In_Min)) + Out_Min);
}

// xóa trang bộ nhớ flash
void Flash_Erase(uint32_t address) {
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef pEraseInit;
  pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  pEraseInit.PageAddress = address;
  pEraseInit.NbPages = 1;
  uint32_t PageError = 0;
  HAL_FLASHEx_Erase(&pEraseInit, &PageError);
  HAL_FLASH_Lock();
}

// viết dữ liệu vào địa chỉ flash
void Flash_write(uint32_t address,uint32_t Data) {
  HAL_FLASH_Unlock();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, Data);
  HAL_FLASH_Lock();
}

//đọc giá trị địa chỉ trong flash
uint32_t Read_Page(uint32_t address) {
  return *(uint32_t *)(address);
}

// lọc kalman
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

// xử lý và trả giá trị cảm biến
float read_ss(void)
{
	int check =0;
	int dem=0;
	while(dem<20)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		read_flash_int = Read_Page(int_address);
		read_flash_float = Read_Page(float_address);
		read_flash = read_flash_int + (read_flash_float *0.1);

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

		for(int x=0;x<=20000;x++){
			kalman_adc = kalman_filter(filter_adc);
		}
		for(int i=0;i<=100;i++){
			distance = map_ss(kalman_adc,KALMAN_ADC_MIN, KALMAN_ADC_MAX, DISTANCE_MIN, DISTANCE_MAX );//568, 3500, 0.0, 190.0;730, 3500, 0, 90
			array_distance[i]=distance;
		}
		for(int i=0;i<99;i++){
			for(int j=i+1;j<100;j++){
				if(array_distance[i]<array_distance[j])
				{
					int virtual = array_distance[i];
					array_distance[i] = array_distance[j];
					array_distance[j] = virtual;
				}
			}
		}
		distance = array_distance[100/2];
		distance =(int) ((distance - 20)*10)/10.0;
		if(distance <-20)
		{
			distance = -20;
		}
		if(distance == 0.0)
		{
			distance = 0.1;
		}
		int_distance = (distance /1);
		float_distance = (int) ((distance - int_distance)*10)%10;
		if(distance - read_flash > 0.3 || distance - read_flash < -0.3)//
		{
			dem ++;
			printf("++++++++++++++++++++ dem %d  ++++++++++++++++++\n",dem);
		}
		else
		{
//			dem ++;
			check++;
			if(check == 10)
			{
				distance = read_flash;
				printf("++++++++++++++++++ no run do while +++++++++++++++++\n");
				break;
			}
		}
		}
	if(dem >19)
	{
		Flash_Erase(int_address);
		Flash_write(int_address,int_distance);
		HAL_Delay(1000);
		Flash_Erase(float_address);
		Flash_write(float_address,float_distance);
		printf("+++++++++++++++++++++ run do while +++++++++++++++++++++\n");
	}
	HAL_ADC_Stop(&hadc1);

return distance;
}


