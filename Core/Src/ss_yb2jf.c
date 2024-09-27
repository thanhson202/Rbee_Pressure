#include "main.h"
#include "math.h"
#include "stdio.h"
int val_adc;
float distance;
float first_distance;
uint16_t array_ADC[1000];
uint16_t array_distance[100];
int filter_adc;
float kalman_adc;
extern ADC_HandleTypeDef hadc1;

// Khai báo prototype của hàm kalman_filter
float kalman_filter(unsigned long ADC_Value);

float map_ss(float x, float In_Max, float In_Min, float Out_Max, float Out_Min)
{
    return (((x - In_Min) * (Out_Max - Out_Min) / (In_Max - In_Min)) + Out_Min);
}

float read_ss(void)
{

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
//    val_adc = HAL_ADC_GetValue(&hadc1);
//    HAL_ADC_Stop(&hadc1);
	HAL_Delay(200);
	for (int i = 0; i < 1000; i++) {
		val_adc = HAL_ADC_GetValue(&hadc1);
		array_ADC[i] = val_adc;
	}

	for (int i = 0; i < 999; i++) {
		for (int j = i + 1; j < 1000; j++) {
			if (array_ADC[i] < array_ADC[j]) {
				int temp = array_ADC[i];
				array_ADC[i] = array_ADC[j];
				array_ADC[j] = temp;
			}
		}
	}

	filter_adc = array_ADC[1000 / 2];
	for(int x=0;x<=20000;x++)
	{
		kalman_adc = kalman_filter(filter_adc);
//	  HAL_Delay(5000);
	}
	int count=0;
	do
	{
		for(int i=0;i<=99;i++)
		{
			distance = map_ss(kalman_adc, 35, 3500, 0.0, 190.0);
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
		if(distance - first_distance > 2)
		{
			count++;
			continue;
		}
		else
		{
			first_distance = distance;
			break;
		}
	}
	while(count<1000);
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
