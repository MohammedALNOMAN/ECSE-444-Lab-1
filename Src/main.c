/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include <fenv.h>
#include "math.h"
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))



/**
* @brief Definition for a kalman_state object, as described in the lab
* handout.
*/
struct  kalman_state {
  	  float q; // process noise covariance
  	  float r; // measurement noise covariance
  	  float x; // estimated value
  	  float p; // estimation error covariance
  	  float k; // adaptive Kalman filter gain.
    };

/**
 * @brief C signature for the Kalman filter update function. The keyword "extern"
 * is for the compiler to know that the function is defined elsewhere (kalman.s),
 * and that it is the linker's job to find the function in the global context.
 *
 * @param kstate : Pointer to a kalman_state struct
 * @param measurement : The newest measurement to update the filter coefficients
 * @param error: Pointer to an error variable that is 1 if an error is detected and 0 otherwise.
 */
extern void kalman(struct kalman_state *kstate, float measurement, int* error);

/**
 * @brief C signature for the kalmanfilter function that runs in C looping through an input array
 * and storing output values (x) in an output array
 *
 * @param InputArray : Pointer to an array of input measurements
 * @param OutputArray : Pointer to an array of output x values
 * @param kstate: pointer to the current kalman_state struct
 * @param Length: Length of the input array
 * retvalu int determining whether the program ran successfully or whether there were errors.
 */
int Kalmanfilter(float* InputArray, float* OutputArray, struct kalman_state* kstate, int Length);

/**
 * @brief C signature for the Kalman filter update function in C.
 *
 * @param kstate : Pointer to a kalman_state struct
 * @param measurement : The newest measurement to update the filter coefficients
 * @param error: Pointer to an error variable that is 1 if an error is detected and 0 otherwise.
 */
void kalmanC(struct kalman_state *kstate, float measurement, int* error);

/**
 * @brief C signature for the Kalman filter update function using CMSIS DSP Instructions.
 *
 * @param kstate : Pointer to a kalman_state struct
 * @param measurement : The newest measurement to update the filter coefficients
 * @param error: Pointer to an error variable that is 1 if an error is detected and 0 otherwise.
 */
void kalmanC(struct kalman_state *kstate, float measurement, int* error);


float average(float array[], int length);
float stddev(float *array, int length);
void correlation(float *array1, float *array2, float *array3, int length);
void convolution(float *array1, float *array2,float *array3, int length);

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  // Testing with given parameters and mock data, as described in the lab handout
  struct kalman_state filter1 = {0.1,0.1,5, -0.2, 0};
  float arr [] = {0,1,2,3,4};
  float arr2 [5];
  int error = Kalmanfilter(arr, arr2, &filter1, 5);

  while (1)
  {

  }
}

void kalmanC(struct kalman_state *kstate, float measurement, int* error){
	// p = p + q
	kstate->p += kstate ->q;

	// k = p / (p+r)
	kstate->k = (kstate ->p)/(kstate->p+kstate->r);

	// x = x + k*(measurement - x)
	kstate->x += (kstate->k)*(measurement-kstate->x);

	//p = p * (1-k)
	kstate->p *= (1-kstate->k);

	//Testing for any errors.
	if (fetestexcept(FE_DIVBYZERO || FE_INVALID || FE_OVERFLOW || FE_UNDERFLOW)){
		feclearexcept(FE_ALL_EXCEPT);
		*error = 1;
	}
	feclearexcept(FE_ALL_EXCEPT);
}

void kalmanDSP(struct kalman_state *kstate, float measurement, int* error){

	// p = p + q
	arm_add_f32(&kstate->p, &kstate->q,&kstate->p,1);

	// k = p / (p+r)
	kstate->k = (kstate ->p)/(kstate->p+kstate->r);

	// x = x + k*(measurement - x)
	arm_sub_f32(&measurement, &kstate->x,&measurement,1);
	arm_mult_f32(&measurement, &kstate->k,&measurement,1);
	arm_add_f32(&measurement, &kstate->x,&kstate->x,1);

	// p = p * (1-k)
	float i = 1;
	arm_sub_f32(&i, &kstate->k,&measurement,1);
	arm_mult_f32(&kstate->p, &measurement,&kstate->p,1);
	//Testing for any errors.
		if (fetestexcept(FE_DIVBYZERO || FE_INVALID || FE_OVERFLOW || FE_UNDERFLOW)){
			feclearexcept(FE_ALL_EXCEPT);
			*error = 1;
		}
		feclearexcept(FE_ALL_EXCEPT);
}

int Kalmanfilter(float* InputArray, float* OutputArray, struct kalman_state* kstate, int Length)
{
	// Initializing Error to 0.
	int error = 0;
	// Looping through the input array
	for(int i = 0; i < Length; i++){
		// There are three implementations available to pick from.
		kalmanC(kstate, *InputArray, &error);
		//kalman(kstate, *InputArray, &error);
		//kalmanDSP(kstate, *InputArray, &error);

		// Incrementing arrays and storing new values.
		InputArray++;
		*OutputArray = kstate -> x;
		OutputArray++;
	}
	*OutputArray = '\0';
	return error;
}

void subtract(float *original, float *kalman, float *output, int length){
	for(int i = 0; i < length; i++){
		output[i] = original[i]-kalman[i];
	}
}

float average(float array[], int length){
	float sum = 0;
	for(int i =0; i< length; i++){
		sum += array[i];
	}
	return (sum/(float)length);
}

float stddev(float *array, int length){
	float avg1 = average(array, length);
	float sum = 0;
	for (int i = 0; i<length; i++){
		float difference = array[i] - avg1;
		difference *= difference;
		sum += difference;
	}
	sum /= (length - 1);
	sum = sqrt(sum);
	return sum;
}

void correlation(float *array1, float *array2,float *array3, int length){

	convolution(array1, array2, array3, length);

int start = 0;
int end = length*2-2;
while(start<end){
	float tmp = array3[start];
	array3[start]=array3[end];
	array3[end] = tmp;
	start++;
	end--;
}


}

void convolution(float *array1, float *array2, float *array3, int length){

	for(int n=0; n<length ;n++){
		for(int m=0; m<length; m++){
			array3[m+n] += array1[n]*array2[m];
		}
	}


}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
