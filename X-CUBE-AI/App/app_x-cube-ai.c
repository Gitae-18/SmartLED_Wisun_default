
/**
  ******************************************************************************
  * @file    app_x-cube-ai.c
  * @author  X-CUBE-AI C code generator
  * @brief   AI program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

 /*
  * Description
  *   v1.0 - Minimum template to show how to use the Embedded Client API
  *          model. Only one input and one output is supported. All
  *          memory resources are allocated statically (AI_NETWORK_XX, defines
  *          are used).
  *          Re-target of the printf function is out-of-scope.
  *   v2.0 - add multiple IO and/or multiple heap support
  *
  *   For more information, see the embeded documentation:
  *
  *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
  *
  *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
  *   typical : C:\Users\[user_name]\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\7.1.0
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#if defined ( __ICCARM__ )
#elif defined ( __CC_ARM ) || ( __GNUC__ )
#endif

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"
#include "ai_datatypes_defines.h"
#include "network.h"
#include "network_data.h"
/* USER CODE BEGIN includes */
#include "app_x-cube-ai.h"
#include "signal/ai_config.h"
#include "signal/ai_inference.h"
_Static_assert(AE_COLS == AI_NETWORK_IN_1_SIZE &&
               AE_COLS == AI_NETWORK_OUT_1_SIZE,
               "AI feature count must match the generated model");
_Static_assert(AI_NETWORK_IN_1_FORMAT == AI_BUFFER_FORMAT_S8 &&
               AI_NETWORK_OUT_1_FORMAT == AI_BUFFER_FORMAT_S8,
               "AI preprocessing requires INT8 model input and output");
/* USER CODE END includes */

/* IO buffers ----------------------------------------------------------------*/

#if !defined(AI_NETWORK_INPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_in_1[AI_NETWORK_IN_1_SIZE_BYTES];
ai_i8* data_ins[AI_NETWORK_IN_NUM] = {
data_in_1
};
#else
ai_i8* data_ins[AI_NETWORK_IN_NUM] = {
NULL
};
#endif

#if !defined(AI_NETWORK_OUTPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_out_1[AI_NETWORK_OUT_1_SIZE_BYTES];
ai_i8* data_outs[AI_NETWORK_OUT_NUM] = {
data_out_1
};
#else
ai_i8* data_outs[AI_NETWORK_OUT_NUM] = {
NULL
};
#endif

/* Activations buffers -------------------------------------------------------*/

//ai_handle data_activations0[] = {(ai_handle) 0x20000000};

AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

static ai_handle data_activations[] = {
 activations
};

/* AI objects ----------------------------------------------------------------*/

static ai_handle network = AI_HANDLE_NULL;

static ai_buffer* ai_input;
static ai_buffer* ai_output;

static void ai_log_err(const ai_error err, const char *fct)
{
/* USER CODE BEGIN log */
if (fct)
 printf("TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
	 err.type, err.code);
else
 printf("TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type, err.code);

do {} while (1);
/* USER CODE END log */
}

static int ai_boostrap(ai_handle *act_addr)
{
ai_error err;

/* Create and initialize an instance of the model */
err = ai_network_create_and_init(&network, act_addr, NULL);
if (err.type != AI_ERROR_NONE) {
 ai_log_err(err, "ai_network_create_and_init");
 return -1;
}

ai_input = ai_network_inputs_get(network, NULL);
ai_output = ai_network_outputs_get(network, NULL);

#if defined(AI_NETWORK_INPUTS_IN_ACTIVATIONS)
/*  In the case where "--allocate-inputs" option is used, memory buffer can be
*  used from the activations buffer. This is not mandatory.
*/
for (int idx=0; idx < AI_NETWORK_IN_NUM; idx++) {
data_ins[idx] = ai_input[idx].data;
}
#else
for (int idx=0; idx < AI_NETWORK_IN_NUM; idx++) {
  ai_input[idx].data = data_ins[idx];
}
#endif

#if defined(AI_NETWORK_OUTPUTS_IN_ACTIVATIONS)
/*  In the case where "--allocate-outputs" option is used, memory buffer can be
*  used from the activations buffer. This is no mandatory.
*/
for (int idx=0; idx < AI_NETWORK_OUT_NUM; idx++) {
data_outs[idx] = ai_output[idx].data;
}
#else
for (int idx=0; idx < AI_NETWORK_OUT_NUM; idx++) {
ai_output[idx].data = data_outs[idx];
}
#endif

return 0;
}

static int ai_run(void)
{
ai_i32 batch;

batch = ai_network_run(network, ai_input, ai_output);
if (batch != 1) {
 ai_log_err(ai_network_get_error(network),
	 "ai_network_run");
 return -1;
}

return 0;
}

/* USER CODE BEGIN 2 */
int acquire_and_process_data(ai_i8* data[])
{
/* fill the inputs of the c-model
for (int idx=0; idx < AI_NETWORK_IN_NUM; idx++ )
{
   data[idx] = ....
}

*/
return 0;
}

int post_process(ai_i8* data[])
{
/* process the predictions
for (int idx=0; idx < AI_NETWORK_OUT_NUM; idx++ )
{
   data[idx] = ....
}

*/
return 0;
}


int AI_ModelRun(const int8_t input[4], int8_t output[4])
{
    if (input == NULL || output == NULL || network == AI_HANDLE_NULL ||
        ai_input == NULL || ai_output == NULL ||
        ai_input[0].data == NULL || ai_output[0].data == NULL) return -1;
    memcpy(AI_HANDLE_PTR(ai_input[0].data), input, AI_NETWORK_IN_1_SIZE_BYTES);
    if (ai_network_run(network, ai_input, ai_output) != 1) {
        ai_error error = ai_network_get_error(network);
        printf("[AI] run failed type=%u code=%u\r\n", error.type, error.code);
        return -2;
    }
    memcpy(output, AI_HANDLE_PTR(ai_output[0].data), AI_NETWORK_OUT_1_SIZE_BYTES);
    return 0;
}

/* Compatibility for the existing MSE/prediction callers. */
int AI_Run_FromRaw(const float *x_raw, uint32_t len,
                   float *out_mse, int *out_pred)
{
    if (len != AE_COLS) return -2;
    AI_Result result = AI_infer(x_raw);
    if (result.status != 0) return result.status;
    if (out_mse != NULL) *out_mse = result.mse;
    if (out_pred != NULL) *out_pred = result.is_anomaly;
    return 0;
}

int run_inference(const float *x, float *mse, int *pred)
{
return AI_Run_FromRaw(x, AE_COLS, mse, pred);
}
/* USER CODE END 2 */

/* Entry points --------------------------------------------------------------*/

//void MX_X_CUBE_AI_Init(void)
//{
//    /* USER CODE BEGIN 5 */
//	printf("ai_init\r\n");
//
//	if (ai_boostrap(data_activations0) != 0) {
//		printf("ai_bootstrap_fail\r\n");
//		return;
//	}
//
//	printf("ai_bootstrap_ok\r\n");
//    /* USER CODE END 5 */
//}

void MX_X_CUBE_AI_Init(void)
{
 printf("ai_init\r\n");

 if (ai_boostrap(data_activations) != 0) {
	 printf("ai_bootstrap_fail\r\n");
	 return;
 }

 printf("ai_bootstrap_ok\r\n");
}

void MX_X_CUBE_AI_Process(void)
{
 /* USER CODE BEGIN 6 */
printf("ai_process\r\n");

 printf("ai_run_before\r\n");

 if (ai_run() != 0) {
	 printf("ai_run_fail\r\n");
	 return;
 }

 printf("ai_run_after\r\n");
 /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
