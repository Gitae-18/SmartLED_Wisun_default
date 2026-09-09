/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-08-03T19:08:43+0900
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "network.h"
#include "network_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "0x23ebaf3afa9c92a5b2f7cb402180179b"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2026-08-03T19:08:43+0900"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  input_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 4, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_Add_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_Div_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_Mul_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_2_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_Add_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_Div_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_Mul_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_6_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#37 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#38 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#39 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#40 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#41 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#42 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#43 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#44 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#45 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_Add_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#46 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#47 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_Div_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#48 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_Mul_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#49 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_10_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#50 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 3, AI_STATIC)

/* Array#51 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#52 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#53 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#54 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#55 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#56 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#57 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#58 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#59 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#60 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#61 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#62 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Add_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#63 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#64 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Div_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#65 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Mul_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#66 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_2_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#67 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#68 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#69 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#70 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#71 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#72 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#73 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#74 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#75 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#76 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#77 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#78 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Add_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#79 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#80 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Div_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#81 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Mul_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#82 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_6_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#83 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#84 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#85 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#86 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#87 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#88 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#89 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#90 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_Pow_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#91 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#92 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#93 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#94 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_Add_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#95 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sqrt_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#96 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_Div_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#97 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_9_Mul_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#98 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_10_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#99 */
AI_ARRAY_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 4, AI_STATIC)

/* Array#100 */
AI_ARRAY_OBJ_DECLARE(
  ae_encoder_9_weight_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#101 */
AI_ARRAY_OBJ_DECLARE(
  ae_encoder_9_bias_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#102 */
AI_ARRAY_OBJ_DECLARE(
  ae_encoder_5_weight_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#103 */
AI_ARRAY_OBJ_DECLARE(
  ae_encoder_5_bias_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#104 */
AI_ARRAY_OBJ_DECLARE(
  ae_encoder_1_weight_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#105 */
AI_ARRAY_OBJ_DECLARE(
  ae_encoder_1_bias_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#106 */
AI_ARRAY_OBJ_DECLARE(
  ae_decoder_9_weight_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#107 */
AI_ARRAY_OBJ_DECLARE(
  ae_decoder_9_bias_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#108 */
AI_ARRAY_OBJ_DECLARE(
  ae_decoder_5_weight_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#109 */
AI_ARRAY_OBJ_DECLARE(
  ae_decoder_5_bias_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 128, AI_STATIC)

/* Array#110 */
AI_ARRAY_OBJ_DECLARE(
  ae_decoder_1_weight_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#111 */
AI_ARRAY_OBJ_DECLARE(
  ae_decoder_1_bias_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#112 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#113 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1, AI_STATIC)

/* Array#114 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 2048, AI_STATIC)

/* Array#115 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 512, AI_STATIC)

/* Array#116 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#117 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#118 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 131072, AI_STATIC)

/* Array#119 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 256, AI_STATIC)

/* Array#120 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#121 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 16384, AI_STATIC)

/* Array#122 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 64, AI_STATIC)

/* Array#123 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#124 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 192, AI_STATIC)

/* Array#125 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 3, AI_STATIC)

/* Array#126 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 96, AI_STATIC)

/* Array#127 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 32, AI_STATIC)

/* Array#128 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#129 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 4096, AI_STATIC)

/* Array#130 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 128, AI_STATIC)

/* Array#131 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1, AI_STATIC)

/* Array#132 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32768, AI_STATIC)

/* Array#133 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 256, AI_STATIC)

/* Array#134 */
AI_ARRAY_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 1024, AI_STATIC)

/* Array#135 */
AI_ARRAY_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 4, AI_STATIC)

/* Array#136 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 2564, AI_STATIC)

/* Array#137 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 1792, AI_STATIC)

/* Array#138 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 576, AI_STATIC)

/* Array#139 */
AI_ARRAY_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 79, AI_STATIC)

/* Array#140 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 163, AI_STATIC)

/* Array#141 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 672, AI_STATIC)

/* Array#142 */
AI_ARRAY_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 1408, AI_STATIC)

/* Array#143 */
AI_ARRAY_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 276, AI_STATIC)

/**  Array metadata declarations section  *************************************/
/* Int quant #0 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_0_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.05788961052894592f),
    AI_PACK_INTQ_ZP(-8)))

/* Int quant #1 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_0_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 32,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.004934557247906923f, 0.002352409763261676f, 0.0019977076444774866f, 0.002560019725933671f, 0.0010533301392570138f, 0.0027718667406588793f, 0.0033719211351126432f, 0.0033659052569419146f, 0.003629640443250537f, 0.004055464174598455f, 0.003782083746045828f, 0.006956018507480621f, 0.0049553425051271915f, 0.003013009438291192f, 0.005331174936145544f, 0.0018908516503870487f, 0.0042137932032346725f, 0.004066751338541508f, 0.0038196982350200415f, 0.0046506295911967754f, 0.002817275933921337f, 0.002914243843406439f, 0.007676349487155676f, 0.002983104670420289f, 0.0016761822625994682f, 0.0037713022902607918f, 0.002469424856826663f, 0.002967499429360032f, 0.003498736070469022f, 0.006730564404278994f, 0.006518031470477581f, 0.0067853135988116264f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #2 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_10_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0025580781511962414f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #3 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_Add_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.030381616204977036f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #4 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(7.874015750530816e-08f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #5 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_Div_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.02230340987443924f),
    AI_PACK_INTQ_ZP(-13)))

/* Int quant #6 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_Mul_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.03747345507144928f),
    AI_PACK_INTQ_ZP(-13)))

/* Int quant #7 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0303815770894289f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #8 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0022477328311651945f),
    AI_PACK_INTQ_ZP(69)))

/* Int quant #9 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_Sqrt_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.010915291495621204f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #10 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_1_Sub_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.05788961052894592f),
    AI_PACK_INTQ_ZP(-15)))

/* Int quant #11 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_2_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.01952461525797844f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #12 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_4_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.1173345223069191f),
    AI_PACK_INTQ_ZP(81)))

/* Int quant #13 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_4_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 128,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00506284274160862f, 0.008536944165825844f, 0.004522322677075863f, 0.004894481506198645f, 0.015420188196003437f, 0.009844101034104824f, 0.00972455833107233f, 0.005149972625076771f, 0.00524670397862792f, 0.009685556404292583f, 0.003869631327688694f, 0.009633020497858524f, 0.0059097157791256905f, 0.006589442957192659f, 0.00790149625390768f, 0.014482607133686543f, 0.010495423339307308f, 0.00940100010484457f, 0.006763384211808443f, 0.004110860172659159f, 0.006022683810442686f, 0.009967461228370667f, 0.013986606150865555f, 0.005156903062015772f, 0.006513104308396578f, 0.00597758824005723f, 0.0181264765560627f, 0.003528862027451396f, 0.0049352762289345264f, 0.0037133307196199894f, 0.008997074328362942f, 0.009394709020853043f, 0.005146375857293606f, 0.017166053876280785f, 0.006885299924761057f, 0.004144249018281698f, 0.011353961192071438f, 0.005466071888804436f, 0.014482839033007622f, 0.005573552567511797f, 0.00462390948086977f, 0.008746817708015442f, 0.007690146565437317f, 0.011272047646343708f, 0.004730174317955971f, 0.014066239818930626f, 0.00921375211328268f, 0.006287255324423313f, 0.01540299691259861f, 0.004108554217964411f, 0.025011548772454262f, 0.004162702709436417f, 0.005259352270513773f, 0.012087236158549786f, 0.004048533271998167f, 0.00603195047006011f, 0.00521472143009305f, 0.03838945925235748f, 0.003939506597816944f, 0.008887593634426594f, 0.008141700178384781f, 0.004852848127484322f, 0.004080135375261307f, 0.006382701452821493f, 0.006985739804804325f, 0.005061187781393528f, 0.0044686319306492805f, 0.01671142689883709f, 0.0057899528183043f, 0.01216916274279356f, 0.004008614458143711f, 0.005297766532748938f, 0.007518155034631491f, 0.005050919950008392f, 0.007603139616549015f, 0.007662123069167137f, 0.00513459974899888f, 0.0050586434081196785f, 0.015724368393421173f, 0.006316231098026037f, 0.007434004917740822f, 0.004030720330774784f, 0.007727475371211767f, 0.00764599721878767f, 0.009315901435911655f, 0.006279208697378635f, 0.013007509522140026f, 0.013714022003114223f, 0.01600642129778862f, 0.003736280370503664f, 0.0072677829302847385f, 0.015007510781288147f, 0.010045025497674942f, 0.015275167301297188f, 0.007956269197165966f, 0.007248897571116686f, 0.00830161664634943f, 0.007209590170532465f, 0.01778581738471985f, 0.006272259168326855f, 0.012799404561519623f, 0.008066896349191666f, 0.007279615383595228f, 0.00754697248339653f, 0.013097809627652168f, 0.00598150584846735f, 0.007533987518399954f, 0.005974362138658762f, 0.016364097595214844f, 0.005966579541563988f, 0.0050531793385744095f, 0.0074059488251805305f, 0.006435353308916092f, 0.009952486492693424f, 0.0047337268479168415f, 0.021087199449539185f, 0.004758641589432955f, 0.010150711983442307f, 0.007189066149294376f, 0.0033361418172717094f, 0.004490317311137915f, 0.005145626608282328f, 0.003997367341071367f, 0.00576765788719058f, 0.005031829234212637f, 0.004091345705091953f, 0.004413438029587269f, 0.014796738512814045f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #14 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_Add_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.030941901728510857f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #15 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.015748031437397003f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #16 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_Div_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.04534182697534561f),
    AI_PACK_INTQ_ZP(65)))

/* Int quant #17 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_Mul_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.07541098445653915f),
    AI_PACK_INTQ_ZP(70)))

/* Int quant #18 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.030941862612962723f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #19 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.002608970971778035f),
    AI_PACK_INTQ_ZP(55)))

/* Int quant #20 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_Sqrt_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.011015479452908039f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #21 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_5_Sub_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.11699767410755157f),
    AI_PACK_INTQ_ZP(83)))

/* Int quant #22 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_6_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.01645105704665184f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #23 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_8_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.2525303065776825f),
    AI_PACK_INTQ_ZP(78)))

/* Int quant #24 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_8_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 256,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.006368861999362707f, 0.0037941483315080404f, 0.00464263278990984f, 0.02490343526005745f, 0.014076752588152885f, 0.003922120667994022f, 0.006497184745967388f, 0.008446039631962776f, 0.004713377915322781f, 0.008782739751040936f, 0.008978627622127533f, 0.004445252940058708f, 0.004195427522063255f, 0.015223279595375061f, 0.03566555306315422f, 0.008596203289926052f, 0.00543889869004488f, 0.003994530066847801f, 0.0053412094712257385f, 0.005364041309803724f, 0.007123009767383337f, 0.005443748086690903f, 0.016202345490455627f, 0.003759781364351511f, 0.020457599312067032f, 0.004869800992310047f, 0.005461261607706547f, 0.0046717459335923195f, 0.0078340545296669f, 0.007641585543751717f, 0.0044884891249239445f, 0.014509232714772224f, 0.005673510953783989f, 0.004275296814739704f, 0.009214339777827263f, 0.009771429002285004f, 0.008191974833607674f, 0.004497390240430832f, 0.0031350275967270136f, 0.009259414859116077f, 0.004430105909705162f, 0.011666528880596161f, 0.011948895640671253f, 0.005354803055524826f, 0.006877636071294546f, 0.007376484107226133f, 0.004541286267340183f, 0.00843253917992115f, 0.004559897352010012f, 0.004321729298681021f, 0.006281149573624134f, 0.01134529896080494f, 0.004868297837674618f, 0.00865752249956131f, 0.013651316985487938f, 0.007172435987740755f, 0.006787965074181557f, 0.0037992268335074186f, 0.006372086238116026f, 0.0047322348691523075f, 0.0036193947307765484f, 0.004425952211022377f, 0.014384645968675613f, 0.0033434443175792694f, 0.005640482064336538f, 0.02038615196943283f, 0.013685113750398159f, 0.0072071063332259655f, 0.006186086684465408f, 0.0053262473084032536f, 0.006874813232570887f, 0.005622980650514364f, 0.004789619240909815f, 0.004733497742563486f, 0.004349187482148409f, 0.0044752745889127254f, 0.004950202535837889f, 0.006208701990544796f, 0.0059005445800721645f, 0.011963951401412487f, 0.020559383556246758f, 0.02521459199488163f, 0.019679661840200424f, 0.0037198797799646854f, 0.004083157982677221f, 0.004747191909700632f, 0.007547750603407621f, 0.007789481896907091f, 0.005547307897359133f, 0.007429826073348522f, 0.004463308025151491f, 0.005388808436691761f, 0.0049140676856040955f, 0.005660907365381718f, 0.008988454937934875f, 0.010924979113042355f, 0.004744028206914663f, 0.0038649688940495253f, 0.010584070347249508f, 0.0030096217524260283f, 0.005071952007710934f, 0.005191846750676632f, 0.008542933501303196f, 0.008745011873543262f, 0.0029918826185166836f, 0.00424691429361701f, 0.004589437507092953f, 0.003885868936777115f, 0.006611613091081381f, 0.0036429460160434246f, 0.008055130951106548f, 0.004518334288150072f, 0.004225595388561487f, 0.028346961364150047f, 0.003773726522922516f, 0.0061104376800358295f, 0.007223923224955797f, 0.0045327008701860905f, 0.004428432788699865f, 0.02184124104678631f, 0.008940319530665874f, 0.00484883040189743f, 0.004327314905822277f, 0.007607143372297287f, 0.005085497163236141f, 0.016684256494045258f, 0.009000708349049091f, 0.003763927612453699f, 0.006092310883104801f, 0.01236877590417862f, 0.005798527505248785f, 0.013385524041950703f, 0.003885367652401328f, 0.009688157588243484f, 0.004243165720254183f, 0.004852789919823408f, 0.02619239315390587f, 0.0052819582633674145f, 0.004636875819414854f, 0.00443338742479682f, 0.004799754824489355f, 0.007023238576948643f, 0.005039331968873739f, 0.003783147083595395f, 0.01736212894320488f, 0.008630212396383286f, 0.008885194547474384f, 0.006989923771470785f, 0.004410762805491686f, 0.025048809126019478f, 0.004507957957684994f, 0.0045058997347950935f, 0.0046667857095599174f, 0.0059995935298502445f, 0.006187896244227886f, 0.009117038920521736f, 0.007697257678955793f, 0.010406429879367352f, 0.004831343423575163f, 0.0052277990616858006f, 0.01584203913807869f, 0.004999054130166769f, 0.004428757820278406f, 0.004500525537878275f, 0.007536228280514479f, 0.004922209773212671f, 0.012253322638571262f, 0.005666157230734825f, 0.004880688153207302f, 0.02574242278933525f, 0.0072523923590779305f, 0.010681197047233582f, 0.008348739705979824f, 0.004469280131161213f, 0.0031830461230129004f, 0.012867387384176254f, 0.005153392441570759f, 0.008115655742585659f, 0.02376866154372692f, 0.004797421861439943f, 0.0062291850335896015f, 0.006521513685584068f, 0.004672238603234291f, 0.0048235817812383175f, 0.005539708770811558f, 0.0091007174924016f, 0.017400890588760376f, 0.00941250566393137f, 0.008159136399626732f, 0.0041852532885968685f, 0.0048606861382722855f, 0.010080412961542606f, 0.0062834699638187885f, 0.0029358863830566406f, 0.005989891476929188f, 0.005229821428656578f, 0.004438038915395737f, 0.0039058595430105925f, 0.005321966949850321f, 0.00926879420876503f, 0.007195169571787119f, 0.00758669339120388f, 0.008100121282041073f, 0.013054879382252693f, 0.004291524179279804f, 0.02577139437198639f, 0.006726877298206091f, 0.0047213309444487095f, 0.004981653299182653f, 0.0067320941016077995f, 0.005979334469884634f, 0.02900843322277069f, 0.0043241167441010475f, 0.00915828999131918f, 0.01239799614995718f, 0.01180977188050747f, 0.012616835534572601f, 0.00924434419721365f, 0.007005944848060608f, 0.003735903650522232f, 0.026033762842416763f, 0.010314910672605038f, 0.005424014292657375f, 0.006620864849537611f, 0.004335211589932442f, 0.005359644535928965f, 0.024263309314846992f, 0.013771644793450832f, 0.006225298624485731f, 0.007289746310561895f, 0.016791366040706635f, 0.003819888224825263f, 0.020159965381026268f, 0.006506821606308222f, 0.023993156850337982f, 0.004611288662999868f, 0.0038486344274133444f, 0.0068960669450461864f, 0.0030854344367980957f, 0.004784877877682447f, 0.005772423464804888f, 0.008546778932213783f, 0.025676041841506958f, 0.010454787872731686f, 0.025858262553811073f, 0.006768138147890568f, 0.007945967838168144f, 0.004939627833664417f, 0.005110583733767271f, 0.021484240889549255f, 0.00435147900134325f, 0.003995641600340605f, 0.010202097706496716f, 0.009933368302881718f, 0.025349557399749756f, 0.02090274915099144f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #25 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_9_Add_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.8336520791053772f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #26 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_9_Div_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.020404204726219177f),
    AI_PACK_INTQ_ZP(70)))

/* Int quant #27 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_9_Mul_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.01875816099345684f),
    AI_PACK_INTQ_ZP(87)))

/* Int quant #28 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.8336520195007324f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #29 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.011196249164640903f),
    AI_PACK_INTQ_ZP(127)))

/* Int quant #30 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_9_Sqrt_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.05717713013291359f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #31 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_decoder_decoder_9_Sub_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.25375163555145264f),
    AI_PACK_INTQ_ZP(69)))

/* Int quant #32 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_0_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.012404818087816238f),
    AI_PACK_INTQ_ZP(10)))

/* Int quant #33 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_0_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 512,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0029190632048994303f, 0.006144682876765728f, 0.007149095647037029f, 0.002701401710510254f, 0.007181146182119846f, 0.005617291666567326f, 0.0014231678796932101f, 0.009465115144848824f, 0.0022533293813467026f, 0.00384519319050014f, 0.004811038728803396f, 0.0043052081018686295f, 0.006393799092620611f, 0.004351460840553045f, 0.004980434663593769f, 0.0034769822377711535f, 0.0045310440473258495f, 0.01200930867344141f, 0.004830691963434219f, 0.005141512956470251f, 0.008856650441884995f, 0.0039713745936751366f, 0.003646064782515168f, 0.007988931611180305f, 0.005444255191832781f, 0.0036775153130292892f, 0.007243590895086527f, 0.009809503331780434f, 0.009219100698828697f, 0.005356383975595236f, 0.003768050344660878f, 0.0062112570740282536f, 0.00423679081723094f, 0.005252011585980654f, 0.005640688352286816f, 0.004792644642293453f, 0.003895434783771634f, 0.004282729234546423f, 0.004880958702415228f, 0.0032111727632582188f, 0.004871738608926535f, 0.0049804942682385445f, 0.0025917161256074905f, 0.0018091712845489383f, 0.005866576451808214f, 0.0030594845302402973f, 0.003798343474045396f, 0.0016309339553117752f, 0.002699676202610135f, 0.002070792717859149f, 0.010567348450422287f, 0.0007500472129322588f, 0.006431492045521736f, 0.00633723521605134f, 0.003395281033590436f, 0.005792564712464809f, 0.009089251048862934f, 0.003990120254456997f, 0.004072124604135752f, 0.005022409372031689f, 0.004491845145821571f, 0.01214621402323246f, 0.003503110958263278f, 0.0061953370459377766f, 0.003989385906606913f, 0.004491221625357866f, 0.0037899252492934465f, 0.00635689590126276f, 0.004417107906192541f, 0.00613476661965251f, 0.014497959055006504f, 0.007842456921935081f, 0.0024786004796624184f, 0.008855520747601986f, 0.003000320866703987f, 0.003214226569980383f, 0.006492370739579201f, 0.009675469249486923f, 0.007647203281521797f, 0.005382882431149483f, 0.004387169610708952f, 0.006446005776524544f, 0.0056962608359754086f, 0.003060875926166773f, 0.004237037617713213f, 0.0038268801290541887f, 0.0064010778442025185f, 0.002553492086008191f, 0.006123639177531004f, 0.004280004650354385f, 0.01244245283305645f, 0.003899010131135583f, 0.010485189035534859f, 0.0014671625103801489f, 0.004886319395154715f, 0.003342072945088148f, 0.00898168608546257f, 0.0029509104788303375f, 0.0032951452303677797f, 0.0032096717040985823f, 0.0024926303885877132f, 0.0135947335511446f, 0.01080053485929966f, 0.004966183099895716f, 0.006606333889067173f, 0.00997735932469368f, 0.005332880653440952f, 0.005395814310759306f, 0.007344442419707775f, 0.0026285352651029825f, 0.004220347385853529f, 0.004747368860989809f, 0.004478587303310633f, 0.0026323588099330664f, 0.004033331759274006f, 0.004236956592649221f, 0.005111261736601591f, 0.00531578715890646f, 0.003999228589236736f, 0.002075423952192068f, 0.003991499077528715f, 0.005542528349906206f, 0.003241508500650525f, 0.0009838227415457368f, 0.004937580320984125f, 0.005204028449952602f, 0.005995307117700577f, 0.005947811994701624f, 0.0036455083172768354f, 0.005562171805649996f, 0.0036827498115599155f, 0.004030341282486916f, 0.003924669697880745f, 0.00683780200779438f, 0.012297356501221657f, 0.0030890065245330334f, 0.0031610825099051f, 0.002938777906820178f, 0.004195373505353928f, 0.00352780451066792f, 0.00452251685783267f, 0.0014841781230643392f, 0.001816100673750043f, 0.006222962401807308f, 0.0096348961815238f, 0.004802083596587181f, 0.0066699315793812275f, 0.004857789725065231f, 0.01138540543615818f, 0.007323576137423515f, 0.008063098415732384f, 0.003234765026718378f, 0.0034770669881254435f, 0.006341820582747459f, 0.00573362410068512f, 0.007503874134272337f, 0.00384832127019763f, 0.00590908108279109f, 0.004315131343901157f, 0.010210460051894188f, 0.005096103996038437f, 0.008818591013550758f, 0.011274258606135845f, 0.003417085623368621f, 0.004681842867285013f, 0.008219163864850998f, 0.0027368362061679363f, 0.004371684044599533f, 0.00396695826202631f, 0.003043161705136299f, 0.0034953083377331495f, 0.007209328003227711f, 0.004110323265194893f, 0.00416664220392704f, 0.0036119797732681036f, 0.005438751541078091f, 0.005205783061683178f, 0.004677869379520416f, 0.00542182894423604f, 0.0038906834088265896f, 0.002762594958767295f, 0.003970840014517307f, 0.006796441972255707f, 0.0054483432322740555f, 0.005304550286382437f, 0.00404618214815855f, 0.005793956108391285f, 0.005094627384096384f, 0.006145154125988483f, 0.004980188328772783f, 0.0015654682647436857f, 0.0054766349494457245f, 0.005378397181630135f, 0.0013012817362323403f, 0.002565347822383046f, 0.0035557474475353956f, 0.006797842215746641f, 0.005618828348815441f, 0.004751361906528473f, 0.014475326985120773f, 0.008079632185399532f, 0.010888619348406792f, 0.00476940767839551f, 0.005365668796002865f, 0.003405139548704028f, 0.004709277302026749f, 0.005512116476893425f, 0.005752860568463802f, 0.005770767107605934f, 0.0014335348969325423f, 0.005197141785174608f, 0.0017817101906985044f, 0.004859537351876497f, 0.011182419024407864f, 0.0027177685406059027f, 0.007026733364909887f, 0.005187483504414558f, 0.0052183629013597965f, 0.0021496103145182133f, 0.004715268034487963f, 0.0016503355000168085f, 0.0030003886204212904f, 0.004391804337501526f, 0.0018828135216608644f, 0.0019298524130135775f, 0.0038238056004047394f, 0.008233620785176754f, 0.006651034113019705f, 0.004385326057672501f, 0.0033522420562803745f, 0.013441902585327625f, 0.004653433803468943f, 0.005954088643193245f, 0.0014907093718647957f, 0.0036909920163452625f, 0.008637109771370888f, 0.01521062571555376f, 0.0035275956615805626f, 0.005707457195967436f, 0.004305224400013685f, 0.0044164108112454414f, 0.003548296634107828f, 0.001586354337632656f, 0.01059221476316452f, 0.005206473637372255f, 0.004785499535501003f, 0.006399852689355612f, 0.0013582870597019792f, 0.008387474343180656f, 0.005896310321986675f, 0.003658079542219639f, 0.001288986299186945f, 0.002797601744532585f, 0.005617169197648764f, 0.005894646514207125f, 0.0017053562914952636f, 0.003647085279226303f, 0.003267718246206641f, 0.006387760862708092f, 0.003668540623039007f, 0.006438989657908678f, 0.001691272365860641f, 0.0080632995814085f, 0.010514216497540474f, 0.00522616459056735f, 0.0037915518041700125f, 0.010832928121089935f, 0.0033925173338502645f, 0.003112208563834429f, 0.002587419468909502f, 0.0020838899072259665f, 0.004520973190665245f, 0.016144048422574997f, 0.00591711001470685f, 0.004059319384396076f, 0.005268285516649485f, 0.0023152560461312532f, 0.004386378452181816f, 0.004966192413121462f, 0.0033868206664919853f, 0.001912452862598002f, 0.005270509514957666f, 0.01198931597173214f, 0.0061279707588255405f, 0.006068033166229725f, 0.003986791707575321f, 0.004165302496403456f, 0.003963683266192675f, 0.008399371057748795f, 0.007853140123188496f, 0.0040584453381598f, 0.010688983835279942f, 0.0021334735210984945f, 0.0070585994981229305f, 0.005861440673470497f, 0.0069821556098759174f, 0.00241955928504467f, 0.004541658330708742f, 0.004601710010319948f, 0.0015823451103642583f, 0.010550804436206818f, 0.0015594696160405874f, 0.005811046343296766f, 0.0030520777218043804f, 0.0037157651968300343f, 0.005802072584629059f, 0.004642092622816563f, 0.006419314071536064f, 0.0033629091922193766f, 0.003468015231192112f, 0.006453728303313255f, 0.011528590694069862f, 0.006899814121425152f, 0.006547199096530676f, 0.004586697090417147f, 0.004915810190141201f, 0.0022087725810706615f, 0.003556969575583935f, 0.010575849562883377f, 0.001356051885522902f, 0.001952235121279955f, 0.004797298461198807f, 0.0045876228250563145f, 0.0037593976594507694f, 0.004451476037502289f, 0.001724475878290832f, 0.005984932649880648f, 0.004733064211905003f, 0.0059564621187746525f, 0.011264865286648273f, 0.0025411017704755068f, 0.005956812296062708f, 0.0027289625722914934f, 0.006833293009549379f, 0.006569650489836931f, 0.0037210111040621996f, 0.0067876786924898624f, 0.0065424456261098385f, 0.0030195522122085094f, 0.006309550255537033f, 0.009442431852221489f, 0.005971130449324846f, 0.003933022730052471f, 0.017889492213726044f, 0.005247350782155991f, 0.004405331332236528f, 0.005096199456602335f, 0.004192711319774389f, 0.0024560897145420313f, 0.008343741297721863f, 0.0031614303588867188f, 0.0016190226888284087f, 0.0033572495449334383f, 0.00240364414639771f, 0.002244139090180397f, 0.0042452565394341946f, 0.004328533075749874f, 0.00853494182229042f, 0.006084763910621405f, 0.003794351825490594f, 0.002184427808970213f, 0.006368352100253105f, 0.0017368949484080076f, 0.004354151897132397f, 0.0033787644933909178f, 0.0051128757186234f, 0.005740311928093433f, 0.0035496968775987625f, 0.006684018764644861f, 0.002938050776720047f, 0.004511211067438126f, 0.0029110568575561047f, 0.006039034109562635f, 0.0007025639060884714f, 0.005223583430051804f, 0.003363098483532667f, 0.003926380071789026f, 0.006495453417301178f, 0.0034938324242830276f, 0.0022436075378209352f, 0.0043882871977984905f, 0.005248125176876783f, 0.008254386484622955f, 0.0022538441698998213f, 0.0015345333376899362f, 0.0039927056059241295f, 0.0018034868407994509f, 0.0021257400512695312f, 0.0038310992531478405f, 0.007632309105247259f, 0.003194327699020505f, 0.004161225166171789f, 0.003323861863464117f, 0.008368008770048618f, 0.002700796350836754f, 0.0033810664899647236f, 0.001755113829858601f, 0.004217015579342842f, 0.003555926261469722f, 0.003591047367081046f, 0.015225212089717388f, 0.009241711348295212f, 0.003134665545076132f, 0.005995603743940592f, 0.011331863701343536f, 0.00897405669093132f, 0.004478162620216608f, 0.003793281503021717f, 0.00750006316229701f, 0.005566323176026344f, 0.005240308120846748f, 0.0056983912363648415f, 0.0030816663056612015f, 0.006128589157015085f, 0.0034752916544675827f, 0.0016511785797774792f, 0.009842626750469208f, 0.007911902852356434f, 0.0023626002948731184f, 0.005853683687746525f, 0.002293137600645423f, 0.0039856527000665665f, 0.0027953065000474453f, 0.004863095004111528f, 0.003187748836353421f, 0.009876860305666924f, 0.005251059774309397f, 0.004472585394978523f, 0.004608081188052893f, 0.005450434982776642f, 0.00985121913254261f, 0.004839567467570305f, 0.004513012245297432f, 0.0022454471327364445f, 0.003076805965974927f, 0.0025861177127808332f, 0.005101095885038376f, 0.005339178256690502f, 0.005608549807220697f, 0.003564025741070509f, 0.01294076256453991f, 0.0038822218775749207f, 0.003678759792819619f, 0.008237862028181553f, 0.004513267893344164f, 0.0034954845905303955f, 0.004790255334228277f, 0.0033620602916926146f, 0.0046370928175747395f, 0.009780566208064556f, 0.004243093077093363f, 0.011658689007163048f, 0.003701034700497985f, 0.004717853851616383f, 0.005317359697073698f, 0.0038120835088193417f, 0.003762702224776149f, 0.005355098284780979f, 0.005073169246315956f, 0.004892095923423767f, 0.004185409750789404f, 0.008448164910078049f, 0.005176612176001072f, 0.001987294526770711f, 0.01203768327832222f, 0.0034266961738467216f, 0.005117408465594053f, 0.003997888881713152f, 0.012184038758277893f, 0.00246736709959805f, 0.002415385330095887f, 0.001988198608160019f, 0.0026000982616096735f, 0.0029319648165255785f, 0.006795487366616726f, 0.004037942737340927f, 0.0109602315351367f, 0.007134146057069302f, 0.010537959635257721f, 0.005146445706486702f, 0.0023497948423027992f, 0.002929284470155835f, 0.005375539418309927f, 0.003542035585269332f, 0.005191631615161896f, 0.00634860759600997f, 0.010081383399665356f, 0.004171488806605339f, 0.007097600027918816f, 0.005381707567721605f, 0.010152243077754974f, 0.011444801464676857f, 0.002237560460343957f, 0.0027488626074045897f, 0.00979805551469326f, 0.002448704093694687f, 0.009832251816987991f, 0.0023985966108739376f, 0.004008926916867495f, 0.012443571351468563f, 0.004193561617285013f, 0.0025458114687353373f, 0.0022055115550756454f, 0.002747299848124385f, 0.0033431225456297398f, 0.006752259563654661f, 0.01045779325067997f, 0.004381985403597355f, 0.002217561937868595f, 0.003371052211150527f, 0.012664343230426311f, 0.004756120964884758f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #34 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_10_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.011729802004992962f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #35 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_12_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.03690348193049431f),
    AI_PACK_INTQ_ZP(-7)))

/* Int quant #36 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_12_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 3,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.004362110979855061f, 0.007824146188795567f, 0.00493286969140172f),
    AI_PACK_INTQ_ZP(0, 0, 0)))

/* Int quant #37 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_1_Add_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0013148986035957932f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #38 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_1_Div_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.03085358627140522f),
    AI_PACK_INTQ_ZP(45)))

/* Int quant #39 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_1_Mul_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.06353417038917542f),
    AI_PACK_INTQ_ZP(75)))

/* Int quant #40 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0013148592552170157f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #41 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0003497962898109108f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #42 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_1_Sqrt_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.002270784927532077f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #43 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_1_Sub_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.012335856445133686f),
    AI_PACK_INTQ_ZP(16)))

/* Int quant #44 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_2_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.013422462157905102f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #45 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_4_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.35115495324134827f),
    AI_PACK_INTQ_ZP(25)))

/* Int quant #46 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_4_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 256,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0032591214403510094f, 0.007605807390064001f, 0.006340926978737116f, 0.007900583557784557f, 0.004408572800457478f, 0.002889369847252965f, 0.0062042949721217155f, 0.006575942970812321f, 0.0028520964551717043f, 0.003974206279963255f, 0.005366823170334101f, 0.0032788850367069244f, 0.006149075925350189f, 0.0041898577474057674f, 0.007877218537032604f, 0.030116714537143707f, 0.0072800046764314175f, 0.0042951833456754684f, 0.007923129014670849f, 0.0031848556827753782f, 0.00853086169809103f, 0.006373079027980566f, 0.00739457318559289f, 0.01040186919271946f, 0.0028883228078484535f, 0.007097635883837938f, 0.0030157873407006264f, 0.002635327400639653f, 0.006121617741882801f, 0.00836732517927885f, 0.003281674114987254f, 0.0032536524813622236f, 0.01669129729270935f, 0.00659684743732214f, 0.009646249003708363f, 0.003614639164879918f, 0.003489274997264147f, 0.008481920696794987f, 0.006768389139324427f, 0.005810278467833996f, 0.010601839981973171f, 0.005918154027312994f, 0.011138086207211018f, 0.006276719272136688f, 0.006250877398997545f, 0.0030306074768304825f, 0.003824706422165036f, 0.008675448596477509f, 0.0024859472177922726f, 0.006382358726114035f, 0.007378759793937206f, 0.006643729284405708f, 0.0059880889020860195f, 0.006831637583673f, 0.004776772577315569f, 0.004001605324447155f, 0.0029333464335650206f, 0.003935715649276972f, 0.006625581532716751f, 0.006867679301649332f, 0.00379014341160655f, 0.011868605390191078f, 0.003398246131837368f, 0.0036543270107358694f, 0.007770739030092955f, 0.00466089928522706f, 0.006680873688310385f, 0.0034962184727191925f, 0.0029484087135642767f, 0.01721927523612976f, 0.006925093941390514f, 0.0030371341854333878f, 0.012273363769054413f, 0.005762993823736906f, 0.004211251623928547f, 0.00337786297313869f, 0.006148279644548893f, 0.006743260659277439f, 0.002898199250921607f, 0.0021567244548350573f, 0.0030783372931182384f, 0.0053920443169772625f, 0.003336142050102353f, 0.0030663502402603626f, 0.009691089391708374f, 0.011191119439899921f, 0.007274821400642395f, 0.00556961027905345f, 0.007509004790335894f, 0.0030019059777259827f, 0.003648559097200632f, 0.003318856004625559f, 0.00952052604407072f, 0.009537442587316036f, 0.006677794270217419f, 0.008640805259346962f, 0.00838582031428814f, 0.0059854211285710335f, 0.004492127802222967f, 0.0030984925106167793f, 0.03766007348895073f, 0.0059459577314555645f, 0.0028163061942905188f, 0.003652620827779174f, 0.003072831081226468f, 0.002930167131125927f, 0.0035848929546773434f, 0.008573699742555618f, 0.003344797296449542f, 0.008155837655067444f, 0.004047283437103033f, 0.003075860207900405f, 0.003089664038270712f, 0.007192704826593399f, 0.0037822534795850515f, 0.006666103377938271f, 0.002878869418054819f, 0.011149859055876732f, 0.003424542723223567f, 0.007882385514676571f, 0.00803739856928587f, 0.006055193953216076f, 0.0028908876702189445f, 0.007343132980167866f, 0.007191295735538006f, 0.004118664190173149f, 0.0033155546989291906f, 0.003191133728250861f, 0.003468002425506711f, 0.00998560432344675f, 0.0035150647163391113f, 0.0030721903312951326f, 0.004896409343928099f, 0.0031486183870583773f, 0.008380000479519367f, 0.0028177937492728233f, 0.006726571824401617f, 0.004028224386274815f, 0.012617047876119614f, 0.003641604445874691f, 0.009176431223750114f, 0.0032524268608540297f, 0.00756689952686429f, 0.004046037793159485f, 0.009914393536746502f, 0.006479884497821331f, 0.0036707890685647726f, 0.008666254580020905f, 0.02654111012816429f, 0.0032818419858813286f, 0.003429712262004614f, 0.0027652746066451073f, 0.0032741266768425703f, 0.006264122202992439f, 0.017241070047020912f, 0.002364401938393712f, 0.0027656524907797575f, 0.008355909958481789f, 0.02153557538986206f, 0.006028003059327602f, 0.007507928181439638f, 0.0053835101425647736f, 0.006659611593931913f, 0.0027390336617827415f, 0.00575987109914422f, 0.005714599974453449f, 0.007266449276357889f, 0.003515910590067506f, 0.0062841204926371574f, 0.008142918348312378f, 0.0030799496453255415f, 0.0034213175531476736f, 0.004855663049966097f, 0.010598313994705677f, 0.009874366223812103f, 0.006260034162551165f, 0.00545603409409523f, 0.006119975820183754f, 0.011561502702534199f, 0.015162669122219086f, 0.003053687047213316f, 0.0031407377682626247f, 0.003237496828660369f, 0.008778320625424385f, 0.00432969257235527f, 0.005778559483587742f, 0.010536414571106434f, 0.01488340925425291f, 0.009808816947042942f, 0.00303958379663527f, 0.006048123352229595f, 0.007746935822069645f, 0.014038522727787495f, 0.003073576372116804f, 0.0031668690498918295f, 0.0052880216389894485f, 0.005117794033139944f, 0.007160849403589964f, 0.01127872709184885f, 0.006222540512681007f, 0.003299756906926632f, 0.003008789848536253f, 0.009600936435163021f, 0.0030275736935436726f, 0.0028458740562200546f, 0.016338204964995384f, 0.013765780255198479f, 0.003077459055930376f, 0.00340437819249928f, 0.006346790585666895f, 0.006816252134740353f, 0.029751239344477654f, 0.00291250622831285f, 0.0031941342167556286f, 0.007027660496532917f, 0.0030801414977759123f, 0.0035671705845743418f, 0.011647152714431286f, 0.006067628040909767f, 0.007259144447743893f, 0.0120191415771842f, 0.009412997402250767f, 0.08087609708309174f, 0.006357062142342329f, 0.010240708477795124f, 0.01314464770257473f, 0.007548877503722906f, 0.00794221181422472f, 0.0033914223313331604f, 0.0031988194677978754f, 0.008486859500408173f, 0.003792384872213006f, 0.008358747698366642f, 0.009568668901920319f, 0.034108784049749374f, 0.007857881486415863f, 0.011164789088070393f, 0.006648811046034098f, 0.0035164030268788338f, 0.005411413963884115f, 0.011993947438895702f, 0.00312221166677773f, 0.010302665643393993f, 0.024736803025007248f, 0.008565649390220642f, 0.003673318773508072f, 0.003406231990084052f, 0.003446523565798998f, 0.007277457509189844f, 0.007072336971759796f, 0.00688003096729517f, 0.005756285507231951f, 0.004286125767976046f, 0.0032976665534079075f, 0.0029487237334251404f, 0.005069079343229532f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #47 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_5_Add_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.8655349016189575f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #48 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_5_Div_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.02996332198381424f),
    AI_PACK_INTQ_ZP(25)))

/* Int quant #49 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_5_Mul_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.05708756670355797f),
    AI_PACK_INTQ_ZP(43)))

/* Int quant #50 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.8655348420143127f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #51 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.02218455821275711f),
    AI_PACK_INTQ_ZP(127)))

/* Int quant #52 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_5_Sqrt_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.058260228484869f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #53 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_5_Sub_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.3510391116142273f),
    AI_PACK_INTQ_ZP(13)))

/* Int quant #54 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_6_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.019470559433102608f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #55 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_8_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.5360717177391052f),
    AI_PACK_INTQ_ZP(54)))

/* Int quant #56 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_8_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 64,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.005832654889672995f, 0.005467894487082958f, 0.007599998265504837f, 0.0034262912813574076f, 0.011946775950491428f, 0.014928445219993591f, 0.006582413334399462f, 0.004133881535381079f, 0.015543518587946892f, 0.007099017035216093f, 0.00598886189982295f, 0.004838487599045038f, 0.0068156057968735695f, 0.004719462711364031f, 0.020840108394622803f, 0.021059000864624977f, 0.00848323293030262f, 0.007324805483222008f, 0.011294877156615257f, 0.004394868854433298f, 0.008328196592628956f, 0.0051853712648153305f, 0.009179259650409222f, 0.007210056763142347f, 0.0083164619281888f, 0.0071530151180922985f, 0.009077982977032661f, 0.010538939386606216f, 0.005674343556165695f, 0.007235182449221611f, 0.030361227691173553f, 0.020512009039521217f, 0.008739703334867954f, 0.00648889597505331f, 0.006540199276059866f, 0.0064720953814685345f, 0.007820452563464642f, 0.020559808239340782f, 0.006898017134517431f, 0.007267116103321314f, 0.007717895321547985f, 0.007652192376554012f, 0.010717490687966347f, 0.009466593153774738f, 0.005429362878203392f, 0.00535759050399065f, 0.011916860938072205f, 0.006642416585236788f, 0.007236769422888756f, 0.018617482855916023f, 0.007374223321676254f, 0.005701383575797081f, 0.021563945338129997f, 0.004918312653899193f, 0.008574138395488262f, 0.011011775583028793f, 0.012880636379122734f, 0.022946471348404884f, 0.005016472190618515f, 0.007885288447141647f, 0.013454457744956017f, 0.012319841422140598f, 0.0107048274949193f, 0.0105445496737957f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #57 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_9_Add_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(4.5515923500061035f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #58 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_9_Div_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.02192005142569542f),
    AI_PACK_INTQ_ZP(-2)))

/* Int quant #59 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_9_Mul_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.027283966541290283f),
    AI_PACK_INTQ_ZP(18)))

/* Int quant #60 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(4.5515923500061035f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #61 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.01797088235616684f),
    AI_PACK_INTQ_ZP(127)))

/* Int quant #62 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_9_Sqrt_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.133601576089859f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #63 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_ae_encoder_encoder_9_Sub_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.5360717177391052f),
    AI_PACK_INTQ_ZP(47)))

/* Int quant #64 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_decoder_1_bias_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.002720286836847663f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #65 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_decoder_1_weight_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.013229661621153355f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #66 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_decoder_5_bias_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.006605401635169983f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #67 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_decoder_5_weight_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.020323781296610832f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #68 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_decoder_9_bias_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00455534178763628f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #69 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_decoder_9_weight_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.009082471020519733f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #70 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_encoder_1_bias_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00498491944745183f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #71 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_encoder_1_weight_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.03246039152145386f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #72 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_encoder_5_bias_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.004345946945250034f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #73 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_encoder_5_weight_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.028473960235714912f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #74 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_encoder_9_bias_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.006414805073291063f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #75 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(ae_encoder_9_weight_DequantizeLinear_Output_const_3D_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.01207766868174076f),
    AI_PACK_INTQ_ZP(0)))

/* Int quant #76 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(input_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.003766167676076293f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #77 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(reconstruction_QuantizeLinear_Input_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0038983437698334455f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #78 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(reconstruction_QuantizeLinear_Input_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 4,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.005564113613218069f, 0.005266228225082159f, 0.005015726201236248f, 0.003163714660331607f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0)))

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_bias, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_ae_decoder_decoder_0_Gemm_output_0_bias_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_output, AI_STATIC,
  2, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &_ae_decoder_decoder_0_Gemm_output_0_output_array, &_ae_decoder_decoder_0_Gemm_output_0_output_array_intq)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_scratch0, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 163, 1, 1), AI_STRIDE_INIT(4, 2, 2, 326, 326),
  1, &_ae_decoder_decoder_0_Gemm_output_0_scratch0_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_weights, AI_STATIC,
  4, 0x1,
  AI_SHAPE_INIT(4, 3, 32, 1, 1), AI_STRIDE_INIT(4, 1, 3, 96, 96),
  1, &_ae_decoder_decoder_0_Gemm_output_0_weights_array, &_ae_decoder_decoder_0_Gemm_output_0_weights_array_intq)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_10_Relu_output_0_output, AI_STATIC,
  5, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_decoder_decoder_10_Relu_output_0_output_array, &_ae_decoder_decoder_10_Relu_output_0_output_array_intq)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Add_output_0_output, AI_STATIC,
  6, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_1_Add_output_0_output_array, &_ae_decoder_decoder_1_Add_output_0_output_array_intq)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D, AI_STATIC,
  7, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D_array, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Div_output_0_output, AI_STATIC,
  8, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &_ae_decoder_decoder_1_Div_output_0_output_array, &_ae_decoder_decoder_1_Div_output_0_output_array_intq)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Mul_output_0_output, AI_STATIC,
  9, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &_ae_decoder_decoder_1_Mul_output_0_output_array, &_ae_decoder_decoder_1_Mul_output_0_output_array_intq)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Pow_output_0_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_ae_decoder_decoder_1_Pow_output_0_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output, AI_STATIC,
  11, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output_array, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output_array_intq)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output, AI_STATIC,
  14, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output_array, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output_array_intq)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_1_ReduceMean_output_0_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sqrt_output_0_output, AI_STATIC,
  18, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_1_Sqrt_output_0_output_array, &_ae_decoder_decoder_1_Sqrt_output_0_output_array_intq)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_output, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_output_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_output, AI_STATIC,
  20, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &_ae_decoder_decoder_1_Sub_output_0_output_array, &_ae_decoder_decoder_1_Sub_output_0_output_array_intq)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_2_Relu_output_0_output, AI_STATIC,
  21, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &_ae_decoder_decoder_2_Relu_output_0_output_array, &_ae_decoder_decoder_2_Relu_output_0_output_array_intq)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_output, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_output_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_bias, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_ae_decoder_decoder_4_Gemm_output_0_bias_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_output, AI_STATIC,
  24, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &_ae_decoder_decoder_4_Gemm_output_0_output_array, &_ae_decoder_decoder_4_Gemm_output_0_output_array_intq)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_scratch0, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 672, 1, 1), AI_STRIDE_INIT(4, 2, 2, 1344, 1344),
  1, &_ae_decoder_decoder_4_Gemm_output_0_scratch0_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_weights, AI_STATIC,
  26, 0x1,
  AI_SHAPE_INIT(4, 32, 128, 1, 1), AI_STRIDE_INIT(4, 1, 32, 4096, 4096),
  1, &_ae_decoder_decoder_4_Gemm_output_0_weights_array, &_ae_decoder_decoder_4_Gemm_output_0_weights_array_intq)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Add_output_0_output, AI_STATIC,
  27, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_5_Add_output_0_output_array, &_ae_decoder_decoder_5_Add_output_0_output_array_intq)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D, AI_STATIC,
  28, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_array, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Div_output_0_output, AI_STATIC,
  30, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &_ae_decoder_decoder_5_Div_output_0_output_array, &_ae_decoder_decoder_5_Div_output_0_output_array_intq)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Mul_output_0_output, AI_STATIC,
  31, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &_ae_decoder_decoder_5_Mul_output_0_output_array, &_ae_decoder_decoder_5_Mul_output_0_output_array_intq)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Pow_output_0_output, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_ae_decoder_decoder_5_Pow_output_0_output_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output, AI_STATIC,
  33, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output_array, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output_array_intq)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_output, AI_STATIC,
  35, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output, AI_STATIC,
  36, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output_array, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output_array_intq)

/* Tensor #37 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_output, AI_STATIC,
  37, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #38 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale, AI_STATIC,
  38, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale_array, NULL)

/* Tensor #39 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_output, AI_STATIC,
  39, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_5_ReduceMean_output_0_output_array, NULL)

/* Tensor #40 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sqrt_output_0_output, AI_STATIC,
  40, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_5_Sqrt_output_0_output_array, &_ae_decoder_decoder_5_Sqrt_output_0_output_array_intq)

/* Tensor #41 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_output, AI_STATIC,
  41, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_output_array, NULL)

/* Tensor #42 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_output, AI_STATIC,
  42, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &_ae_decoder_decoder_5_Sub_output_0_output_array, &_ae_decoder_decoder_5_Sub_output_0_output_array_intq)

/* Tensor #43 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_6_Relu_output_0_output, AI_STATIC,
  43, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &_ae_decoder_decoder_6_Relu_output_0_output_array, &_ae_decoder_decoder_6_Relu_output_0_output_array_intq)

/* Tensor #44 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_output, AI_STATIC,
  44, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_output_array, NULL)

/* Tensor #45 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_bias, AI_STATIC,
  45, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_decoder_decoder_8_Gemm_output_0_bias_array, NULL)

/* Tensor #46 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_output, AI_STATIC,
  46, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_decoder_decoder_8_Gemm_output_0_output_array, &_ae_decoder_decoder_8_Gemm_output_0_output_array_intq)

/* Tensor #47 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_scratch0, AI_STATIC,
  47, 0x0,
  AI_SHAPE_INIT(4, 1, 1408, 1, 1), AI_STRIDE_INIT(4, 2, 2, 2816, 2816),
  1, &_ae_decoder_decoder_8_Gemm_output_0_scratch0_array, NULL)

/* Tensor #48 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_weights, AI_STATIC,
  48, 0x1,
  AI_SHAPE_INIT(4, 128, 256, 1, 1), AI_STRIDE_INIT(4, 1, 128, 32768, 32768),
  1, &_ae_decoder_decoder_8_Gemm_output_0_weights_array, &_ae_decoder_decoder_8_Gemm_output_0_weights_array_intq)

/* Tensor #49 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_Add_output_0_output, AI_STATIC,
  49, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_9_Add_output_0_output_array, &_ae_decoder_decoder_9_Add_output_0_output_array_intq)

/* Tensor #50 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_Div_output_0_output, AI_STATIC,
  50, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_decoder_decoder_9_Div_output_0_output_array, &_ae_decoder_decoder_9_Div_output_0_output_array_intq)

/* Tensor #51 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_Mul_output_0_output, AI_STATIC,
  51, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_decoder_decoder_9_Mul_output_0_output_array, &_ae_decoder_decoder_9_Mul_output_0_output_array_intq)

/* Tensor #52 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_Pow_output_0_output, AI_STATIC,
  52, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_decoder_decoder_9_Pow_output_0_output_array, NULL)

/* Tensor #53 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output, AI_STATIC,
  53, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output_array, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output_array_intq)

/* Tensor #54 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  54, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #55 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_output, AI_STATIC,
  55, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #56 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output, AI_STATIC,
  56, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output_array, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output_array_intq)

/* Tensor #57 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_output, AI_STATIC,
  57, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #58 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_output, AI_STATIC,
  58, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_decoder_decoder_9_ReduceMean_output_0_output_array, NULL)

/* Tensor #59 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sqrt_output_0_output, AI_STATIC,
  59, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_decoder_decoder_9_Sqrt_output_0_output_array, &_ae_decoder_decoder_9_Sqrt_output_0_output_array_intq)

/* Tensor #60 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_output, AI_STATIC,
  60, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_output_array, NULL)

/* Tensor #61 */
AI_TENSOR_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_output, AI_STATIC,
  61, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_decoder_decoder_9_Sub_output_0_output_array, &_ae_decoder_decoder_9_Sub_output_0_output_array_intq)

/* Tensor #62 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_output, AI_STATIC,
  62, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &_ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_output_array, NULL)

/* Tensor #63 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_bias, AI_STATIC,
  63, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &_ae_encoder_encoder_0_Gemm_output_0_bias_array, NULL)

/* Tensor #64 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_output, AI_STATIC,
  64, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &_ae_encoder_encoder_0_Gemm_output_0_output_array, &_ae_encoder_encoder_0_Gemm_output_0_output_array_intq)

/* Tensor #65 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_scratch0, AI_STATIC,
  65, 0x0,
  AI_SHAPE_INIT(4, 1, 2564, 1, 1), AI_STRIDE_INIT(4, 2, 2, 5128, 5128),
  1, &_ae_encoder_encoder_0_Gemm_output_0_scratch0_array, NULL)

/* Tensor #66 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_weights, AI_STATIC,
  66, 0x1,
  AI_SHAPE_INIT(4, 4, 512, 1, 1), AI_STRIDE_INIT(4, 1, 4, 2048, 2048),
  1, &_ae_encoder_encoder_0_Gemm_output_0_weights_array, &_ae_encoder_encoder_0_Gemm_output_0_weights_array_intq)

/* Tensor #67 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_10_Relu_output_0_output, AI_STATIC,
  67, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &_ae_encoder_encoder_10_Relu_output_0_output_array, &_ae_encoder_encoder_10_Relu_output_0_output_array_intq)

/* Tensor #68 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_bias, AI_STATIC,
  68, 0x0,
  AI_SHAPE_INIT(4, 1, 3, 1, 1), AI_STRIDE_INIT(4, 4, 4, 12, 12),
  1, &_ae_encoder_encoder_12_Gemm_output_0_bias_array, NULL)

/* Tensor #69 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_output, AI_STATIC,
  69, 0x1,
  AI_SHAPE_INIT(4, 1, 3, 1, 1), AI_STRIDE_INIT(4, 1, 1, 3, 3),
  1, &_ae_encoder_encoder_12_Gemm_output_0_output_array, &_ae_encoder_encoder_12_Gemm_output_0_output_array_intq)

/* Tensor #70 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_scratch0, AI_STATIC,
  70, 0x0,
  AI_SHAPE_INIT(4, 1, 79, 1, 1), AI_STRIDE_INIT(4, 2, 2, 158, 158),
  1, &_ae_encoder_encoder_12_Gemm_output_0_scratch0_array, NULL)

/* Tensor #71 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_weights, AI_STATIC,
  71, 0x1,
  AI_SHAPE_INIT(4, 64, 3, 1, 1), AI_STRIDE_INIT(4, 1, 64, 192, 192),
  1, &_ae_encoder_encoder_12_Gemm_output_0_weights_array, &_ae_encoder_encoder_12_Gemm_output_0_weights_array_intq)

/* Tensor #72 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_Add_output_0_output, AI_STATIC,
  72, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_1_Add_output_0_output_array, &_ae_encoder_encoder_1_Add_output_0_output_array_intq)

/* Tensor #73 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_Div_output_0_output, AI_STATIC,
  73, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &_ae_encoder_encoder_1_Div_output_0_output_array, &_ae_encoder_encoder_1_Div_output_0_output_array_intq)

/* Tensor #74 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_Mul_output_0_output, AI_STATIC,
  74, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &_ae_encoder_encoder_1_Mul_output_0_output_array, &_ae_encoder_encoder_1_Mul_output_0_output_array_intq)

/* Tensor #75 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_Pow_output_0_output, AI_STATIC,
  75, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &_ae_encoder_encoder_1_Pow_output_0_output_array, NULL)

/* Tensor #76 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output, AI_STATIC,
  76, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output_array, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output_array_intq)

/* Tensor #77 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  77, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #78 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_output, AI_STATIC,
  78, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #79 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output, AI_STATIC,
  79, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output_array, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output_array_intq)

/* Tensor #80 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias, AI_STATIC,
  80, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias_array, NULL)

/* Tensor #81 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_output, AI_STATIC,
  81, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #82 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale, AI_STATIC,
  82, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale_array, NULL)

/* Tensor #83 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_output, AI_STATIC,
  83, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_1_ReduceMean_output_0_output_array, NULL)

/* Tensor #84 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sqrt_output_0_output, AI_STATIC,
  84, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_1_Sqrt_output_0_output_array, &_ae_encoder_encoder_1_Sqrt_output_0_output_array_intq)

/* Tensor #85 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_output, AI_STATIC,
  85, 0x0,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 4, 4, 2048, 2048),
  1, &_ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_output_array, NULL)

/* Tensor #86 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_output, AI_STATIC,
  86, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &_ae_encoder_encoder_1_Sub_output_0_output_array, &_ae_encoder_encoder_1_Sub_output_0_output_array_intq)

/* Tensor #87 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_2_Relu_output_0_output, AI_STATIC,
  87, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &_ae_encoder_encoder_2_Relu_output_0_output_array, &_ae_encoder_encoder_2_Relu_output_0_output_array_intq)

/* Tensor #88 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_output, AI_STATIC,
  88, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_output_array, NULL)

/* Tensor #89 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_bias, AI_STATIC,
  89, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_encoder_encoder_4_Gemm_output_0_bias_array, NULL)

/* Tensor #90 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_output, AI_STATIC,
  90, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_encoder_encoder_4_Gemm_output_0_output_array, &_ae_encoder_encoder_4_Gemm_output_0_output_array_intq)

/* Tensor #91 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_scratch0, AI_STATIC,
  91, 0x0,
  AI_SHAPE_INIT(4, 1, 1792, 1, 1), AI_STRIDE_INIT(4, 2, 2, 3584, 3584),
  1, &_ae_encoder_encoder_4_Gemm_output_0_scratch0_array, NULL)

/* Tensor #92 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_weights, AI_STATIC,
  92, 0x1,
  AI_SHAPE_INIT(4, 512, 256, 1, 1), AI_STRIDE_INIT(4, 1, 512, 131072, 131072),
  1, &_ae_encoder_encoder_4_Gemm_output_0_weights_array, &_ae_encoder_encoder_4_Gemm_output_0_weights_array_intq)

/* Tensor #93 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_Add_output_0_output, AI_STATIC,
  93, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_5_Add_output_0_output_array, &_ae_encoder_encoder_5_Add_output_0_output_array_intq)

/* Tensor #94 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_Div_output_0_output, AI_STATIC,
  94, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_encoder_encoder_5_Div_output_0_output_array, &_ae_encoder_encoder_5_Div_output_0_output_array_intq)

/* Tensor #95 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_Mul_output_0_output, AI_STATIC,
  95, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_encoder_encoder_5_Mul_output_0_output_array, &_ae_encoder_encoder_5_Mul_output_0_output_array_intq)

/* Tensor #96 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_Pow_output_0_output, AI_STATIC,
  96, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_encoder_encoder_5_Pow_output_0_output_array, NULL)

/* Tensor #97 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output, AI_STATIC,
  97, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output_array, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output_array_intq)

/* Tensor #98 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  98, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #99 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_output, AI_STATIC,
  99, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #100 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output, AI_STATIC,
  100, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output_array, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output_array_intq)

/* Tensor #101 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_output, AI_STATIC,
  101, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #102 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale, AI_STATIC,
  102, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale_array, NULL)

/* Tensor #103 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_output, AI_STATIC,
  103, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_5_ReduceMean_output_0_output_array, NULL)

/* Tensor #104 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sqrt_output_0_output, AI_STATIC,
  104, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_5_Sqrt_output_0_output_array, &_ae_encoder_encoder_5_Sqrt_output_0_output_array_intq)

/* Tensor #105 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_output, AI_STATIC,
  105, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_output_array, NULL)

/* Tensor #106 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_output, AI_STATIC,
  106, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_encoder_encoder_5_Sub_output_0_output_array, &_ae_encoder_encoder_5_Sub_output_0_output_array_intq)

/* Tensor #107 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_6_Relu_output_0_output, AI_STATIC,
  107, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &_ae_encoder_encoder_6_Relu_output_0_output_array, &_ae_encoder_encoder_6_Relu_output_0_output_array_intq)

/* Tensor #108 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_output, AI_STATIC,
  108, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_output_array, NULL)

/* Tensor #109 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_bias, AI_STATIC,
  109, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_ae_encoder_encoder_8_Gemm_output_0_bias_array, NULL)

/* Tensor #110 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_output, AI_STATIC,
  110, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &_ae_encoder_encoder_8_Gemm_output_0_output_array, &_ae_encoder_encoder_8_Gemm_output_0_output_array_intq)

/* Tensor #111 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_scratch0, AI_STATIC,
  111, 0x0,
  AI_SHAPE_INIT(4, 1, 576, 1, 1), AI_STRIDE_INIT(4, 2, 2, 1152, 1152),
  1, &_ae_encoder_encoder_8_Gemm_output_0_scratch0_array, NULL)

/* Tensor #112 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_weights, AI_STATIC,
  112, 0x1,
  AI_SHAPE_INIT(4, 256, 64, 1, 1), AI_STRIDE_INIT(4, 1, 256, 16384, 16384),
  1, &_ae_encoder_encoder_8_Gemm_output_0_weights_array, &_ae_encoder_encoder_8_Gemm_output_0_weights_array_intq)

/* Tensor #113 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_Add_output_0_output, AI_STATIC,
  113, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_9_Add_output_0_output_array, &_ae_encoder_encoder_9_Add_output_0_output_array_intq)

/* Tensor #114 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_Div_output_0_output, AI_STATIC,
  114, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &_ae_encoder_encoder_9_Div_output_0_output_array, &_ae_encoder_encoder_9_Div_output_0_output_array_intq)

/* Tensor #115 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_Mul_output_0_output, AI_STATIC,
  115, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &_ae_encoder_encoder_9_Mul_output_0_output_array, &_ae_encoder_encoder_9_Mul_output_0_output_array_intq)

/* Tensor #116 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_Pow_output_0_output, AI_STATIC,
  116, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_ae_encoder_encoder_9_Pow_output_0_output_array, NULL)

/* Tensor #117 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output, AI_STATIC,
  117, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output_array, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output_array_intq)

/* Tensor #118 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_output, AI_STATIC,
  118, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_output_array, NULL)

/* Tensor #119 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_output, AI_STATIC,
  119, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_output_array, NULL)

/* Tensor #120 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output, AI_STATIC,
  120, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output_array, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output_array_intq)

/* Tensor #121 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_output, AI_STATIC,
  121, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_output_array, NULL)

/* Tensor #122 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale, AI_STATIC,
  122, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale_array, NULL)

/* Tensor #123 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_output, AI_STATIC,
  123, 0x0,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 4, 4, 4, 4),
  1, &_ae_encoder_encoder_9_ReduceMean_output_0_output_array, NULL)

/* Tensor #124 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sqrt_output_0_output, AI_STATIC,
  124, 0x1,
  AI_SHAPE_INIT(4, 1, 1, 1, 1), AI_STRIDE_INIT(4, 1, 1, 1, 1),
  1, &_ae_encoder_encoder_9_Sqrt_output_0_output_array, &_ae_encoder_encoder_9_Sqrt_output_0_output_array_intq)

/* Tensor #125 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_output, AI_STATIC,
  125, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_output_array, NULL)

/* Tensor #126 */
AI_TENSOR_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_output, AI_STATIC,
  126, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &_ae_encoder_encoder_9_Sub_output_0_output_array, &_ae_encoder_encoder_9_Sub_output_0_output_array_intq)

/* Tensor #127 */
AI_TENSOR_OBJ_DECLARE(
  ae_decoder_1_bias_DequantizeLinear_Output_const_3D, AI_STATIC,
  127, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &ae_decoder_1_bias_DequantizeLinear_Output_const_3D_array, &ae_decoder_1_bias_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #128 */
AI_TENSOR_OBJ_DECLARE(
  ae_decoder_1_weight_DequantizeLinear_Output_const_3D, AI_STATIC,
  128, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &ae_decoder_1_weight_DequantizeLinear_Output_const_3D_array, &ae_decoder_1_weight_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #129 */
AI_TENSOR_OBJ_DECLARE(
  ae_decoder_5_bias_DequantizeLinear_Output_const_3D, AI_STATIC,
  129, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &ae_decoder_5_bias_DequantizeLinear_Output_const_3D_array, &ae_decoder_5_bias_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #130 */
AI_TENSOR_OBJ_DECLARE(
  ae_decoder_5_weight_DequantizeLinear_Output_const_3D, AI_STATIC,
  130, 0x1,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 1, 1, 128, 128),
  1, &ae_decoder_5_weight_DequantizeLinear_Output_const_3D_array, &ae_decoder_5_weight_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #131 */
AI_TENSOR_OBJ_DECLARE(
  ae_decoder_9_bias_DequantizeLinear_Output_const_3D, AI_STATIC,
  131, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &ae_decoder_9_bias_DequantizeLinear_Output_const_3D_array, &ae_decoder_9_bias_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #132 */
AI_TENSOR_OBJ_DECLARE(
  ae_decoder_9_weight_DequantizeLinear_Output_const_3D, AI_STATIC,
  132, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &ae_decoder_9_weight_DequantizeLinear_Output_const_3D_array, &ae_decoder_9_weight_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #133 */
AI_TENSOR_OBJ_DECLARE(
  ae_encoder_1_bias_DequantizeLinear_Output_const_3D, AI_STATIC,
  133, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &ae_encoder_1_bias_DequantizeLinear_Output_const_3D_array, &ae_encoder_1_bias_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #134 */
AI_TENSOR_OBJ_DECLARE(
  ae_encoder_1_weight_DequantizeLinear_Output_const_3D, AI_STATIC,
  134, 0x1,
  AI_SHAPE_INIT(4, 1, 512, 1, 1), AI_STRIDE_INIT(4, 1, 1, 512, 512),
  1, &ae_encoder_1_weight_DequantizeLinear_Output_const_3D_array, &ae_encoder_1_weight_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #135 */
AI_TENSOR_OBJ_DECLARE(
  ae_encoder_5_bias_DequantizeLinear_Output_const_3D, AI_STATIC,
  135, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &ae_encoder_5_bias_DequantizeLinear_Output_const_3D_array, &ae_encoder_5_bias_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #136 */
AI_TENSOR_OBJ_DECLARE(
  ae_encoder_5_weight_DequantizeLinear_Output_const_3D, AI_STATIC,
  136, 0x1,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 1, 1, 256, 256),
  1, &ae_encoder_5_weight_DequantizeLinear_Output_const_3D_array, &ae_encoder_5_weight_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #137 */
AI_TENSOR_OBJ_DECLARE(
  ae_encoder_9_bias_DequantizeLinear_Output_const_3D, AI_STATIC,
  137, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &ae_encoder_9_bias_DequantizeLinear_Output_const_3D_array, &ae_encoder_9_bias_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #138 */
AI_TENSOR_OBJ_DECLARE(
  ae_encoder_9_weight_DequantizeLinear_Output_const_3D, AI_STATIC,
  138, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &ae_encoder_9_weight_DequantizeLinear_Output_const_3D_array, &ae_encoder_9_weight_DequantizeLinear_Output_const_3D_array_intq)

/* Tensor #139 */
AI_TENSOR_OBJ_DECLARE(
  input_output, AI_STATIC,
  139, 0x1,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 1, 1, 4, 4),
  1, &input_output_array, &input_output_array_intq)

/* Tensor #140 */
AI_TENSOR_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_bias, AI_STATIC,
  140, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &reconstruction_QuantizeLinear_Input_bias_array, NULL)

/* Tensor #141 */
AI_TENSOR_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_output, AI_STATIC,
  141, 0x1,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 1, 1, 4, 4),
  1, &reconstruction_QuantizeLinear_Input_output_array, &reconstruction_QuantizeLinear_Input_output_array_intq)

/* Tensor #142 */
AI_TENSOR_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_scratch0, AI_STATIC,
  142, 0x0,
  AI_SHAPE_INIT(4, 1, 276, 1, 1), AI_STRIDE_INIT(4, 2, 2, 552, 552),
  1, &reconstruction_QuantizeLinear_Input_scratch0_array, NULL)

/* Tensor #143 */
AI_TENSOR_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_weights, AI_STATIC,
  143, 0x1,
  AI_SHAPE_INIT(4, 256, 4, 1, 1), AI_STRIDE_INIT(4, 1, 256, 1024, 1024),
  1, &reconstruction_QuantizeLinear_Input_weights_array, &reconstruction_QuantizeLinear_Input_weights_array_intq)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &reconstruction_QuantizeLinear_Input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &reconstruction_QuantizeLinear_Input_weights, &reconstruction_QuantizeLinear_Input_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &reconstruction_QuantizeLinear_Input_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  reconstruction_QuantizeLinear_Input_layer, 216,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &reconstruction_QuantizeLinear_Input_chain,
  NULL, &reconstruction_QuantizeLinear_Input_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_10_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_9_Mul_output_0_output, &ae_decoder_9_bias_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_10_Relu_output_0_layer, 213,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_10_Relu_output_0_chain,
  NULL, &reconstruction_QuantizeLinear_Input_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_9_Div_output_0_output, &ae_decoder_9_weight_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_Mul_output_0_layer, 210,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_9_Mul_output_0_chain,
  NULL, &_ae_decoder_decoder_10_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_9_Sub_output_0_output, &_ae_decoder_decoder_9_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_Div_output_0_layer, 207,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_9_Div_output_0_chain,
  NULL, &_ae_decoder_decoder_9_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_INT8, 
)


AI_STATIC_CONST ai_i8 _ae_decoder_decoder_9_Sqrt_output_0_nl_params_data[] = { -128, -112, -105, -100, -96, -92, -89, -86, -83, -80, -78, -75, -73, -70, -68, -66, -64, -62, -60, -58, -57, -55, -53, -51, -50, -48, -47, -45, -44, -42, -41, -39, -38, -36, -35, -34, -32, -31, -30, -28, -27, -26, -25, -23, -22, -21, -20, -19, -17, -16, -15, -14, -13, -12, -11, -10, -9, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 42, 43, 44, 45, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 53, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 76, 77, 78, 78, 79, 80, 80, 81, 81, 82, 83, 83, 84, 84, 85, 86, 86, 87, 87, 88, 89, 89, 90, 90, 91, 92, 92, 93, 93, 94, 94, 95, 96, 96, 97, 97, 98, 98, 99, 100, 100, 101, 101, 102, 102, 103, 103, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111, 112, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_9_Sqrt_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _ae_decoder_decoder_9_Sqrt_output_0_nl_params_data, _ae_decoder_decoder_9_Sqrt_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sqrt_output_0_layer, 204,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_ae_decoder_decoder_9_Sqrt_output_0_chain,
  NULL, &_ae_decoder_decoder_9_Div_output_0_layer, AI_STATIC, 
  .nl_params = &_ae_decoder_decoder_9_Sqrt_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_Add_output_0_layer, 201,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_9_Add_output_0_chain,
  NULL, &_ae_decoder_decoder_9_Sqrt_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_layer, 198,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_9_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_layer, 198,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_chain,
  NULL, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_decoder_decoder_9_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_9_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_decoder_decoder_9_ReduceMean_1_output_0_neutral_value_data, _ae_decoder_decoder_9_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_1_output_0_layer, 198,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_decoder_decoder_9_ReduceMean_1_output_0_chain,
  NULL, &_ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_decoder_decoder_9_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_output, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_Pow_output_0_layer, 195,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_ae_decoder_decoder_9_Pow_output_0_chain,
  NULL, &_ae_decoder_decoder_9_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_layer, 192,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_9_Pow_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_8_Gemm_output_0_output, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_Sub_output_0_layer, 192,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_9_Sub_output_0_chain,
  NULL, &_ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_layer, 189,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_9_Sub_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_Mul_layer, 189,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_chain,
  NULL, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_decoder_decoder_9_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_9_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_decoder_decoder_9_ReduceMean_output_0_neutral_value_data, _ae_decoder_decoder_9_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_9_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_9_ReduceMean_output_0_layer, 189,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_decoder_decoder_9_ReduceMean_output_0_chain,
  NULL, &_ae_decoder_decoder_9_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_decoder_decoder_9_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_8_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_layer, 186,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_9_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_8_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_8_Gemm_output_0_weights, &_ae_decoder_decoder_8_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_8_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_8_Gemm_output_0_layer, 186,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_ae_decoder_decoder_8_Gemm_output_0_chain,
  NULL, &_ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_6_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_5_Mul_output_0_output, &ae_decoder_5_bias_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_6_Relu_output_0_layer, 183,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_6_Relu_output_0_chain,
  NULL, &_ae_decoder_decoder_8_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_5_Div_output_0_output, &ae_decoder_5_weight_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Mul_output_0_layer, 180,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_5_Mul_output_0_chain,
  NULL, &_ae_decoder_decoder_6_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_5_Sub_output_0_output, &_ae_decoder_decoder_5_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Div_output_0_layer, 177,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_5_Div_output_0_chain,
  NULL, &_ae_decoder_decoder_5_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_INT8, 
)


AI_STATIC_CONST ai_i8 _ae_decoder_decoder_5_Sqrt_output_0_nl_params_data[] = { -128, -112, -105, -100, -96, -92, -89, -86, -83, -80, -78, -75, -73, -70, -68, -66, -64, -62, -60, -58, -57, -55, -53, -51, -50, -48, -47, -45, -44, -42, -41, -39, -38, -36, -35, -34, -32, -31, -30, -28, -27, -26, -25, -23, -22, -21, -20, -19, -17, -16, -15, -14, -13, -12, -11, -10, -9, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 42, 43, 44, 45, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 53, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 76, 77, 78, 78, 79, 80, 80, 81, 81, 82, 83, 83, 84, 84, 85, 86, 86, 87, 87, 88, 89, 89, 90, 90, 91, 92, 92, 93, 93, 94, 94, 95, 96, 96, 97, 97, 98, 98, 99, 100, 100, 101, 101, 102, 102, 103, 103, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111, 112, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_5_Sqrt_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _ae_decoder_decoder_5_Sqrt_output_0_nl_params_data, _ae_decoder_decoder_5_Sqrt_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sqrt_output_0_layer, 174,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_ae_decoder_decoder_5_Sqrt_output_0_chain,
  NULL, &_ae_decoder_decoder_5_Div_output_0_layer, AI_STATIC, 
  .nl_params = &_ae_decoder_decoder_5_Sqrt_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Add_output_0_layer, 171,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_5_Add_output_0_chain,
  NULL, &_ae_decoder_decoder_5_Sqrt_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_layer, 168,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_5_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_layer, 168,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_chain,
  NULL, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_decoder_decoder_5_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_5_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_decoder_decoder_5_ReduceMean_1_output_0_neutral_value_data, _ae_decoder_decoder_5_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_1_output_0_layer, 168,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_decoder_decoder_5_ReduceMean_1_output_0_chain,
  NULL, &_ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_decoder_decoder_5_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_output, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Pow_output_0_layer, 165,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_ae_decoder_decoder_5_Pow_output_0_chain,
  NULL, &_ae_decoder_decoder_5_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_layer, 162,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_5_Pow_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_4_Gemm_output_0_output, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Sub_output_0_layer, 162,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_5_Sub_output_0_chain,
  NULL, &_ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_layer, 159,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_5_Sub_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_Mul_layer, 159,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_chain,
  NULL, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_decoder_decoder_5_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_5_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_decoder_decoder_5_ReduceMean_output_0_neutral_value_data, _ae_decoder_decoder_5_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_ReduceMean_output_0_layer, 159,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_decoder_decoder_5_ReduceMean_output_0_chain,
  NULL, &_ae_decoder_decoder_5_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_decoder_decoder_5_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_layer, 156,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_5_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_4_Gemm_output_0_weights, &_ae_decoder_decoder_4_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_4_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_4_Gemm_output_0_layer, 156,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_ae_decoder_decoder_4_Gemm_output_0_chain,
  NULL, &_ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_2_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_1_Mul_output_0_output, &ae_decoder_1_bias_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_2_Relu_output_0_layer, 153,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_2_Relu_output_0_chain,
  NULL, &_ae_decoder_decoder_4_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_1_Div_output_0_output, &ae_decoder_1_weight_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_Mul_output_0_layer, 150,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_1_Mul_output_0_chain,
  NULL, &_ae_decoder_decoder_2_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_1_Sub_output_0_output, &_ae_decoder_decoder_1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_Div_output_0_layer, 147,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_1_Div_output_0_chain,
  NULL, &_ae_decoder_decoder_1_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_INT8, 
)


AI_STATIC_CONST ai_i8 _ae_decoder_decoder_1_Sqrt_output_0_nl_params_data[] = { -128, -112, -105, -100, -96, -92, -89, -86, -83, -80, -78, -75, -73, -70, -68, -66, -64, -62, -60, -58, -57, -55, -53, -51, -50, -48, -47, -45, -44, -42, -41, -39, -38, -36, -35, -34, -32, -31, -30, -28, -27, -26, -25, -23, -22, -21, -20, -19, -17, -16, -15, -14, -13, -12, -11, -10, -9, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 42, 43, 44, 45, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 53, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 76, 77, 78, 78, 79, 80, 80, 81, 81, 82, 83, 83, 84, 84, 85, 86, 86, 87, 87, 88, 89, 89, 90, 90, 91, 92, 92, 93, 93, 94, 94, 95, 96, 96, 97, 97, 98, 98, 99, 100, 100, 101, 101, 102, 102, 103, 103, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111, 112, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_1_Sqrt_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _ae_decoder_decoder_1_Sqrt_output_0_nl_params_data, _ae_decoder_decoder_1_Sqrt_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sqrt_output_0_layer, 144,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_ae_decoder_decoder_1_Sqrt_output_0_chain,
  NULL, &_ae_decoder_decoder_1_Div_output_0_layer, AI_STATIC, 
  .nl_params = &_ae_decoder_decoder_1_Sqrt_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_Add_output_0_layer, 141,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_1_Add_output_0_chain,
  NULL, &_ae_decoder_decoder_1_Sqrt_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_layer, 138,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_1_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_layer, 138,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_chain,
  NULL, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_decoder_decoder_1_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_1_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_decoder_decoder_1_ReduceMean_1_output_0_neutral_value_data, _ae_decoder_decoder_1_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_1_output_0_layer, 138,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_decoder_decoder_1_ReduceMean_1_output_0_chain,
  NULL, &_ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_decoder_decoder_1_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_output, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_Pow_output_0_layer, 135,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_ae_decoder_decoder_1_Pow_output_0_chain,
  NULL, &_ae_decoder_decoder_1_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_layer, 132,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_1_Pow_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_0_Gemm_output_0_output, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_Sub_output_0_layer, 132,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_decoder_decoder_1_Sub_output_0_chain,
  NULL, &_ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_layer, 129,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_1_Sub_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_Mul_layer, 129,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_chain,
  NULL, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_decoder_decoder_1_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_decoder_decoder_1_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_decoder_decoder_1_ReduceMean_output_0_neutral_value_data, _ae_decoder_decoder_1_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_1_ReduceMean_output_0_layer, 129,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_decoder_decoder_1_ReduceMean_output_0_chain,
  NULL, &_ae_decoder_decoder_1_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_decoder_decoder_1_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_layer, 126,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_chain,
  NULL, &_ae_decoder_decoder_1_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_12_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_decoder_decoder_0_Gemm_output_0_weights, &_ae_decoder_decoder_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_0_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_0_Gemm_output_0_layer, 126,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_ae_decoder_decoder_0_Gemm_output_0_chain,
  NULL, &_ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_12_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_12_Gemm_output_0_weights, &_ae_encoder_encoder_12_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_12_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_12_Gemm_output_0_layer, 123,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_ae_encoder_encoder_12_Gemm_output_0_chain,
  NULL, &_ae_decoder_decoder_0_Gemm_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_10_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_9_Mul_output_0_output, &ae_encoder_9_bias_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_10_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_10_Relu_output_0_layer, 120,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_10_Relu_output_0_chain,
  NULL, &_ae_encoder_encoder_12_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_9_Div_output_0_output, &ae_encoder_9_weight_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_Mul_output_0_layer, 117,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_9_Mul_output_0_chain,
  NULL, &_ae_encoder_encoder_10_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_9_Sub_output_0_output, &_ae_encoder_encoder_9_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_Div_output_0_layer, 114,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_9_Div_output_0_chain,
  NULL, &_ae_encoder_encoder_9_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_INT8, 
)


AI_STATIC_CONST ai_i8 _ae_encoder_encoder_9_Sqrt_output_0_nl_params_data[] = { -128, -112, -105, -100, -96, -92, -89, -86, -83, -80, -78, -75, -73, -70, -68, -66, -64, -62, -60, -58, -57, -55, -53, -51, -50, -48, -47, -45, -44, -42, -41, -39, -38, -36, -35, -34, -32, -31, -30, -28, -27, -26, -25, -23, -22, -21, -20, -19, -17, -16, -15, -14, -13, -12, -11, -10, -9, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 42, 43, 44, 45, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 53, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 76, 77, 78, 78, 79, 80, 80, 81, 81, 82, 83, 83, 84, 84, 85, 86, 86, 87, 87, 88, 89, 89, 90, 90, 91, 92, 92, 93, 93, 94, 94, 95, 96, 96, 97, 97, 98, 98, 99, 100, 100, 101, 101, 102, 102, 103, 103, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111, 112, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_9_Sqrt_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _ae_encoder_encoder_9_Sqrt_output_0_nl_params_data, _ae_encoder_encoder_9_Sqrt_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sqrt_output_0_layer, 111,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_ae_encoder_encoder_9_Sqrt_output_0_chain,
  NULL, &_ae_encoder_encoder_9_Div_output_0_layer, AI_STATIC, 
  .nl_params = &_ae_encoder_encoder_9_Sqrt_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_Add_output_0_layer, 108,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_9_Add_output_0_chain,
  NULL, &_ae_encoder_encoder_9_Sqrt_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_layer, 105,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_9_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_layer, 105,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_chain,
  NULL, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_encoder_encoder_9_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_9_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_encoder_encoder_9_ReduceMean_1_output_0_neutral_value_data, _ae_encoder_encoder_9_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_1_output_0_layer, 105,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_encoder_encoder_9_ReduceMean_1_output_0_chain,
  NULL, &_ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_encoder_encoder_9_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_output, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_Pow_output_0_layer, 102,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_ae_encoder_encoder_9_Pow_output_0_chain,
  NULL, &_ae_encoder_encoder_9_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_layer, 99,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_9_Pow_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_8_Gemm_output_0_output, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_Sub_output_0_layer, 99,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_9_Sub_output_0_chain,
  NULL, &_ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_layer, 96,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_9_Sub_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_Mul_layer, 96,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_chain,
  NULL, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_encoder_encoder_9_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_9_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_encoder_encoder_9_ReduceMean_output_0_neutral_value_data, _ae_encoder_encoder_9_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_9_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_9_ReduceMean_output_0_layer, 96,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_encoder_encoder_9_ReduceMean_output_0_chain,
  NULL, &_ae_encoder_encoder_9_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_encoder_encoder_9_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_8_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_layer, 93,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_9_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_8_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_8_Gemm_output_0_weights, &_ae_encoder_encoder_8_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_8_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_8_Gemm_output_0_layer, 93,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_ae_encoder_encoder_8_Gemm_output_0_chain,
  NULL, &_ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_6_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_Mul_output_0_output, &ae_encoder_5_bias_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_6_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_6_Relu_output_0_layer, 90,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_6_Relu_output_0_chain,
  NULL, &_ae_encoder_encoder_8_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_Div_output_0_output, &ae_encoder_5_weight_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_Mul_output_0_layer, 87,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_5_Mul_output_0_chain,
  NULL, &_ae_encoder_encoder_6_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_Sub_output_0_output, &_ae_encoder_encoder_5_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_Div_output_0_layer, 84,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_5_Div_output_0_chain,
  NULL, &_ae_encoder_encoder_5_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_INT8, 
)


AI_STATIC_CONST ai_i8 _ae_encoder_encoder_5_Sqrt_output_0_nl_params_data[] = { -128, -112, -105, -100, -96, -92, -89, -86, -83, -80, -78, -75, -73, -70, -68, -66, -64, -62, -60, -58, -57, -55, -53, -51, -50, -48, -47, -45, -44, -42, -41, -39, -38, -36, -35, -34, -32, -31, -30, -28, -27, -26, -25, -23, -22, -21, -20, -19, -17, -16, -15, -14, -13, -12, -11, -10, -9, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 42, 43, 44, 45, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 53, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 76, 77, 78, 78, 79, 80, 80, 81, 81, 82, 83, 83, 84, 84, 85, 86, 86, 87, 87, 88, 89, 89, 90, 90, 91, 92, 92, 93, 93, 94, 94, 95, 96, 96, 97, 97, 98, 98, 99, 100, 100, 101, 101, 102, 102, 103, 103, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111, 112, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_5_Sqrt_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _ae_encoder_encoder_5_Sqrt_output_0_nl_params_data, _ae_encoder_encoder_5_Sqrt_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sqrt_output_0_layer, 81,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_ae_encoder_encoder_5_Sqrt_output_0_chain,
  NULL, &_ae_encoder_encoder_5_Div_output_0_layer, AI_STATIC, 
  .nl_params = &_ae_encoder_encoder_5_Sqrt_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_Add_output_0_layer, 78,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_5_Add_output_0_chain,
  NULL, &_ae_encoder_encoder_5_Sqrt_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_layer, 75,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_5_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_layer, 75,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_chain,
  NULL, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_encoder_encoder_5_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_5_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_encoder_encoder_5_ReduceMean_1_output_0_neutral_value_data, _ae_encoder_encoder_5_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_1_output_0_layer, 75,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_encoder_encoder_5_ReduceMean_1_output_0_chain,
  NULL, &_ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_encoder_encoder_5_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_output, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_Pow_output_0_layer, 72,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_ae_encoder_encoder_5_Pow_output_0_chain,
  NULL, &_ae_encoder_encoder_5_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_layer, 69,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_5_Pow_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_4_Gemm_output_0_output, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_Sub_output_0_layer, 69,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_5_Sub_output_0_chain,
  NULL, &_ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_layer, 66,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_5_Sub_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_Mul_layer, 66,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_chain,
  NULL, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_encoder_encoder_5_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_5_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_encoder_encoder_5_ReduceMean_output_0_neutral_value_data, _ae_encoder_encoder_5_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_5_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_5_ReduceMean_output_0_layer, 66,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_encoder_encoder_5_ReduceMean_output_0_chain,
  NULL, &_ae_encoder_encoder_5_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_encoder_encoder_5_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_layer, 63,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_5_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_4_Gemm_output_0_weights, &_ae_encoder_encoder_4_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_4_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_4_Gemm_output_0_layer, 63,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_ae_encoder_encoder_4_Gemm_output_0_chain,
  NULL, &_ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_2_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_1_Mul_output_0_output, &ae_encoder_1_bias_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_2_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_2_Relu_output_0_layer, 60,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_2_Relu_output_0_chain,
  NULL, &_ae_encoder_encoder_4_Gemm_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_Mul_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_1_Div_output_0_output, &ae_encoder_1_weight_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Mul_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_Mul_output_0_layer, 57,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_1_Mul_output_0_chain,
  NULL, &_ae_encoder_encoder_2_Relu_output_0_layer, AI_STATIC, 
  .operation = ai_mul_f32, 
  .buffer_operation = ai_mul_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_Div_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_1_Sub_output_0_output, &_ae_encoder_encoder_1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Div_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_Div_output_0_layer, 54,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_1_Div_output_0_chain,
  NULL, &_ae_encoder_encoder_1_Mul_output_0_layer, AI_STATIC, 
  .operation = ai_div_f32, 
  .buffer_operation = ai_div_buffer_INT8, 
)


AI_STATIC_CONST ai_i8 _ae_encoder_encoder_1_Sqrt_output_0_nl_params_data[] = { -128, -112, -105, -100, -96, -92, -89, -86, -83, -80, -78, -75, -73, -70, -68, -66, -64, -62, -60, -58, -57, -55, -53, -51, -50, -48, -47, -45, -44, -42, -41, -39, -38, -36, -35, -34, -32, -31, -30, -28, -27, -26, -25, -23, -22, -21, -20, -19, -17, -16, -15, -14, -13, -12, -11, -10, -9, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 27, 28, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 36, 37, 38, 39, 39, 40, 41, 42, 42, 43, 44, 45, 45, 46, 47, 48, 48, 49, 50, 51, 51, 52, 53, 53, 54, 55, 55, 56, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64, 65, 66, 66, 67, 68, 68, 69, 70, 70, 71, 71, 72, 73, 73, 74, 75, 75, 76, 76, 77, 78, 78, 79, 80, 80, 81, 81, 82, 83, 83, 84, 84, 85, 86, 86, 87, 87, 88, 89, 89, 90, 90, 91, 92, 92, 93, 93, 94, 94, 95, 96, 96, 97, 97, 98, 98, 99, 100, 100, 101, 101, 102, 102, 103, 103, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111, 112, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 127 };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_1_Sqrt_output_0_nl_params, AI_ARRAY_FORMAT_S8,
    _ae_encoder_encoder_1_Sqrt_output_0_nl_params_data, _ae_encoder_encoder_1_Sqrt_output_0_nl_params_data, 256, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sqrt_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Sqrt_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sqrt_output_0_layer, 51,
  NL_TYPE, 0x0, NULL,
  nl, forward_nl_integer,
  &_ae_encoder_encoder_1_Sqrt_output_0_chain,
  NULL, &_ae_encoder_encoder_1_Div_output_0_layer, AI_STATIC, 
  .nl_params = &_ae_encoder_encoder_1_Sqrt_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_Add_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output, &_ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Add_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_Add_output_0_layer, 48,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_1_Add_output_0_chain,
  NULL, &_ae_encoder_encoder_1_Sqrt_output_0_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_layer, 45,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_1_Add_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_layer, 45,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_chain,
  NULL, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_encoder_encoder_1_ReduceMean_1_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_1_ReduceMean_1_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_encoder_encoder_1_ReduceMean_1_output_0_neutral_value_data, _ae_encoder_encoder_1_ReduceMean_1_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_1_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_1_output_0_layer, 45,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_encoder_encoder_1_ReduceMean_1_output_0_chain,
  NULL, &_ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_encoder_encoder_1_ReduceMean_1_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_Pow_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_output, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Pow_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_Pow_output_0_layer, 42,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &_ae_encoder_encoder_1_Pow_output_0_chain,
  NULL, &_ae_encoder_encoder_1_ReduceMean_1_output_0_layer, AI_STATIC, 
  .operation = ai_pow, 
  .buffer_operation = ai_pow_buffer, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_layer, 39,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_1_Pow_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_0_Gemm_output_0_output, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_Sub_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_Sub_output_0_layer, 39,
  ELTWISE_INTEGER_TYPE, 0x0, NULL,
  eltwise_integer, forward_eltwise_integer_INT8,
  &_ae_encoder_encoder_1_Sub_output_0_chain,
  NULL, &_ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_layer, AI_STATIC, 
  .operation = ai_sub_f32, 
  .buffer_operation = ai_sub_buffer_INT8, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_layer, 36,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_1_Sub_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_Mul_layer, 36,
  BN_TYPE, 0x0, NULL,
  bn, forward_bn,
  &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_chain,
  NULL, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _ae_encoder_encoder_1_ReduceMean_output_0_neutral_value_data[] = { 0.0f };
AI_ARRAY_OBJ_DECLARE(
    _ae_encoder_encoder_1_ReduceMean_output_0_neutral_value, AI_ARRAY_FORMAT_FLOAT,
    _ae_encoder_encoder_1_ReduceMean_output_0_neutral_value_data, _ae_encoder_encoder_1_ReduceMean_output_0_neutral_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_1_ReduceMean_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_1_ReduceMean_output_0_layer, 36,
  REDUCE_TYPE, 0x0, NULL,
  reduce, forward_reduce,
  &_ae_encoder_encoder_1_ReduceMean_output_0_chain,
  NULL, &_ae_encoder_encoder_1_ReduceMean_output_0_Mul_layer, AI_STATIC, 
  .operation = ai_sum, 
  .neutral_value = &_ae_encoder_encoder_1_ReduceMean_output_0_neutral_value, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_layer, 33,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_chain,
  NULL, &_ae_encoder_encoder_1_ReduceMean_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_ae_encoder_encoder_0_Gemm_output_0_weights, &_ae_encoder_encoder_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_encoder_encoder_0_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _ae_encoder_encoder_0_Gemm_output_0_layer, 33,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_ae_encoder_encoder_0_Gemm_output_0_chain,
  NULL, &_ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_layer, 2,
  NL_TYPE, 0x0, NULL,
  nl, node_convert,
  &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_chain,
  NULL, &_ae_encoder_encoder_0_Gemm_output_0_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 195228, 1, 1),
    195228, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 5648, 1, 1),
    5648, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &reconstruction_QuantizeLinear_Input_output),
  &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_layer, 0xa49b23b8, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 195228, 1, 1),
      195228, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 5648, 1, 1),
      5648, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &reconstruction_QuantizeLinear_Input_output),
  &_ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_layer, 0xa49b23b8, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    input_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    input_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    _ae_encoder_encoder_0_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_0_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_0_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 5136);
    _ae_encoder_encoder_0_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 5136);
    _ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_0_Gemm_output_0_0_0__ae_encoder_encoder_1_ReduceMean_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_1_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_1_Sub_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_1_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_Sub_output_0_0_0__ae_encoder_encoder_1_Pow_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 2568);
    _ae_encoder_encoder_1_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 2568);
    _ae_encoder_encoder_1_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_1_Add_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_1_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_1_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_1_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_2_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_2_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_4_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 1032);
    _ae_encoder_encoder_4_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 1032);
    _ae_encoder_encoder_4_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_4_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_4_Gemm_output_0_0_0__ae_encoder_encoder_5_ReduceMean_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_5_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_5_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_5_Sub_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_5_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_5_Sub_output_0_0_0__ae_encoder_encoder_5_Pow_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _ae_encoder_encoder_5_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1544);
    _ae_encoder_encoder_5_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1544);
    _ae_encoder_encoder_5_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_5_Add_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_5_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_5_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_5_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_5_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_5_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_5_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_6_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_6_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_8_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_8_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_encoder_encoder_8_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1416);
    _ae_encoder_encoder_8_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1416);
    _ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_8_Gemm_output_0_0_0__ae_encoder_encoder_9_ReduceMean_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_9_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_9_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_ReduceMean_output_0_Mul_0_1__ae_encoder_encoder_9_Sub_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_9_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_Sub_output_0_0_0__ae_encoder_encoder_9_Pow_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 328);
    _ae_encoder_encoder_9_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 328);
    _ae_encoder_encoder_9_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_ReduceMean_1_output_0_Mul_0_0__ae_encoder_encoder_9_Add_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_9_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_9_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_9_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_encoder_encoder_10_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_10_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 72);
    _ae_encoder_encoder_12_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 136);
    _ae_encoder_encoder_12_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 136);
    _ae_encoder_encoder_12_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_encoder_encoder_12_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_0_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_0_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_0_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 336);
    _ae_decoder_decoder_0_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 336);
    _ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_0_Gemm_output_0_0_0__ae_decoder_decoder_1_ReduceMean_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_1_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_1_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_1_Sub_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_1_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_Sub_output_0_0_0__ae_decoder_decoder_1_Pow_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 168);
    _ae_decoder_decoder_1_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 168);
    _ae_decoder_decoder_1_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_1_Add_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_1_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_1_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_1_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_2_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_2_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 40);
    _ae_decoder_decoder_4_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 72);
    _ae_decoder_decoder_4_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 72);
    _ae_decoder_decoder_4_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1416);
    _ae_decoder_decoder_4_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1416);
    _ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_4_Gemm_output_0_0_0__ae_decoder_decoder_5_ReduceMean_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_5_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_5_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_5_Sub_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_5_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_Sub_output_0_0_0__ae_decoder_decoder_5_Pow_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 648);
    _ae_decoder_decoder_5_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 648);
    _ae_decoder_decoder_5_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_5_Add_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_5_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_5_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_5_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_6_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_6_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 136);
    _ae_decoder_decoder_8_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_decoder_decoder_8_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_decoder_decoder_8_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 3080);
    _ae_decoder_decoder_8_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 3080);
    _ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_8_Gemm_output_0_0_0__ae_decoder_decoder_9_ReduceMean_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_9_ReduceMean_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_ReduceMean_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_ReduceMean_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_9_ReduceMean_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_ReduceMean_output_0_Mul_0_1__ae_decoder_decoder_9_Sub_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_Sub_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_9_Sub_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 8);
    _ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_decoder_decoder_9_Sub_output_0_0_0__ae_decoder_decoder_9_Pow_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_decoder_decoder_9_Pow_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1288);
    _ae_decoder_decoder_9_Pow_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1288);
    _ae_decoder_decoder_9_ReduceMean_1_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_ReduceMean_1_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_ReduceMean_1_output_0_Mul_0_0__ae_decoder_decoder_9_Add_output_0_conversion_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_Add_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 4);
    _ae_decoder_decoder_9_Add_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    _ae_decoder_decoder_9_Sqrt_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_Sqrt_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_Div_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 264);
    _ae_decoder_decoder_9_Div_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 264);
    _ae_decoder_decoder_9_Mul_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_9_Mul_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _ae_decoder_decoder_10_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 256);
    _ae_decoder_decoder_10_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 256);
    reconstruction_QuantizeLinear_Input_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 512);
    reconstruction_QuantizeLinear_Input_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    reconstruction_QuantizeLinear_Input_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    reconstruction_QuantizeLinear_Input_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_network_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    ae_encoder_9_weight_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_encoder_9_weight_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 0);
    ae_encoder_9_weight_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    ae_encoder_9_bias_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_encoder_9_bias_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 64);
    ae_encoder_9_bias_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 64);
    ae_encoder_5_weight_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_encoder_5_weight_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 128);
    ae_encoder_5_weight_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 128);
    ae_encoder_5_bias_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_encoder_5_bias_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 384);
    ae_encoder_5_bias_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 384);
    ae_encoder_1_weight_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_encoder_1_weight_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 640);
    ae_encoder_1_weight_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 640);
    ae_encoder_1_bias_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_encoder_1_bias_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 1152);
    ae_encoder_1_bias_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 1152);
    ae_decoder_9_weight_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_decoder_9_weight_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 1664);
    ae_decoder_9_weight_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 1664);
    ae_decoder_9_bias_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_decoder_9_bias_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 1920);
    ae_decoder_9_bias_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 1920);
    ae_decoder_5_weight_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_decoder_5_weight_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 2176);
    ae_decoder_5_weight_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 2176);
    ae_decoder_5_bias_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_decoder_5_bias_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 2304);
    ae_decoder_5_bias_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 2304);
    ae_decoder_1_weight_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_decoder_1_weight_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 2432);
    ae_decoder_1_weight_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 2432);
    ae_decoder_1_bias_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    ae_decoder_1_bias_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 2464);
    ae_decoder_1_bias_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 2464);
    _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 2496);
    _ae_decoder_decoder_5_Constant_output_0_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 2496);
    _ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D_array.data = AI_PTR(g_network_weights_map[0] + 2500);
    _ae_decoder_decoder_1_Constant_1_output_0_DequantizeLinear_Output_const_3D_array.data_start = AI_PTR(g_network_weights_map[0] + 2500);
    _ae_encoder_encoder_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_0_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 2504);
    _ae_encoder_encoder_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 2504);
    _ae_encoder_encoder_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_0_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 4552);
    _ae_encoder_encoder_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 4552);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale_array.data = AI_PTR(g_network_weights_map[0] + 6600);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 6600);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias_array.data = AI_PTR(g_network_weights_map[0] + 6604);
    _ae_encoder_encoder_1_ReduceMean_output_0_Mul_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 6604);
    _ae_encoder_encoder_4_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_4_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 6608);
    _ae_encoder_encoder_4_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 6608);
    _ae_encoder_encoder_4_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_4_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 137680);
    _ae_encoder_encoder_4_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 137680);
    _ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale_array.data = AI_PTR(g_network_weights_map[0] + 138704);
    _ae_encoder_encoder_5_ReduceMean_output_0_Mul_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 138704);
    _ae_encoder_encoder_8_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_8_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 138708);
    _ae_encoder_encoder_8_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 138708);
    _ae_encoder_encoder_8_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_8_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 155092);
    _ae_encoder_encoder_8_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 155092);
    _ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale_array.data = AI_PTR(g_network_weights_map[0] + 155348);
    _ae_encoder_encoder_9_ReduceMean_output_0_Mul_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 155348);
    _ae_encoder_encoder_12_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_12_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 155352);
    _ae_encoder_encoder_12_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 155352);
    _ae_encoder_encoder_12_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_encoder_encoder_12_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 155544);
    _ae_encoder_encoder_12_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 155544);
    _ae_decoder_decoder_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_0_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 155556);
    _ae_decoder_decoder_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 155556);
    _ae_decoder_decoder_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_0_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 155652);
    _ae_decoder_decoder_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 155652);
    _ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale_array.data = AI_PTR(g_network_weights_map[0] + 155780);
    _ae_decoder_decoder_1_ReduceMean_output_0_Mul_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 155780);
    _ae_decoder_decoder_4_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_4_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 155784);
    _ae_decoder_decoder_4_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 155784);
    _ae_decoder_decoder_4_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_4_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 159880);
    _ae_decoder_decoder_4_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 159880);
    _ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale_array.data = AI_PTR(g_network_weights_map[0] + 160392);
    _ae_decoder_decoder_5_ReduceMean_output_0_Mul_scale_array.data_start = AI_PTR(g_network_weights_map[0] + 160392);
    _ae_decoder_decoder_8_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_8_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 160396);
    _ae_decoder_decoder_8_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 160396);
    _ae_decoder_decoder_8_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _ae_decoder_decoder_8_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 193164);
    _ae_decoder_decoder_8_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 193164);
    reconstruction_QuantizeLinear_Input_weights_array.format |= AI_FMT_FLAG_CONST;
    reconstruction_QuantizeLinear_Input_weights_array.data = AI_PTR(g_network_weights_map[0] + 194188);
    reconstruction_QuantizeLinear_Input_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 194188);
    reconstruction_QuantizeLinear_Input_bias_array.format |= AI_FMT_FLAG_CONST;
    reconstruction_QuantizeLinear_Input_bias_array.data = AI_PTR(g_network_weights_map[0] + 195212);
    reconstruction_QuantizeLinear_Input_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 195212);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 218949,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xa49b23b8,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_network_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 218949,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xa49b23b8,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_network_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_network_create(network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_network_data_params_get(&params) != true) {
    err = ai_network_get_error(*network);
    return err;
  }
#if defined(AI_NETWORK_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_NETWORK_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_network_init(*network, &params) != true) {
    err = ai_network_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_network_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_network_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= network_configure_weights(net_ctx, params);
  ok &= network_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

