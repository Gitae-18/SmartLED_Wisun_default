/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-04-15T16:17:49+0900
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
#define AI_NETWORK_MODEL_SIGNATURE     "0x042e9ce9f38d76d8567180d26141f9e4"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2026-04-15T16:17:49+0900"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  input_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 4, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  relu_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 64, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  linear_1_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 8, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  relu_1_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 48, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  recon_QuantizeLinear_Input_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 4, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  relu_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 256, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  relu_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 64, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  linear_1_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  linear_1_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 8, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  relu_1_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 384, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  relu_1_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 48, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  recon_QuantizeLinear_Input_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 192, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  recon_QuantizeLinear_Input_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 4, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  relu_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 324, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  linear_1_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 104, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  relu_1_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 248, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  recon_QuantizeLinear_Input_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 68, AI_STATIC)

/**  Array metadata declarations section  *************************************/
/* Int quant #0 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(input_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.003918086644262075f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #1 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(linear_1_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.008604324422776699f),
    AI_PACK_INTQ_ZP(-15)))

/* Int quant #2 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(linear_1_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 8,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.002537523163482547f, 0.0027636794839054346f, 0.0028752365615218878f, 0.002133322414010763f, 0.00221626041457057f, 0.0025910986587405205f, 0.003497652243822813f, 0.0030060410499572754f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #3 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(recon_QuantizeLinear_Input_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.003799047553911805f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #4 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(recon_QuantizeLinear_Input_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 4,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.002899356884881854f, 0.002232565311715007f, 0.001918849186040461f, 0.002655883552506566f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0)))

/* Int quant #5 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(relu_1_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.006244332995265722f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #6 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(relu_1_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 48,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0029738792218267918f, 0.0027678844053298235f, 0.0027830933686345816f, 0.0029068212024867535f, 0.0019265925511717796f, 0.0025804475881159306f, 0.002473429311066866f, 0.0027576382271945477f, 0.002697539748623967f, 0.0030118345748633146f, 0.0021734125912189484f, 0.0030783882830291986f, 0.0020390572026371956f, 0.0033902123104780912f, 0.002892455318942666f, 0.0028179283253848553f, 0.002724809106439352f, 0.003062531817704439f, 0.0031811692751944065f, 0.0025233342312276363f, 0.001908556092530489f, 0.0024881993886083364f, 0.003087370889261365f, 0.0020747880917042494f, 0.0026432438753545284f, 0.00222311494871974f, 0.0032110463362187147f, 0.0030379456002265215f, 0.0022731423377990723f, 0.002843920374289155f, 0.004079208709299564f, 0.0024770789314061403f, 0.00324414880014956f, 0.002603467321023345f, 0.002807838376611471f, 0.0030561364255845547f, 0.003083001123741269f, 0.003168368013575673f, 0.002549027791246772f, 0.002682385267689824f, 0.002998998621478677f, 0.003377649700269103f, 0.0024279109202325344f, 0.0027297341730445623f, 0.0022754142992198467f, 0.0027593441773205996f, 0.002500778529793024f, 0.0026546455919742584f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #7 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(relu_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00611183000728488f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #8 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(relu_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 64,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.003953810315579176f, 0.002312136348336935f, 0.0036094794049859047f, 0.002532622776925564f, 0.0034441540483385324f, 0.002611696720123291f, 0.002203903626650572f, 0.003888500388711691f, 0.0035394742153584957f, 0.0035939328372478485f, 0.004150448366999626f, 0.002956988522782922f, 0.003897277405485511f, 0.004625424277037382f, 0.0035797846503555775f, 0.002687165979295969f, 0.003419844200834632f, 0.0039036504458636045f, 0.004638508427888155f, 0.0038789762184023857f, 0.0026839047204703093f, 0.0030145335476845503f, 0.0017121530836448073f, 0.0028550131246447563f, 0.004255444742739201f, 0.0028578187339007854f, 0.003340144408866763f, 0.005035774782299995f, 0.003671985352411866f, 0.003464257111772895f, 0.002667833585292101f, 0.0027517531998455524f, 0.003107281867414713f, 0.003333846339955926f, 0.003914018161594868f, 0.003230958478525281f, 0.003864806378260255f, 0.0035864536184817553f, 0.004118391778320074f, 0.00212852843105793f, 0.0040899524465203285f, 0.003535006893798709f, 0.0021782536059617996f, 0.002687975065782666f, 0.003894782392308116f, 0.0026054787449538708f, 0.002687001135200262f, 0.0011732324492186308f, 0.0041190157644450665f, 0.0014755266020074487f, 0.0036261621862649918f, 0.002899379935115576f, 0.0030222474597394466f, 0.004510291386395693f, 0.0019689255859702826f, 0.003517466364428401f, 0.004521293565630913f, 0.003756536403670907f, 0.003636461216956377f, 0.0033731188159435987f, 0.002767956117168069f, 0.002425093436613679f, 0.0028880599420517683f, 0.0031563290394842625f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  input_output, AI_STATIC,
  0, 0x1,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 1, 1, 4, 4),
  1, &input_output_array, &input_output_array_intq)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  linear_1_bias, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &linear_1_bias_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  linear_1_output, AI_STATIC,
  2, 0x1,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 1, 1, 8, 8),
  1, &linear_1_output_array, &linear_1_output_array_intq)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  linear_1_scratch0, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 104, 1, 1), AI_STRIDE_INIT(4, 2, 2, 208, 208),
  1, &linear_1_scratch0_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  linear_1_weights, AI_STATIC,
  4, 0x1,
  AI_SHAPE_INIT(4, 64, 8, 1, 1), AI_STRIDE_INIT(4, 1, 64, 512, 512),
  1, &linear_1_weights_array, &linear_1_weights_array_intq)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  recon_QuantizeLinear_Input_bias, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &recon_QuantizeLinear_Input_bias_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  recon_QuantizeLinear_Input_output, AI_STATIC,
  6, 0x1,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 1, 1, 4, 4),
  1, &recon_QuantizeLinear_Input_output_array, &recon_QuantizeLinear_Input_output_array_intq)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  recon_QuantizeLinear_Input_scratch0, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 68, 1, 1), AI_STRIDE_INIT(4, 2, 2, 136, 136),
  1, &recon_QuantizeLinear_Input_scratch0_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  recon_QuantizeLinear_Input_weights, AI_STATIC,
  8, 0x1,
  AI_SHAPE_INIT(4, 48, 4, 1, 1), AI_STRIDE_INIT(4, 1, 48, 192, 192),
  1, &recon_QuantizeLinear_Input_weights_array, &recon_QuantizeLinear_Input_weights_array_intq)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  relu_1_bias, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &relu_1_bias_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  relu_1_output, AI_STATIC,
  10, 0x1,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 1, 1, 48, 48),
  1, &relu_1_output_array, &relu_1_output_array_intq)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  relu_1_scratch0, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 248, 1, 1), AI_STRIDE_INIT(4, 2, 2, 496, 496),
  1, &relu_1_scratch0_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  relu_1_weights, AI_STATIC,
  12, 0x1,
  AI_SHAPE_INIT(4, 8, 48, 1, 1), AI_STRIDE_INIT(4, 1, 8, 384, 384),
  1, &relu_1_weights_array, &relu_1_weights_array_intq)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  relu_bias, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &relu_bias_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  relu_output, AI_STATIC,
  14, 0x1,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 1, 1, 64, 64),
  1, &relu_output_array, &relu_output_array_intq)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  relu_scratch0, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 324, 1, 1), AI_STRIDE_INIT(4, 2, 2, 648, 648),
  1, &relu_scratch0_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  relu_weights, AI_STATIC,
  16, 0x1,
  AI_SHAPE_INIT(4, 4, 64, 1, 1), AI_STRIDE_INIT(4, 1, 4, 256, 256),
  1, &relu_weights_array, &relu_weights_array_intq)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  recon_QuantizeLinear_Input_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &recon_QuantizeLinear_Input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &recon_QuantizeLinear_Input_weights, &recon_QuantizeLinear_Input_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &recon_QuantizeLinear_Input_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  recon_QuantizeLinear_Input_layer, 20,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &recon_QuantizeLinear_Input_chain,
  NULL, &recon_QuantizeLinear_Input_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  relu_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &linear_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &relu_1_weights, &relu_1_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_1_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  relu_1_layer, 17,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &relu_1_chain,
  NULL, &recon_QuantizeLinear_Input_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  linear_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &linear_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &linear_1_weights, &linear_1_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &linear_1_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  linear_1_layer, 14,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &linear_1_chain,
  NULL, &relu_1_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  relu_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &relu_weights, &relu_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &relu_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  relu_layer, 11,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &relu_chain,
  NULL, &linear_1_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1840, 1, 1),
    1840, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 716, 1, 1),
    716, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &recon_QuantizeLinear_Input_output),
  &relu_layer, 0x3cafa743, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1840, 1, 1),
      1840, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 716, 1, 1),
      716, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &recon_QuantizeLinear_Input_output),
  &relu_layer, 0x3cafa743, NULL)

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
    relu_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 4);
    relu_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 4);
    relu_output_array.data = AI_PTR(g_network_activations_map[0] + 652);
    relu_output_array.data_start = AI_PTR(g_network_activations_map[0] + 652);
    linear_1_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 0);
    linear_1_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    linear_1_output_array.data = AI_PTR(g_network_activations_map[0] + 208);
    linear_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 208);
    relu_1_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 216);
    relu_1_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 216);
    relu_1_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    relu_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    recon_QuantizeLinear_Input_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 48);
    recon_QuantizeLinear_Input_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 48);
    recon_QuantizeLinear_Input_output_array.data = AI_PTR(g_network_activations_map[0] + 184);
    recon_QuantizeLinear_Input_output_array.data_start = AI_PTR(g_network_activations_map[0] + 184);
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
    
    relu_weights_array.format |= AI_FMT_FLAG_CONST;
    relu_weights_array.data = AI_PTR(g_network_weights_map[0] + 0);
    relu_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    relu_bias_array.format |= AI_FMT_FLAG_CONST;
    relu_bias_array.data = AI_PTR(g_network_weights_map[0] + 256);
    relu_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 256);
    linear_1_weights_array.format |= AI_FMT_FLAG_CONST;
    linear_1_weights_array.data = AI_PTR(g_network_weights_map[0] + 512);
    linear_1_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 512);
    linear_1_bias_array.format |= AI_FMT_FLAG_CONST;
    linear_1_bias_array.data = AI_PTR(g_network_weights_map[0] + 1024);
    linear_1_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 1024);
    relu_1_weights_array.format |= AI_FMT_FLAG_CONST;
    relu_1_weights_array.data = AI_PTR(g_network_weights_map[0] + 1056);
    relu_1_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 1056);
    relu_1_bias_array.format |= AI_FMT_FLAG_CONST;
    relu_1_bias_array.data = AI_PTR(g_network_weights_map[0] + 1440);
    relu_1_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 1440);
    recon_QuantizeLinear_Input_weights_array.format |= AI_FMT_FLAG_CONST;
    recon_QuantizeLinear_Input_weights_array.data = AI_PTR(g_network_weights_map[0] + 1632);
    recon_QuantizeLinear_Input_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 1632);
    recon_QuantizeLinear_Input_bias_array.format |= AI_FMT_FLAG_CONST;
    recon_QuantizeLinear_Input_bias_array.data = AI_PTR(g_network_weights_map[0] + 1824);
    recon_QuantizeLinear_Input_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 1824);
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
      
      .n_macc            = 1468,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x3cafa743,
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
      
      .n_macc            = 1468,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x3cafa743,
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

