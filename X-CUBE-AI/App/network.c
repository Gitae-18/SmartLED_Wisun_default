/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-12-12T16:40:10+0900
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define AI_NETWORK_MODEL_SIGNATURE     "0x37e92305b90701293d7e8fa8053cf722"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2025-12-12T16:40:10+0900"

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
  NULL, NULL, 130, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 65, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 16, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 32, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_output_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 65, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  output_QuantizeLinear_Input_output_array, AI_ARRAY_FORMAT_S8|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 130, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 8450, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 65, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 2080, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 32, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 16, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 512, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 32, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 2080, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 65, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  output_QuantizeLinear_Input_weights_array, AI_ARRAY_FORMAT_S8,
  NULL, NULL, 8450, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  output_QuantizeLinear_Input_bias_array, AI_ARRAY_FORMAT_S32,
  NULL, NULL, 130, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 455, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 225, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 112, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 176, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 357, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  output_QuantizeLinear_Input_scratch0_array, AI_ARRAY_FORMAT_S16,
  NULL, NULL, 715, AI_STATIC)

/**  Array metadata declarations section  *************************************/
/* Int quant #0 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_decoder_decoder_1_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00929982215166092f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #1 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_decoder_decoder_1_Relu_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 32,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.0029152848292142153f, 0.004925925750285387f, 0.004222851246595383f, 0.005745139438658953f, 0.004608094692230225f, 0.005707689560949802f, 0.006585709750652313f, 0.004541704896837473f, 0.0026064319536089897f, 0.005882274359464645f, 0.00431361049413681f, 0.0036515151150524616f, 0.0036881817504763603f, 0.004539658315479755f, 0.005712149199098349f, 0.005169140174984932f, 0.008827095851302147f, 0.006914306897670031f, 0.006027963012456894f, 0.006479448638856411f, 0.005263072904199362f, 0.0061768353916704655f, 0.005766794551163912f, 0.006616624072194099f, 0.0034985672682523727f, 0.003780967090278864f, 0.0036000972613692284f, 0.005196689162403345f, 0.006592034362256527f, 0.006464107893407345f, 0.0027579995803534985f, 0.014762555249035358f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #2 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_decoder_decoder_3_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.00560704106464982f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #3 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_decoder_decoder_3_Relu_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 65,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.005798786878585815f, 0.004039610270410776f, 0.0036610455717891455f, 0.008029546588659286f, 0.0028029524255543947f, 0.002793276682496071f, 0.004489615559577942f, 0.004233579151332378f, 0.003195199416950345f, 0.004010891076177359f, 0.005641045514494181f, 0.0034299008548259735f, 0.006950247101485729f, 0.005749564152210951f, 0.005639827810227871f, 0.007359284441918135f, 0.0027551930397748947f, 0.0048071518540382385f, 0.002632670570164919f, 0.0044864630326628685f, 0.004186372272670269f, 0.003950620535761118f, 0.0026442776434123516f, 0.007723614107817411f, 0.007021840661764145f, 0.005617619026452303f, 0.004486040212213993f, 0.0036770033184438944f, 0.0035698548890650272f, 0.0030148352961987257f, 0.005606633611023426f, 0.005769788753241301f, 0.010312322527170181f, 0.0026668368373066187f, 0.00699260039255023f, 0.0026746627409011126f, 0.0061876303516328335f, 0.004422860220074654f, 0.0032602790743112564f, 0.005984820891171694f, 0.002658304525539279f, 0.005539975129067898f, 0.0037717153318226337f, 0.0026281145401299f, 0.0045990790240466595f, 0.013086930848658085f, 0.0032284811604768038f, 0.004266729112714529f, 0.005168731790035963f, 0.008239582180976868f, 0.006740942131727934f, 0.01022461149841547f, 0.0026758010499179363f, 0.0027510281652212143f, 0.005013149231672287f, 0.006833042483776808f, 0.005292635411024094f, 0.004393341485410929f, 0.003602684009820223f, 0.002692506881430745f, 0.005001824349164963f, 0.0027036978863179684f, 0.0055539654567837715f, 0.0034610158763825893f, 0.002709250431507826f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #4 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_encoder_encoder_1_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.016091158613562584f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #5 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_encoder_encoder_1_Relu_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 65,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.013128187507390976f, 0.01093373540788889f, 0.005726522766053677f, 0.0014544917503371835f, 0.0014010060112923384f, 0.004483744036406279f, 0.006973029114305973f, 0.001404439564794302f, 0.003550776047632098f, 0.0036029797047376633f, 0.0037711807526648045f, 0.0014483482809737325f, 0.0014106293674558401f, 0.009685707278549671f, 0.006107257679104805f, 0.0033951939549297094f, 0.0015624954830855131f, 0.003266620682552457f, 0.005697616375982761f, 0.005505934823304415f, 0.004104956518858671f, 0.002789160003885627f, 0.001398703665472567f, 0.0058169057592749596f, 0.009333774447441101f, 0.0014184144092723727f, 0.004643542692065239f, 0.0045000165700912476f, 0.015965620055794716f, 0.006530670914798975f, 0.013007539324462414f, 0.01633366197347641f, 0.010271270759403706f, 0.0013561027590185404f, 0.007686899043619633f, 0.0030940542928874493f, 0.006113817449659109f, 0.004535680636763573f, 0.0016587686259299517f, 0.006348588038235903f, 0.011025105603039265f, 0.004661952145397663f, 0.012781927362084389f, 0.0044386000372469425f, 0.005126930307596922f, 0.009621063247323036f, 0.0015740921953693032f, 0.0013841110048815608f, 0.0015058965655043721f, 0.003981214482337236f, 0.0037417346611618996f, 0.008953148499131203f, 0.003415279556065798f, 0.0029378815088421106f, 0.005864390637725592f, 0.003141892608255148f, 0.0074452608823776245f, 0.005094241816550493f, 0.0015040099387988448f, 0.006978441495448351f, 0.004229083191603422f, 0.005467690993100405f, 0.0015279577346518636f, 0.00370240630581975f, 0.004131146240979433f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #6 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_encoder_encoder_3_Relu_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.023226754739880562f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #7 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_encoder_encoder_3_Relu_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 32,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.006486984435468912f, 0.0044038062915205956f, 0.0021372719202190638f, 0.007002241909503937f, 0.001900293049402535f, 0.00664286594837904f, 0.004576310981065035f, 0.002133506117388606f, 0.002062651328742504f, 0.0046117277815938f, 0.006977862678468227f, 0.0023292959667742252f, 0.007242281921207905f, 0.0063964249566197395f, 0.003816033946350217f, 0.009447457268834114f, 0.004547486547380686f, 0.001959574641659856f, 0.0020801303908228874f, 0.005571229383349419f, 0.005210985895246267f, 0.0025125015527009964f, 0.006206789053976536f, 0.005947759374976158f, 0.0019327719928696752f, 0.00535493902862072f, 0.005309486761689186f, 0.00522509403526783f, 0.0020605376921594143f, 0.00570217240601778f, 0.004896451719105244f, 0.0018554903799667954f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #8 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_encoder_encoder_4_Gemm_output_0_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.022141002118587494f),
    AI_PACK_INTQ_ZP(58)))

/* Int quant #9 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(_net_encoder_encoder_4_Gemm_output_0_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 16,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.005251425318419933f, 0.003635141998529434f, 0.005315143149346113f, 0.005170060321688652f, 0.0047020683996379375f, 0.00377219938673079f, 0.00516425259411335f, 0.004596533719450235f, 0.00457153283059597f, 0.004717567469924688f, 0.0036014236975461245f, 0.006354379002004862f, 0.005195132922381163f, 0.006280624307692051f, 0.004530660342425108f, 0.004010578617453575f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/* Int quant #10 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(input_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.003921316470950842f),
    AI_PACK_INTQ_ZP(-128)))

/* Int quant #11 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(output_QuantizeLinear_Input_output_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 1,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.004475232679396868f),
    AI_PACK_INTQ_ZP(-108)))

/* Int quant #12 */
AI_INTQ_INFO_LIST_OBJ_DECLARE(output_QuantizeLinear_Input_weights_array_intq, AI_STATIC_CONST,
  AI_BUFFER_META_FLAG_SCALE_FLOAT|AI_BUFFER_META_FLAG_ZEROPOINT_S8, 130,
  AI_PACK_INTQ_INFO(
    AI_PACK_INTQ_SCALE(0.008321817964315414f, 0.005568535998463631f, 0.007530761882662773f, 0.005861559882760048f, 0.0067836325615644455f, 0.005446417722851038f, 0.0028477187734097242f, 0.006576285697519779f, 0.0062752049416303635f, 0.005462857894599438f, 0.0037797652184963226f, 0.0037435570266097784f, 0.0035288904327899218f, 0.0033642943017184734f, 0.0037673539482057095f, 0.0034670112654566765f, 0.0050184279680252075f, 0.01734152063727379f, 0.00298437662422657f, 0.0030045383609831333f, 0.005043854005634785f, 0.002750575076788664f, 0.0033771360758692026f, 0.0027445273008197546f, 0.0036569819785654545f, 0.0035648909397423267f, 0.0035373594146221876f, 0.005959371570497751f, 0.004087795969098806f, 0.008075346238911152f, 0.0037081511691212654f, 0.0034049642272293568f, 0.003425539005547762f, 0.0032772605773061514f, 0.0029283040203154087f, 0.005932173691689968f, 0.0024824843276292086f, 0.002777590649202466f, 0.003720797598361969f, 0.0036441574338823557f, 0.005715245380997658f, 0.0067956717684865f, 0.0029454869218170643f, 0.002744894241914153f, 0.0036471833009272814f, 0.003089227480813861f, 0.003554258495569229f, 0.004643057938665152f, 0.005960313603281975f, 0.0038074692711234093f, 0.002332742093130946f, 0.0032154847867786884f, 0.0024833506904542446f, 0.0019379624864086509f, 0.002236364409327507f, 0.008307872340083122f, 0.0043375990353524685f, 0.0038063267711549997f, 0.004345662891864777f, 0.0043648043647408485f, 0.004502217285335064f, 0.004117061849683523f, 0.004183928947895765f, 0.004408920183777809f, 0.00435881270095706f, 0.005709255579859018f, 0.0027912850491702557f, 0.0030328594148159027f, 0.004124180879443884f, 0.002823268063366413f, 0.0030648496467620134f, 0.003699551336467266f, 0.004870768636465073f, 0.002790179569274187f, 0.010568572208285332f, 0.0025179919321089983f, 0.00296636950224638f, 0.002144219819456339f, 0.0033735090401023626f, 0.00867387093603611f, 0.00428417744114995f, 0.005306888837367296f, 0.0035794973373413086f, 0.003347100457176566f, 0.004181241616606712f, 0.004271667450666428f, 0.0036156794521957636f, 0.0062308283522725105f, 0.0032852578442543745f, 0.00349095044657588f, 0.004760153591632843f, 0.005360592622309923f, 0.005287886597216129f, 0.002782700816169381f, 0.004274234175682068f, 0.0033566977363079786f, 0.005639966111630201f, 0.008877621032297611f, 0.003302144818007946f, 0.005358266178518534f, 0.0027144865598529577f, 0.004291686695069075f, 0.004342610482126474f, 0.007091744802892208f, 0.008339007385075092f, 0.005538330413401127f, 0.0025438149459660053f, 0.004990322049707174f, 0.002687084721401334f, 0.003263709833845496f, 0.005675999913364649f, 0.005860622972249985f, 0.0032657606061547995f, 0.003674912266433239f, 0.003215261735022068f, 0.011876546777784824f, 0.004080051090568304f, 0.003631547326222062f, 0.004051888827234507f, 0.0027020741254091263f, 0.0076381489634513855f, 0.006333089433610439f, 0.006615829188376665f, 0.0034272228367626667f, 0.00311308354139328f, 0.002943819621577859f, 0.0034834900870919228f, 0.0045459214597940445f, 0.014703470282256603f, 0.0198560431599617f),
    AI_PACK_INTQ_ZP(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)))

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_bias, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_net_decoder_decoder_1_Relu_output_0_bias_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_output, AI_STATIC,
  1, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &_net_decoder_decoder_1_Relu_output_0_output_array, &_net_decoder_decoder_1_Relu_output_0_output_array_intq)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_scratch0, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 176, 1, 1), AI_STRIDE_INIT(4, 2, 2, 352, 352),
  1, &_net_decoder_decoder_1_Relu_output_0_scratch0_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_weights, AI_STATIC,
  3, 0x1,
  AI_SHAPE_INIT(4, 16, 32, 1, 1), AI_STRIDE_INIT(4, 1, 16, 512, 512),
  1, &_net_decoder_decoder_1_Relu_output_0_weights_array, &_net_decoder_decoder_1_Relu_output_0_weights_array_intq)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_bias, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 65, 1, 1), AI_STRIDE_INIT(4, 4, 4, 260, 260),
  1, &_net_decoder_decoder_3_Relu_output_0_bias_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_output, AI_STATIC,
  5, 0x1,
  AI_SHAPE_INIT(4, 1, 65, 1, 1), AI_STRIDE_INIT(4, 1, 1, 65, 65),
  1, &_net_decoder_decoder_3_Relu_output_0_output_array, &_net_decoder_decoder_3_Relu_output_0_output_array_intq)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_scratch0, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 357, 1, 1), AI_STRIDE_INIT(4, 2, 2, 714, 714),
  1, &_net_decoder_decoder_3_Relu_output_0_scratch0_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_weights, AI_STATIC,
  7, 0x1,
  AI_SHAPE_INIT(4, 32, 65, 1, 1), AI_STRIDE_INIT(4, 1, 32, 2080, 2080),
  1, &_net_decoder_decoder_3_Relu_output_0_weights_array, &_net_decoder_decoder_3_Relu_output_0_weights_array_intq)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_bias, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 65, 1, 1), AI_STRIDE_INIT(4, 4, 4, 260, 260),
  1, &_net_encoder_encoder_1_Relu_output_0_bias_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_output, AI_STATIC,
  9, 0x1,
  AI_SHAPE_INIT(4, 1, 65, 1, 1), AI_STRIDE_INIT(4, 1, 1, 65, 65),
  1, &_net_encoder_encoder_1_Relu_output_0_output_array, &_net_encoder_encoder_1_Relu_output_0_output_array_intq)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_scratch0, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 455, 1, 1), AI_STRIDE_INIT(4, 2, 2, 910, 910),
  1, &_net_encoder_encoder_1_Relu_output_0_scratch0_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_weights, AI_STATIC,
  11, 0x1,
  AI_SHAPE_INIT(4, 130, 65, 1, 1), AI_STRIDE_INIT(4, 1, 130, 8450, 8450),
  1, &_net_encoder_encoder_1_Relu_output_0_weights_array, &_net_encoder_encoder_1_Relu_output_0_weights_array_intq)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_bias, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 4, 4, 128, 128),
  1, &_net_encoder_encoder_3_Relu_output_0_bias_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_output, AI_STATIC,
  13, 0x1,
  AI_SHAPE_INIT(4, 1, 32, 1, 1), AI_STRIDE_INIT(4, 1, 1, 32, 32),
  1, &_net_encoder_encoder_3_Relu_output_0_output_array, &_net_encoder_encoder_3_Relu_output_0_output_array_intq)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_scratch0, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 225, 1, 1), AI_STRIDE_INIT(4, 2, 2, 450, 450),
  1, &_net_encoder_encoder_3_Relu_output_0_scratch0_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_weights, AI_STATIC,
  15, 0x1,
  AI_SHAPE_INIT(4, 65, 32, 1, 1), AI_STRIDE_INIT(4, 1, 65, 2080, 2080),
  1, &_net_encoder_encoder_3_Relu_output_0_weights_array, &_net_encoder_encoder_3_Relu_output_0_weights_array_intq)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_bias, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &_net_encoder_encoder_4_Gemm_output_0_bias_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_output, AI_STATIC,
  17, 0x1,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 1, 1, 16, 16),
  1, &_net_encoder_encoder_4_Gemm_output_0_output_array, &_net_encoder_encoder_4_Gemm_output_0_output_array_intq)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_scratch0, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 112, 1, 1), AI_STRIDE_INIT(4, 2, 2, 224, 224),
  1, &_net_encoder_encoder_4_Gemm_output_0_scratch0_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_weights, AI_STATIC,
  19, 0x1,
  AI_SHAPE_INIT(4, 32, 16, 1, 1), AI_STRIDE_INIT(4, 1, 32, 512, 512),
  1, &_net_encoder_encoder_4_Gemm_output_0_weights_array, &_net_encoder_encoder_4_Gemm_output_0_weights_array_intq)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  input_output, AI_STATIC,
  20, 0x1,
  AI_SHAPE_INIT(4, 1, 130, 1, 1), AI_STRIDE_INIT(4, 1, 1, 130, 130),
  1, &input_output_array, &input_output_array_intq)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  output_QuantizeLinear_Input_bias, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 130, 1, 1), AI_STRIDE_INIT(4, 4, 4, 520, 520),
  1, &output_QuantizeLinear_Input_bias_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  output_QuantizeLinear_Input_output, AI_STATIC,
  22, 0x1,
  AI_SHAPE_INIT(4, 1, 130, 1, 1), AI_STRIDE_INIT(4, 1, 1, 130, 130),
  1, &output_QuantizeLinear_Input_output_array, &output_QuantizeLinear_Input_output_array_intq)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  output_QuantizeLinear_Input_scratch0, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 715, 1, 1), AI_STRIDE_INIT(4, 2, 2, 1430, 1430),
  1, &output_QuantizeLinear_Input_scratch0_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  output_QuantizeLinear_Input_weights, AI_STATIC,
  24, 0x1,
  AI_SHAPE_INIT(4, 65, 130, 1, 1), AI_STRIDE_INIT(4, 1, 65, 8450, 8450),
  1, &output_QuantizeLinear_Input_weights_array, &output_QuantizeLinear_Input_weights_array_intq)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  output_QuantizeLinear_Input_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_decoder_decoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &output_QuantizeLinear_Input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &output_QuantizeLinear_Input_weights, &output_QuantizeLinear_Input_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &output_QuantizeLinear_Input_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  output_QuantizeLinear_Input_layer, 30,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &output_QuantizeLinear_Input_chain,
  NULL, &output_QuantizeLinear_Input_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_decoder_decoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_decoder_decoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_decoder_decoder_3_Relu_output_0_weights, &_net_decoder_decoder_3_Relu_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_decoder_decoder_3_Relu_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_decoder_decoder_3_Relu_output_0_layer, 27,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_net_decoder_decoder_3_Relu_output_0_chain,
  NULL, &output_QuantizeLinear_Input_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_decoder_decoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_decoder_decoder_1_Relu_output_0_weights, &_net_decoder_decoder_1_Relu_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_decoder_decoder_1_Relu_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_decoder_decoder_1_Relu_output_0_layer, 24,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_net_decoder_decoder_1_Relu_output_0_chain,
  NULL, &_net_decoder_decoder_3_Relu_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_encoder_encoder_4_Gemm_output_0_weights, &_net_encoder_encoder_4_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_4_Gemm_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_encoder_encoder_4_Gemm_output_0_layer, 21,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_net_encoder_encoder_4_Gemm_output_0_chain,
  NULL, &_net_decoder_decoder_1_Relu_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_3_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_encoder_encoder_3_Relu_output_0_weights, &_net_encoder_encoder_3_Relu_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_3_Relu_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_encoder_encoder_3_Relu_output_0_layer, 18,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_net_encoder_encoder_3_Relu_output_0_chain,
  NULL, &_net_encoder_encoder_4_Gemm_output_0_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_1_Relu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_net_encoder_encoder_1_Relu_output_0_weights, &_net_encoder_encoder_1_Relu_output_0_bias),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_net_encoder_encoder_1_Relu_output_0_scratch0)
)

AI_LAYER_OBJ_DECLARE(
  _net_encoder_encoder_1_Relu_output_0_layer, 15,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense_integer_SSSA_ch,
  &_net_encoder_encoder_1_Relu_output_0_chain,
  NULL, &_net_encoder_encoder_3_Relu_output_0_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 23448, 1, 1),
    23448, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1632, 1, 1),
    1632, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &output_QuantizeLinear_Input_output),
  &_net_encoder_encoder_1_Relu_output_0_layer, 0xa7d34535, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 23448, 1, 1),
      23448, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1632, 1, 1),
      1632, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &output_QuantizeLinear_Input_output),
  &_net_encoder_encoder_1_Relu_output_0_layer, 0xa7d34535, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    input_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    input_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _net_encoder_encoder_1_Relu_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 652);
    _net_encoder_encoder_1_Relu_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 652);
    _net_encoder_encoder_1_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1564);
    _net_encoder_encoder_1_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1564);
    _net_encoder_encoder_3_Relu_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _net_encoder_encoder_3_Relu_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _net_encoder_encoder_3_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 972);
    _net_encoder_encoder_3_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 972);
    _net_encoder_encoder_4_Gemm_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _net_encoder_encoder_4_Gemm_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _net_encoder_encoder_4_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 744);
    _net_encoder_encoder_4_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 744);
    _net_decoder_decoder_1_Relu_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 760);
    _net_decoder_decoder_1_Relu_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 760);
    _net_decoder_decoder_1_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 520);
    _net_decoder_decoder_1_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 520);
    _net_decoder_decoder_3_Relu_output_0_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 552);
    _net_decoder_decoder_3_Relu_output_0_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 552);
    _net_decoder_decoder_3_Relu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 1564);
    _net_decoder_decoder_3_Relu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 1564);
    output_QuantizeLinear_Input_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 132);
    output_QuantizeLinear_Input_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 132);
    output_QuantizeLinear_Input_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    output_QuantizeLinear_Input_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
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
    
    _net_encoder_encoder_1_Relu_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_encoder_encoder_1_Relu_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 0);
    _net_encoder_encoder_1_Relu_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    _net_encoder_encoder_1_Relu_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_encoder_encoder_1_Relu_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 8452);
    _net_encoder_encoder_1_Relu_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 8452);
    _net_encoder_encoder_3_Relu_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_encoder_encoder_3_Relu_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 8712);
    _net_encoder_encoder_3_Relu_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 8712);
    _net_encoder_encoder_3_Relu_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_encoder_encoder_3_Relu_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 10792);
    _net_encoder_encoder_3_Relu_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 10792);
    _net_encoder_encoder_4_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_encoder_encoder_4_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 10920);
    _net_encoder_encoder_4_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 10920);
    _net_encoder_encoder_4_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_encoder_encoder_4_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 11432);
    _net_encoder_encoder_4_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 11432);
    _net_decoder_decoder_1_Relu_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_decoder_decoder_1_Relu_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 11496);
    _net_decoder_decoder_1_Relu_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 11496);
    _net_decoder_decoder_1_Relu_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_decoder_decoder_1_Relu_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 12008);
    _net_decoder_decoder_1_Relu_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 12008);
    _net_decoder_decoder_3_Relu_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _net_decoder_decoder_3_Relu_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 12136);
    _net_decoder_decoder_3_Relu_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 12136);
    _net_decoder_decoder_3_Relu_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _net_decoder_decoder_3_Relu_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 14216);
    _net_decoder_decoder_3_Relu_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 14216);
    output_QuantizeLinear_Input_weights_array.format |= AI_FMT_FLAG_CONST;
    output_QuantizeLinear_Input_weights_array.data = AI_PTR(g_network_weights_map[0] + 14476);
    output_QuantizeLinear_Input_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 14476);
    output_QuantizeLinear_Input_bias_array.format |= AI_FMT_FLAG_CONST;
    output_QuantizeLinear_Input_bias_array.data = AI_PTR(g_network_weights_map[0] + 22928);
    output_QuantizeLinear_Input_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 22928);
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
      
      .n_macc            = 22424,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xa7d34535,
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
      
      .n_macc            = 22424,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xa7d34535,
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

