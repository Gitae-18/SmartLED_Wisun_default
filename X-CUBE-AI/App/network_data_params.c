/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-04-15T16:17:49+0900
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "network_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_network_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_network_weights_array_u64[230] = {
  0x7f972cd17fe9756eU, 0x46a397f267f9c7aU, 0x7fb2eaa838bf227fU, 0x9374810c81b1b082U,
  0xd7217f256fde2b7fU, 0x3f71c67f7f3ceb32U, 0x33327f9bdc81c65fU, 0xc039810ca47f093dU,
  0x437fc0c6a77fe238U, 0x7f3849b9ac88b77fU, 0x866ee47fb56381a0U, 0xa3817768dc637f8eU,
  0x81e92c9c9fd6df7fU, 0x267fb19eae819a04U, 0xaac28117d3a7812aU, 0x81c29c9a81f3ba01U,
  0xc7d768811c7fcf7cU, 0x8e73813ce4258196U, 0xc423917f7f482100U, 0x814c4d78b27fb2fdU,
  0x7fc0ed4f94102b7fU, 0xcf96d57ff58c8133U, 0xd7357f46817e3db2U, 0x81b52e8ff858747fU,
  0x7f760c37f6077f66U, 0x81969e14527edc81U, 0x7ffad3a75e7f58daU, 0xf4ce7fab816bc801U,
  0x6e9281c127636681U, 0x8c819b1934597f49U, 0x658781f9abf9a57fU, 0x81d0c40a81c86bcdU,
  0xffff69e7000048ebU, 0x59070000419fU, 0xffff6d9800006b52U, 0xffffc3650000dd0bU,
  0x8f9a00003c20U, 0x91efffff394U, 0xffffbaba00000348U, 0xffff47d800004e4cU,
  0x66b300005d08U, 0xffffebfb00007f7dU, 0x3199ffff6765U, 0xffff52380000b7d6U,
  0xffff9770fffffe7bU, 0xffffeab4fffff2f2U, 0xffffd252ffffc882U, 0xaf0cffff4e6fU,
  0xffffe88b00007ed7U, 0xffffc108fffff17fU, 0xffff78beffff8efcU, 0xffff7a3a00003797U,
  0xfffff3a4ffffc0cbU, 0xffffe309ffffb3caU, 0xffffe6560000205fU, 0xffff3490ffffcfb8U,
  0x133c1ffffb113U, 0x526affffd211U, 0x1461ffffdc79U, 0x499a0000eb0fU,
  0x547effffb0b9U, 0x482500008958U, 0xd4e9000093ddU, 0x4a16ffffe58dU,
  0xcff7d8d80d2ef6cdU, 0xdb0981cd0e21d8d2U, 0xd4c5fbfce1e711f3U, 0x360e0cdf6730e6caU,
  0x30fdfb1104cf1b22U, 0xdbf0dbdcebd4fed0U, 0x98d405cc201bf58dU, 0x30120654d61540bcU,
  0xedd1f629fbe72911U, 0xeb1509c0f8222031U, 0xeadcddf513e228f8U, 0xc520102b91f30eceU,
  0x18d4ef18e2e211f9U, 0xfa281f81c6f82a23U, 0x5f815fff8d62524U, 0x1dfee34ff0bf9f3U,
  0x9e608e9e810df2eU, 0xe129eddcf9133900U, 0x1a394c2af1b6fa2aU, 0xe4f2e3e9cae8faffU,
  0xf646e4ec06e6ef28U, 0x1121a7ff80d281bU, 0x241ed704c6e9382dU, 0xb204f921c1f0ef13U,
  0xfe62c93ac9dcd30bU, 0xb08815e25f113fbU, 0xfefe4af9c34aee02U, 0x3ef3fcdcccd803fdU,
  0x20d7ec1d2d220631U, 0x3a0fef9407f33d1aU, 0x97c1a4b80bb62933U, 0x2c31291103ceeb85U,
  0x3204cf18ded91eeeU, 0x310c4ae1f2de0f02U, 0x7339efbe4ca08cbU, 0xf8dcd0f53fcb1dc1U,
  0x3fa31e6de243626U, 0xe0c7f1f411edc4e4U, 0x12d44ad30030ddcdU, 0xdecfd081d2f04647U,
  0xf632e9c10e16d3f8U, 0x26fc083fce0edee8U, 0x19c5fddc1218d2ebU, 0x38f9da1b81150229U,
  0xfbc4f6d625dd191fU, 0xf1c21c361de4e527U, 0x2601fdf352e8ddbbU, 0xfae9d20850d54d20U,
  0x10e10af0171be31aU, 0xf6f7fcfb2e36e527U, 0xe21309f331f31ef8U, 0x11314e19918edcaU,
  0x20f906361f12e6f0U, 0xc1c2881dc1919e5U, 0x26e53712ce2a0b1aU, 0xc30920d8cc03d9fdU,
  0xf8d2eeee1e1a29eeU, 0x101e08e3060d12ebU, 0x1c0041eb14ece4fcU, 0x2e4f9e9cb1bddf2U,
  0x122ae0f618db1103U, 0xf7282b7feeecdc11U, 0xbf81422f2f72a3bU, 0xd319e720bc25dd05U,
  0xfffff44fffffdb3dU, 0xae0fffff579U, 0xfffff651ffffe76aU, 0xfffff85a00000923U,
  0x408c7d889be8114U, 0x50bc2681d73e6e48U, 0xca789e34477f333fU, 0x7f078bfe2b6e75c5U,
  0xd879117fb6bb1733U, 0xdd697f565616393cU, 0x3a7ef6e3db132c81U, 0x1b7f325fdafadac3U,
  0x7fb59eeeec993320U, 0xfade2ef4c57f3347U, 0x7fe8ad7e07bf3285U, 0xdee7b4afa37ff8c6U,
  0xe1bfbd8381364b8eU, 0xdec526a6acba2e81U, 0x137fc09c97535662U, 0xe1386bfcb69a1881U,
  0xfb7fbf12373fec08U, 0x6855c7177f6863ceU, 0x42f0e91a45be0381U, 0x81ff03ebb07587ebU,
  0xc4ed8e8156b1d954U, 0x3f1b07b5810ce3c3U, 0xd9a0cc83305e3181U, 0xc6758106bb570a57U,
  0xf102b9417f008cdeU, 0x2b15c1cf0fea7881U, 0x117dbba9027fd61bU, 0x7f5650bf8eecdbc8U,
  0x5b697f7766994e97U, 0xf581e5c14dec4ae7U, 0x2c68331b81c9f3b2U, 0xfe7b3783631e81c3U,
  0x7fa844ac4c3e5c9fU, 0x81b838df38144fbeU, 0x316352c642e97fc6U, 0x783e8e7f81149f9eU,
  0xd17f8de86c873934U, 0xa334e412afba7fefU, 0x6e8b1b4a81bcc204U, 0x78995a7f14b31828U,
  0x1a5759f7f473a05U, 0x417ffbf0e906fdb1U, 0x8b811e0cf9713a51U, 0x7fd4491dbb1accf7U,
  0xdcf2c3ac81898a79U, 0xfd33a2ce7fd5dbb6U, 0x7db87663917f3878U, 0xb0172481da3cf465U,
  0xffffd20000001bcaU, 0x381c00002f38U, 0xffffd8f9000010d9U, 0x114c000021c4U,
  0x1097000016b0U, 0xffffed6a00004335U, 0xe20fffff0b7U, 0x2a25000008faU,
  0xabffffe48aU, 0xfffff2ba000006d8U, 0x23200003f5cU, 0xfffffbe9ffffda47U,
  0x2ccffffe475U, 0xffffc798ffffff07U, 0x449fffff075U, 0x378efffffa88U,
  0x2a79fffff6abU, 0xffffde4d0000105cU, 0x2b3d00000aefU, 0xffffd952ffffd6d0U,
  0x274b0000324aU, 0x27d6ffffdb1eU, 0xffffc83dfffff650U, 0x1406000005abU,
  0xe141b3131e395f0U, 0xd8322ce9f002f8eeU, 0x3652ef3d1c3b2d30U, 0x19bd17e54f1dedd1U,
  0x2e10071e81372b33U, 0x11d005f3e72dff21U, 0x512dc3dc16115fd2U, 0x2be1295018df23c2U,
  0x2243f4e5f3561530U, 0xe735d3ce7ff3ee05U, 0xed0938d75a39f17eU, 0xcccded2117c21cd7U,
  0x2e2cb7e95a137f4cU, 0xba55aa4d4d4b1a59U, 0x23c4daf279c71553U, 0xccebf9099247fa3bU,
  0xf7f2e4c041e5a9b2U, 0x807b70532f514dbU, 0xfc102951210a9adaU, 0x2d48e6fbfc04dc17U,
  0x2fa0332212dc4730U, 0x1e61ccd2a80812e1U, 0xd80a47462215e581U, 0xf2141fd0d62b230eU,
  0xffffed3cfffff7fdU, 0xfffffcd200002cebU,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

