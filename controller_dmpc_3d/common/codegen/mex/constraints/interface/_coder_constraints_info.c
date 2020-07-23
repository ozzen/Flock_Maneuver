/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_constraints_info.c
 *
 * Code generation for function '_coder_constraints_info'
 *
 */

/* Include files */
#include "_coder_constraints_info.h"
#include "emlrt.h"
#include "rt_nonfinite.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void);

/* Function Definitions */
static const mxArray *emlrtMexFcnResolvedFunctionsInfo(void)
{
  const mxArray *nameCaptureInfo;
  const char * data[38] = {
    "789ced5d4d70e3c875e6c6bbf61cb2195fbcbb951f4f268eabec9a8a484a1445ddc27f5212ffc41f91da1faa09802444000d012045eae0b09c2d67533ec427a7"
    "9c1c73482a97f816e792e8e863bc76d56637d99c73cac59563522108b686a2880147dd68896077d58ca6e789fdbdf71afde1f1e175c3f74636f786cfe7fbadc9",
    "9f7ffa139fef3b1f9b3d9fefed59ffeb3eabfd86ef765b94bf31fb999ceb9b72d4def2bd39edbfbdf0fb7f31fbc941c5108686d551802cdc7c9287b2a800c5a8"
    "8c54c1a7093a9406023f95b44549a888b2509eefe4cd9e9c9a13dd744c91f9ef7857e07ae5beecd3bafa4b0da5f9ce8d3f7e6e63ef9b2bfa83b7f1c7d7677d24",
    "7f3ff9a1bfaa0b9aeed7203482fe84a0f70ca8fa5312e47aa2d269ca4011fa03f317dae67fddf4fd3b09bfe93c0d4a92a0357959e59a3bfce4bf64192aa64437"
    "34202a86be25cfdb758669d75bb676bd35edf7b701c711c4fbaa2dde57a7fd89957dce7889778d89f7912d9ed5477297e66dea3e73c69cfcf774457be67fcedb",
    "f3b6efc9b4fff7ef7d363589161e6a9b8237b4196fd5ebf11d1b3c743d22b95a3bcf14b861e8cae8c0836c35ac56d55e3af6528fa2038e931ebeb9fe323bdd1e"
    "ffdae6f3ebbaaec798f6fcbe833d48ce415ed0b626f701415380b435b98f76812a94c52beb6ea8e3f2f462b3d3073552f379ea8087e4ef678fea93298daaaa24",
    "72c01027f7457f2e5a398ac69ac7db81ed00d802aaea3720945a70e8176469fae7c5d46dfe17c86ffe3b7eb3eeaaf478e5c9c7e90f68e2a1b62978b4787aefd4"
    "183506bd8eda87a9525edf09727a4c8b339e7eaceb1af7ba78cfc11e245fe06905681d51e1babd87e267dcef3fc0010fc95f6f1e2793a509e64dd59c45eb6f49",
    "6cdd9dd61bf7a1e9a4c62bef347fc078da453c5a3cdd899462fbed60241fcdd45aa7f9f405d8dbbef2503cbd69eb9b547e6312f4438d209e537e8387fd9624bc"
    "c4fb2926de812d9ed547f2d7bfbf9a13250343022dbf20b5fb8a7fea2ada71f297bff913c6bf2ee2d1e2dfab5c7010e7b86e5aaa0ab5a41c0815c36a38ed1dfe",
    "5df7753cc6d4ff9983fe48be10178b7aac2f4a4656c9f7654113b907e361dcfb67d7c17e2477e7feb9e8c62d99169ffcf93fef317e76118f1a3f0754f1504f72"
    "8544514c75772b91686bb79064fcec157efe1d07fd917c819f750e4840db4251327e9cbcd856cd5ffc0cd3fe9a031e9213ca43bdb01cf7722e69f1c7672c5e76",
    "158f161fd70a1789be163b4a4495f04523220ce285f6c843f98a75e7e31f61eaff5d07fd917c818f27ca4aa3f2945b527d8533edc92a450970a8ce06e9a762ea"
    "f774aebf4c3f246fcfb4687681c24f026a52f134e7808fe484f8dadeafb389a7c5379ffee27b8cbf5dc4a3c5dfe1de6eefa45ce7959674ba7f00f5d34631267a",
    "28dfb169eb7b55bb9ed8da655d8fa2aeab40d38575cd37176cf1ac3e9263dd774d1799775ee4acc91cd1e28bed6fab8c7f5dc4a3c5bf1771785c0cb702c7914c"
    "ee980f746b7b7da99260fcfbd8f9778c69d763c973dc979f599ee3eecf65bcc2f21ceee2b13c0799f1af6d3ebfaef5739f60daf3dcc11e245fcc7be8baa01935",
    "2089bca95b54eb2cf8f90c53afd7e5e96b4cbcf71dfc80e4a4eebf77fc4739aff1f3177fc5f8da453c5cbefe860d1eba1e91bc9533f6ae8cda65b75db92a07f9"
    "961a28874a3ec6d7ebb6ae593ec36a2c9f41160fb54dc163f90c32e35fdb7c7e5d79778c69cf33077b909cd5d3b17a3a1c3cd436058fd5d391191f777db76dc6",
    "477e447217f9fab9a89bf28e06247afb4d4cccb6efe1f216b8f132a5ba9bc93ffc5357d1de6f72fc934f3ea48987daa6e0d1e2dfcbec5e5018a647a3eec18e5a"
    "08b705a3920729eff0efbaafe331a6fe2c3e7eb5fd2c3e268387daa6e0b1f898ccf85e898f5bb0aff03a39defe3d07bb90fc0e6f67cd505dd0e2d244551fb9f8",
    "f9cdb9fe6d7dac730eb90923fae6f070eb334e1cec477242f37adb6f14f3cec1d33faccffbcd6d3cd436058f164fa794a3622c7a92bf12e561e15ce5e582911f"
    "79e87c23aff2f419a65d4ef1ec4423190c1f8e8771bfff1cdae2597d2427f1fd67ea2acaf5164f7ff57d1627bb88478b7f43a97076d486a35cba16bcec0e4545",
    "86728ec5c98c7f27a4222a8c7f57e45f5161fceb293c5afc5b2e6e9fcbb1e428cdc74be044c819b5f342d543fbf8ae6d3ebfae75176798f630deb51ae3ddd5f0"
    "50db143cc6bb64c6bfb6f93ce35d7bde65f906966f782c3c481b8fe51bc88c7f6df3f975dd8f37c6b4e79b0ef620f944bbe644ab661b6a12846a130e04ad2dc1",
    "cb2667aa85cfcb8bcd4e1fd448e58f80031e92e3f1b235bdaf7020f573ed7ffde377d9f33917f168f1b57e981c26135757c583e17628b71d8897a2c18087f8fa"
    "bf6c3ebfaa1fffcc667ce44724777d7d3f7ff52f34bb82a40a1ab5f856d4dbe2bc9f71afd7771dfc8ce48b751d0a2f0cb38a71a3c7df61ea9176d003c9ef37df",
    "6d7128f02a9ca8ef371d48b19ee2cb0fb6587ced221e2dbe0eaa95d38cc285f243e57c900ae4935a7a98f6105fafebfa5531f57edb416f241775c52a9935ccd7"
    "e0ae6f3ea338d75f662f921398a73987d1ab33fe3fc6b7aee2d1e2dbc3dd43b518ccf4f86e47b8c880cbbd9221873cb40f84c5c7f7b39be59badc6f2cdabe1a1",
    "b629782cdf4c667cc6cfeef133abc36075188f852f69e3b13a0c32e33fce7db8aff33cf0f90068d3d7a736550df2ecb9209979050e7848ce9e0be2e1a1b62978"
    "ecb92099f171d7376f333ef22392bf3f59dc555dd074bf06a111f42704bd6740d59f9220d713954e53068ad01f98bfd036ffeba6efdf49f839a8181a9424416b",
    "f2b2ca3577cca31564192aa64437343059f8b38a0e5271acd3f9432abc141ef27dd7b8cf0732b678561fc9b17819aaba7fea287af9e6bff99f213b77c8453c5a"
    "bc1b8d848fa4e393515a3d8968bac1f39df8b1e8a1f3ecd775fd8e31f57ee6a03792b3f386d879433878a86d0a1e3b6f88ccf8b8e7d2946cc6477e447252bcfc",
    "bc2d1928169db703f7fd22df71b003c997bd5735262a405b784b15a9fa90877e9f6acb011fc949beef6b993f699fe7f9397b4f94ab78d4f2d0f93097bb9204a0"
    "9d9694445fcae8f5dd18cb433ffaf58dcbe7df72b00bc917f97c00453eaa696094928061088aa8746ee975764fbd16dbaaf9e86b4c3f7ce88087e4a4e67789ff",
    "a6534b8d67bef86163de7faee3cddaa6e0518bbb0f323bf1aa5a974bad021fadb733b9a15cc830de7eecbc3dc6b40be33dacc2508d4a12e46ee97386a90fedf7"
    "b0561dec477242f33aef37aa3cfdef2cbe76158f164f370e86463d9788ebb1fc283e04b5fdd09592f3104f7b653de3c6d37fe0600792bf8297f3306ed576ccfb",
    "f70c532fdaef5ffdc0c10f48eec27ccefcb7da735fc6d3eb81478ba7bbe561a056ddabec48c1f42ea7068a85b251637910cfc7d3f73c37bfaf0b45a0b5a11637"
    "0b4e7ce4f8dae3e7e6dff61bc57dde79f973567fe7221e2d9e1e5dc8ea7e7dff400ad5c46e498865868341c343ef6bf5ca7a1e63da71cf3cc7a42b2a9d14d4d2",
    "c5ea2dbf9edd539fc5b66a5e7acdbe17cdfb8d6a9ee367a11ccb47bb88478b97f776f9c3ca91a8b5e29df6611d5c8ad1ab48c3437520ff6af3f955fdd8b3191f"
    "f911c9dd8f9f9fb72560648d598508dbd742e6fb1170c04372b6af050f0fb54dc163fb5ac88c8f1b8fd56dc6477e447262757c0a34f2205fd066ef99a3b58f45",
    "d415a0f8c8e5333cfd1eed89ab68d7ddf5d97bb45dc5a3f61eed93e06e43abc593a7a14caebb972f36b2c7a287ce4f5af7753cc6d4ff9983fe48cef6b5b07d2d"
    "3878a86d0a1edbd742667cafc4c3634c3b30eae980b635f9e20d6fe751ce30f5a15d4f5773b01fc909e5a55e588ef34f1d67de7369f1c867ac4ec3553c5abc5c",
    "2b5c24fa5aec281155c2178d88308817da230fed0367e76f2cb7eb2bb6767d65dad7fbf22d3fe2e279326fc10303b4fb8a7fe22cda798bff60fceb2a1e2dfedd"
    "065c4204bbf9705cee1baddc20ba1fcd1f7b887fd77d1de35e07ef39e88fe40bf1b0323d298febf6d8f3bbfbe42b6edc47fdf9dd3bcd1f305e76118f162f7722",
    "a5d87e3b18c94733b5d6693e7d01f6b6af182f3f1a5e3ec3d4ff6bb6fa7f0dc5bfe631a51b51875cc49e20551307c010fc33af51ac43fee3d65fb3e7772ee2e1"
    "f2ed376cf0d07588e4e18a5e49c9e9fea8526d19b1642bdd0db6659f77f8d62beb99c5c32c1e9eb78bc5c30f8bc7e26132e37b859fc798763c73b003c9599d05",
    "abb3c0c1436d53f0589d0599f1bdc2d3b4ae87092c27e87a73a6415354d4bea13fdcfe6a5c7e6e3bd88de464e76fb91769d65bfc14b03c879b78b4d6e3793653"
    "02fddd7c446e051ae5f356a8508ca7d8b9188f7e7db3bcc7fdec66798fcde04fda782cef41667caff2f518d3aedf76b00bc9efe4416a0267402d21127e5e49bb",
    "8eb9e2603f9213aa639ef31bddf3e6fee11fff96f1b48b78d4decf7d70d2501529943e3d3eca9647b56aa35faec4bdc3d35e59cf0f14478b7a45181ac7f0725d"
    "f3d20f1b47dfb88f7a1cfd05ab7376158f163f83c1d1def92079da2d298d7a2a5580da61b1e5a1bcf4a6adef55ed7a626b97753d8aba6e68a2d259d77d26795b",
    "3cab8fe4587590131799df7e90af68e69dff93f1afab78b4f857cee5eabbf0a263c0ccb037acc4f64bd95ec643e7e6b33cc672bbee791e33d07541336a401279"
    "2b95b1ae71f3ebcc2bf9fbea6d37d2e46deeb33ffdc8ec6f0a8fd2c6a3c5dbfdfaf6511ce48ee5d39386541dd5ca52c94832de7ef4eb7b8c69d73307bb909cd5",
    "e1b13a3c1c3cd436058fd5e19119dfabbc7d866997d379a0e884a375cd77503a47d0fc927473a6916f057f91e28b2f59bec3553c6afc9b0b0ee21cd74d4b55a1"
    "969403a162580db33a3bcff3af131f4e5490c1f0e1ea9f71f9f7d016cfea233989735ca7ae9a11302dbe78faabef33fe75118f5a3d462a9c1db5e12897ae052f",
    "bb435191a19cf350fccbf6a1586dd5fdfd1c945ba22258552149499005c5dc8672a3878aa9c7d3b9fe323d90bc3d7b6f4bb30b147e121893ba9f0a0ef8484e76"
    "3e977a95eafba6fe92ed4b71158fdaf3c17021df57c299e209c8ed1d0ef5e4e17e0a78a87e6ed3d637a97390445db8e803e9d1f0346efc7ce4808fe444de8330",
    "751ddd7ae6271fa759fcec221e2d3e8687bda3f08ec4e9dc287ac09d277761e3b0eea1e77eebbe8e717970bebf4c7f249fbe9ece52a0c9414d782c3c7c8d89ff"
    "91033e92e3dd4fad094477d3455fd2acc718fff7a78c975dc4a3c5cb7c672f17e0f5a2563d4957f7e47d1890fa210fc5c9d7369f5fd775cdf66ddfcf6ee08087",
    "e46cdf361e1e6a9b82c7f66d9319dfabf98c4f30edfa96835d48be58ef3c80221fd534304a49c0300445543ab7fc7d764fbd16dbaafc7d8de9870f1df0909cd4"
    "fbd197f88f6a1edaf7c50f1bf3fe731d6fd636058f5addc64166275e55eb72a955e0a3f5762637940b1eca7b7895b7cf30ed5a619fa00a345d208547bb6eae60",
    "8b67f5911c6f9fe0c445b38d825367513c877ffbdb2a8b9b5dc4a3c5bf1771785c0cb702c7914cee980f746b7b7da992f00efffe9bcde757f5e3c0667ce44724"
    "a7c2bfcf3928e5c039d4cca02b3b09c5c85d2fef3ad889e40b71b4595da28b5702b1fbc2ebf2f43526de070e76233976dee34e088d5c47f779e1bffcd12f196f",
    "bb88478bb7f7931727c1a3446f94e4603f9be3cac1ab14efa17ae74de1ed31a69ddf74b013c9a73973c568b6a12641a836e140d0da12bc6c725d81eb3d5cdee3"
    "71e6ad171f3f5869905738907adefad73f7eb74e130fb54dc1a3c5e3fa6172984c5c5d150f86dba1dc76205e8a06031eaa9b66efe35e6e97d33e12fd423348e2",
    "d1ce7f646df1ac3e9213d83768ba8a3affb27392dcc5a316471fe413fb994ca27110aa554f1291a3c865a893f20effaefb3a1e63eaffbb0efa23f99d73351250"
    "9ed07e52d3e0adfd7c2aa63eb87578b8fb906a0ef8484eec5cd8393fd2acbf63e788ba8b478b9f8b01a52883eae0045484dcee7e617059081f7928cfe1f5f5cc",
    "9e075a8d3d0f248b87daa6e0b1e78164c6f70adfb2ba39ab7f8de9075637e732deac6d0a1eab9b2333fea6e72deef93e2a9d0312d0b650f6f8e17819f73e5b75"
    "c0437242bcfcc272dccbb9a4c51fbffcdf4fd9791a2ee2d1e2e3d8612d1b310c55d8e7e4ed5ea4bad78964ca1e7a8eb7ee7cfc234cfdbfeba03f922fc6cb133b",
    "46e529b7a46619ddac52940087eab61f4b5e19f7392de7808fe4a4e2685bbf523ebfeed35f7c8fe53d5cc4a3c5dfe1de6eefa45ce7959674ba7f00f5d3463126"
    "7a28cfecd5f5cdf22056ff1ad30f2c0fe232deac6d0a1ecb839019dfabbc3dc6b4eb9eefb9eaeb4211686da8c5cdc23e1f3e5fbfb1f07b77f5597e2e34cdf364",
    "09cceb6dbf517cae98973f6775ce2ee2d1e2e9d185aceed7f70fa4504dec968458663818343cf45c91f1f472bbd8fe9457e32139db9f828787daa6e0b1fd2964"
    "c667bcbddcaec7f2fc11e9f3baf57aecf9e3dd9fcb78853d7f74178f3d7fc41bffff011943709e",
    "" };

  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 99392U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xResult;
  mxArray *xEntryPoints;
  const char * epFieldName[6] = { "Name", "NumberOfInputs", "NumberOfOutputs",
    "ConstantInputs", "FullPath", "TimeStamp" };

  mxArray *xInputs;
  const char * propFieldName[4] = { "Version", "ResolvedFunctions",
    "EntryPoints", "CoverageInfo" };

  xEntryPoints = emlrtCreateStructMatrix(1, 1, 6, epFieldName);
  xInputs = emlrtCreateLogicalMatrix(1, 2);
  emlrtSetField(xEntryPoints, 0, "Name", emlrtMxCreateString("constraints"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs", emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs", emlrtMxCreateDoubleScalar
                (2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath", emlrtMxCreateString(
    "/Users/root1/Desktop/Flocking_maneuvers/flock_maneuver/3D/controller_dmpc_3d/common/constraints.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp", emlrtMxCreateDoubleScalar
                (737944.97752314818));
  xResult = emlrtCreateStructMatrix(1, 1, 4, propFieldName);
  emlrtSetField(xResult, 0, "Version", emlrtMxCreateString(
    "9.8.0.1417392 (R2020a) Update 4"));
  emlrtSetField(xResult, 0, "ResolvedFunctions", (mxArray *)
                emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_constraints_info.c) */
