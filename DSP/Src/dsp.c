

#include "stm32f407xx.h"
#include "arm_math.h"
#include "math_helper.h"
#include "math.h"
#include "dsp.h"
#include "defines.h"
//bs50
arm_biquad_cascade_df2T_instance_f32 iir_bs50_0;
arm_biquad_cascade_df2T_instance_f32 iir_bs50_1;
arm_biquad_cascade_df2T_instance_f32 iir_bs50_2;
arm_biquad_cascade_df2T_instance_f32 iir_bs50_3;
arm_biquad_cascade_df2T_instance_f32 iir_bs50_4;
arm_biquad_cascade_df2T_instance_f32 iir_bs50_5;
arm_biquad_cascade_df2T_instance_f32 iir_bs50_6;
arm_biquad_cascade_df2T_instance_f32 iir_bs50_7;
float32_t iir_state_bs50[96];
uint8_t numStages_bs50 = 6;
const float32_t iir_coefs_bs50[30];
//bs100
arm_biquad_cascade_df2T_instance_f32 iir_bs100_0;
arm_biquad_cascade_df2T_instance_f32 iir_bs100_1;
arm_biquad_cascade_df2T_instance_f32 iir_bs100_2;
arm_biquad_cascade_df2T_instance_f32 iir_bs100_3;
arm_biquad_cascade_df2T_instance_f32 iir_bs100_4;
arm_biquad_cascade_df2T_instance_f32 iir_bs100_5;
arm_biquad_cascade_df2T_instance_f32 iir_bs100_6;
arm_biquad_cascade_df2T_instance_f32 iir_bs100_7;
float32_t iir_state_bs100[96];
uint8_t numStages_bs100 = 6;
const float32_t iir_coefs_bs100[30];
//bp3-499
arm_biquad_cascade_df2T_instance_f32 iir_bp_0;
arm_biquad_cascade_df2T_instance_f32 iir_bp_1;
arm_biquad_cascade_df2T_instance_f32 iir_bp_2;
arm_biquad_cascade_df2T_instance_f32 iir_bp_3;
arm_biquad_cascade_df2T_instance_f32 iir_bp_4;
arm_biquad_cascade_df2T_instance_f32 iir_bp_5;
arm_biquad_cascade_df2T_instance_f32 iir_bp_6;
arm_biquad_cascade_df2T_instance_f32 iir_bp_7;
float32_t iir_state_bp[112];
uint8_t numStages_bp = 7;
const float32_t iir_coefs_bp[35];

//20Hz 3 Ord. Smooth
arm_biquad_cascade_df2T_instance_f32 iir_lp_smooth;
const float32_t iir_coefs_lp_smooth[10];
float32_t iir_state_lp_smooth[4];
uint8_t numStages_lp_smooth = 2;

extern float32_t dsp_res1[ELECTRODESNUMBER];
extern float32_t dsp_res2[ELECTRODESNUMBER];
extern float32_t sigma;

/*Раскомеентировать эту строку, если ODR = 1000 Гц, и закоментировать строку с настройками на ODR = 500 Гц
/*
//#if ODR == 1000

//BS 50Hz ODR = 1000 Hz
const float32_t iir_coefs_bs50[30] = {
+1.6158108710f, -3.0687346548f, +1.6158108710f,
+1.8845956330f, -0.9917417169f,
+1.6158108710f, -3.0785129928f, +1.6158108710f,
+1.9040467740f, -0.9924780130f,
+4.7968544960f, -9.1193437289f, +4.7968544960f,
+1.8475230930f, -0.9561412334f,
+4.7968544960f, -9.1303531672f, +4.7968544960f,
+1.8768714670f, -0.9611843824f,
+0.1221401542f, -0.2318451637f, +0.1221401542f,
+1.8927204610f, -0.9983312488f,
+0.1221401542f, -0.2328208600f, +0.1221401542f,
+1.9081633090f, -0.9984539151f
};

//BS 100Hz ODR = 1000 Hz
const float32_t iir_coefs_bs100[30] = {
+1.5731073620f, -2.5266880705f, +1.5731073620f,
+1.5690711740f, -0.9863000512f,
+1.5731073620f, -2.5650538797f, +1.5731073620f,
+1.6435904500f, -0.9874315262f,
+5.0831875800f, -8.2036186098f, +5.0831875800f,
+1.4891707900f, -0.9139574170f,
+5.0831875800f, -8.2508607081f, +5.0831875800f,
+1.6098710300f, -0.9245311618f,
+0.1121902913f, -0.1797533077f, +0.1121902913f,
+1.5857819320f, -0.9973062277f,
+0.1121902913f, -0.1833492354f, +0.1121902913f,
+1.6450474260f, -0.9974864721f
};

//BP 3 Hz - 499 Hz ODR = 1000 Hz
const float32_t iir_coefs_bp[35] = {
+0.4902730882f, +0.9796635383f, +0.4902730882f,
-1.9919048550f, -0.9958600998f,
+0.4902730882f, -0.9804667497f, +0.4902730882f,
+1.9983989000f, -0.9987554550f,
+0.9930749536f, +1.9848796491f, +0.9930749536f,
+1.9938833710f, -0.9943691492f,
+0.9930749536f, -1.9860356668f, +0.9930749536f,
-1.9760112760f, -0.9813702106f,
+0.9714930058f, +1.9425491730f, +0.9714930058f,
+1.9742199180f, -0.9754008055f,
+0.9714930058f, -1.9429467516f, +0.9714930058f,
-1.9077731370f, -0.9204980731f,
+3.5448429580f, +0.0000000000f, -3.5448429580f,
+0.1560278684f, +0.7102911472f
};


//Smooth LP 20Hz 3 Ord. ODR = 1000 Hz
const float32_t iir_coefs_lp_smooth[10] = {
0.0001691240f, 0.0003382479f, 0.0001691240f,
1.973484755f, -0.9741612673f,
0.0129215643f, 0.0129215643f, 0.0f,
0.9741568565f, 0.0f
};

//#elif ODR == 500
*/

/*Раскомеентировать эту строку, если ODR = 500 Гц, и закоментировать строку с настройками на ODR = 1000 Гц */

//BS 50Hz ODR = 500 Hz
const float32_t iir_coefs_bs50[30] = {
+1.6089665890f, -2.5854098224f, +1.6089665890f,
+1.5683093070f, -0.9836370945f,
+1.6089665890f, -2.6224601718f, +1.6089665890f,
+1.6404006480f, -0.9849433303f,
+4.6970534320f, -7.5817770172f, +4.6970534320f,
+1.5013834240f, -0.9147599936f,
+4.6970534320f, -7.6227993439f, +4.6970534320f,
+1.5998539920f, -0.9234134555f,
+0.1220215932f, -0.1956099434f, +0.1220215932f,
+1.5857373480f, -0.9966791868f,
+0.1220215932f, -0.1993188953f, +0.1220215932f,
+1.6441680190f, -0.9968982935f
};

//BS 100Hz ODR = 500 Hz
const float32_t iir_coefs_bs100[30] = {
+1.5608187910f, -1.0270875033f, +1.5608187910f,
+0.7267251015f, -0.9744894505f,
+1.5608187910f, -0.9037974307f, +1.5608187910f,
+0.4900162220f, -0.9734368920f,
+4.8704876900f, -3.0870635879f, +4.8704876900f,
+0.3904991150f, -0.8411195278f,
+4.8704876900f, -2.9404259696f, +4.8704876900f,
+0.7413055301f, -0.8502965569f,
+0.1119695157f, -0.0750477346f, +0.1119695157f,
+0.7111777663f, -0.9948909283f,
+0.1119695157f, -0.0634260532f, +0.1119695157f,
+0.5201193094f, -0.9947212338f
};

//BP 3 Hz - 499 Hz ODR = 500 Hz
const float32_t iir_coefs_bp[35] = {
    +0.4797780216f, +0.9593143416f, +0.4797780216f,
    -1.995836973f, -0.9972631931f,
    +0.4797780216f, -0.9594486329f, +0.4797780216f,
    +1.997540116f, -0.9981743693f,
    +0.9943896532f, +1.9884325757f, +0.9943896532f,
    +1.991021752f, -0.9919091463f,
    +0.9943896532f, -1.9886252039f, +0.9943896532f,
    -1.985898376f, -0.9878907204f,
    +0.977118969f, +1.9541225051f, +0.977118969f,
    +1.963336945f, -0.9655727148f,
    +0.977118969f, -1.9541865699f, +0.977118969f,
    -1.943847179f, -0.9488319755f,
    +3.812615395f, +0.0f, -3.812615395f,
    +0.04793407395f, +0.7603674531f
};

//Smooth LP 20Hz 3 Ord. ODR = 1000 Hz
const float32_t iir_coefs_lp_smooth[10] = {
0.0001691240f, 0.0003382479f, 0.0001691240f,
1.973484755f, -0.9741612673f,
0.0129215643f, 0.0129215643f, 0.0f,
0.9741568565f, 0.0f
};

//#endif


//------------------------------------------------------------------------------
void dsp_init (void)
{
  //bp 3 Hz - 499 Hz
  arm_biquad_cascade_df2T_init_f32(&iir_bp_0, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[0 * numStages_bp * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bp_1, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[1 * numStages_bp * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bp_2, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[2 * numStages_bp * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bp_3, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[3 * numStages_bp * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bp_4, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[4 * numStages_bp * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bp_5, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[5 * numStages_bp * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bp_6, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[6 * numStages_bp * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bp_7, numStages_bp, (float32_t *)&iir_coefs_bp[0], &iir_state_bp[7 * numStages_bp * 2]);
  //bs 50 Hz
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_0, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[0 * numStages_bs50 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_1, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[1 * numStages_bs50 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_2, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[2 * numStages_bs50 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_3, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[3 * numStages_bs50 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_4, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[4 * numStages_bs50 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_5, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[5 * numStages_bs50 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_6, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[6 * numStages_bs50 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs50_7, numStages_bs50, (float32_t *)&iir_coefs_bs50[0], &iir_state_bs50[7 * numStages_bs50 * 2]);
  //bs 100 Hz
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_0, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[0 * numStages_bs100 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_1, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[1 * numStages_bs100 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_2, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[2 * numStages_bs100 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_3, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[3 * numStages_bs100 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_4, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[4 * numStages_bs100 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_5, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[5 * numStages_bs100 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_6, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[6 * numStages_bs100 * 2]);
  arm_biquad_cascade_df2T_init_f32(&iir_bs100_7, numStages_bs100, (float32_t *)&iir_coefs_bs100[0], &iir_state_bs100[7 * numStages_bs100 * 2]);
  //20Hz 3 Ord. Smooth
  arm_biquad_cascade_df2T_init_f32(&iir_lp_smooth, numStages_lp_smooth, (float32_t *)&iir_coefs_lp_smooth[0], &iir_state_lp_smooth[0 * numStages_lp_smooth * 2]);
}


//------------------------------------------------------------------------------
void dsp_pocess (void)
{
      //bp 3 Hz - 499 Hz
      arm_biquad_cascade_df2T_f32(&iir_bp_0, &dsp_res1[0], &dsp_res2[0], 1);
      arm_biquad_cascade_df2T_f32(&iir_bp_1, &dsp_res1[1], &dsp_res2[1], 1);
      arm_biquad_cascade_df2T_f32(&iir_bp_2, &dsp_res1[2], &dsp_res2[2], 1);
      arm_biquad_cascade_df2T_f32(&iir_bp_3, &dsp_res1[3], &dsp_res2[3], 1);
      arm_biquad_cascade_df2T_f32(&iir_bp_4, &dsp_res1[4], &dsp_res2[4], 1);
      arm_biquad_cascade_df2T_f32(&iir_bp_5, &dsp_res1[5], &dsp_res2[5], 1);
      arm_biquad_cascade_df2T_f32(&iir_bp_6, &dsp_res1[6], &dsp_res2[6], 1);
      arm_biquad_cascade_df2T_f32(&iir_bp_7, &dsp_res1[7], &dsp_res2[7], 1);
      //bs 100 Hz
      arm_biquad_cascade_df2T_f32(&iir_bs100_0, &dsp_res2[0], &dsp_res1[0], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs100_1, &dsp_res2[1], &dsp_res1[1], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs100_2, &dsp_res2[2], &dsp_res1[2], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs100_3, &dsp_res2[3], &dsp_res1[3], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs100_4, &dsp_res2[4], &dsp_res1[4], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs100_5, &dsp_res2[5], &dsp_res1[5], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs100_6, &dsp_res2[6], &dsp_res1[6], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs100_7, &dsp_res2[7], &dsp_res1[7], 1);
     //bs 50Hz
      arm_biquad_cascade_df2T_f32(&iir_bs50_0, &dsp_res1[0], &dsp_res2[0], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs50_1, &dsp_res1[1], &dsp_res2[1], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs50_2, &dsp_res1[2], &dsp_res2[2], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs50_3, &dsp_res1[3], &dsp_res2[3], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs50_4, &dsp_res1[4], &dsp_res2[4], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs50_5, &dsp_res1[5], &dsp_res2[5], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs50_6, &dsp_res1[6], &dsp_res2[6], 1);
      arm_biquad_cascade_df2T_f32(&iir_bs50_7, &dsp_res1[7], &dsp_res2[7], 1);
     
      //Коэфф. передачи трех фильтров (lp, bs100, bs50) по потсоянному току = 0,748 - результат измерения

}

/********************************************************************************************/

float32_t dispers (float32_t* win, uint32_t winSize)
{
  float32_t rdisp = 0;
  
  float32_t mean = 0;
  float32_t rqrt = 0;
  float32_t wincopy[50];
  
  arm_mean_f32(win, winSize, &mean);
  arm_negate_f32(&mean, &mean, 1);
  arm_offset_f32(win, mean, wincopy, winSize);
  arm_power_f32(wincopy, winSize, &rqrt);
  arm_scale_f32(&rqrt, 2.0408163265e-2f, &rdisp, 1);
  
  //arm_var_f32(win, winSize, &rdisp);
  return rdisp;
}

//------------------------------------------------
void sig(float32_t* smth, float32_t offset)
{
  float32_t x = 0;
  //float32_t sigma = 0;
   
  x = *smth - offset;
  sigma = (float32_t)(1 / (1 + expf(-x)));
  //return sigma;
}


