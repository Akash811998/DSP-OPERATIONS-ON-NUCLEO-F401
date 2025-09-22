
#include "main.h"
#include "arm_math.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"



#define HZ_5_SIG_LEN    301
#define KHZ1_15_SIG_LEN 320
#define IMP_RESP_LENGTH 29

void plot_input_signal(void);

float32_t signal_mean(float32_t *sig_src_arr, uint32_t sig_length);

float32_t signal_variance(float32_t *sig_src_arr,float32_t sig_mean, uint32_t sig_length);

 float32_t signal_standard_deviation(float32_t);

 void plot_impulse_response(void);

 void convolution(float32_t* sig_source,float32_t* sig_dest,float32_t* imp_resp, uint32_t sig_input_length, uint32_t imp_resp_len);

 void convolution_print_all_signals();

 void calc_running_sum(float32_t *sig_input, float32_t *sig_output, uint32_t sig_length);

 void calc_first_difference(float32_t *sig_input, float32_t *sig_output, uint32_t sig_length);

 void plot_runningsum_firstdifference(void);