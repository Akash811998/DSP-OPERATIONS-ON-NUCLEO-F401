
#include "main.h"
#include "arm_math.h"



#define HZ_5_SIG_LEN    301
#define KHZ1_15_SIG_LEN 320

void plot_input_signal(void);

float32_t signal_mean(float32_t *sig_src_arr, uint32_t sig_length);

float32_t signal_variance(float32_t *sig_src_arr,float32_t sig_mean, uint32_t sig_length);

 float32_t signal_standard_deviation(float32_t);