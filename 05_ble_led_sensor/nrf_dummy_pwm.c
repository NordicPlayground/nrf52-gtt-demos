#include "nrf_dummy_pwm.h"

void pwm_init(pwm_init_t *pwm_config)
{
	PWM->PSEL.OUT[0] =   (pwm_config->pin1 << PWM_PSEL_OUT_PIN_Pos)
												  | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	PWM->PSEL.OUT[1] =   (pwm_config->pin2 << PWM_PSEL_OUT_PIN_Pos)
												  | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	PWM->PSEL.OUT[2] =   (pwm_config->pin3 << PWM_PSEL_OUT_PIN_Pos)
												  | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	PWM->PSEL.OUT[3] =   (pwm_config->pin4 << PWM_PSEL_OUT_PIN_Pos)
												  | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
	PWM->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
	PWM->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
	PWM->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_4 << PWM_PRESCALER_PRESCALER_Pos);
	PWM->COUNTERTOP = (TIMER_RELOAD << PWM_COUNTERTOP_COUNTERTOP_Pos);
	PWM->DECODER =   (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos)
										  | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
	PWM->SEQ[0].PTR = ((uint32_t)(pwm_config->pwm_buffer) << PWM_SEQ_PTR_PTR_Pos);
	PWM->SEQ[0].CNT = ((pwm_config->pwm_buffer_size) << PWM_SEQ_CNT_CNT_Pos);
	PWM->SEQ[0].REFRESH = 40;
	PWM->SEQ[0].ENDDELAY = 0;
	PWM->SEQ[1].PTR = ((uint32_t)(pwm_config->pwm_buffer) << PWM_SEQ_PTR_PTR_Pos);
	PWM->SEQ[1].CNT = ((pwm_config->pwm_buffer_size) << PWM_SEQ_CNT_CNT_Pos);
	PWM->SEQ[1].REFRESH = 40;
	PWM->SEQ[1].ENDDELAY = 0;
}

void pwm_run(bool run)
{
    if(run)
    {
        PWM->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;
        PWM->LOOP = (1 << PWM_LOOP_CNT_Pos);
        PWM->TASKS_SEQSTART[0] = 1;
    }
    else
    {
        PWM->SHORTS = PWM_SHORTS_LOOPSDONE_STOP_Msk;
    }
}
