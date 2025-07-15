/**
 * @file lw_pid.c
 * @author watashiwacaicai (woshixiaoqi676@gmail.com)
 * @version 1.3.0
 * @date 2025-7-15
 * 
 * @copyright (c) 2025 watashiwacaicai
 * 
 */

#include "lw_pid.h"
#include "math.h"
#include "stdbool.h"

/********************静态函数声明********************/
static float increment_pid_compute(Pid_object_t* pid_object, float current_input);
static float fullscale_pid_compute(Pid_object_t* pid_object, float current_input);

/********************全局函数定义********************/

/**
 * @brief	使能pid计算
 * @param	pid_object pid对象
 */
void pid_enable(Pid_object_t* pid_object)
{
	pid_object->pid_state = PID_CALCU_ENABLE;
}

/**
 * @brief	禁用pid计算
 * @param	pid_object pid对象
 */
void pid_disable(Pid_object_t* pid_object)
{
	pid_object->pid_state = PID_CALCU_DISABLE;
	pid_object->error0 = 0.0f;
	pid_object->error1 = 0.0f;
	pid_object->error2 = 0.0f;
	pid_object->error_intergral = 0.0f;
}

/**
 * @brief	设置pid参数
 * @param	pid_object pid对象
 * @param	kp 比例参数
 * @param	ki 积分参数
 * @param	kd 微分参数
 * @return	无
 */
void pid_set_argument(Pid_object_t* pid_object, float kp, float ki, float kd)
{
	pid_object->kp = kp;
	pid_object->ki = ki;
	pid_object->kd = kd;
}

/**
 * @brief	设置pid目标值
 * @param	pid_object pid对象
 * @param	target 目标值	
 * @return	无
 */
void pid_set_target(Pid_object_t* pid_object, float target)
{
	pid_object->target = target;
}

/**
 * @brief	使能pid输出限幅
 * @param	pid_object pid对象
 * @return	无
 */
void pid_output_limit_enable(Pid_object_t* pid_object)
{
	pid_object->output_limit_state = PID_OUTPUT_LIMIT_ENABLE;
}

/**
 * @brief	禁用pid输出限幅
 * @param	pid_object pid对象
 * @return	无
 */
void pid_output_limit_disable(Pid_object_t* pid_object)
{
	pid_object->output_limit_state = PID_OUTPUT_LIMIT_DISABLE;
}

/**
 * @brief	设置pid输出限幅上下限
 * @param	pid_object pid对象
 * @param	upper_limit 输出上限
 * @param	floor_limit 输出下限
 * @return	无
 */
void pid_set_output_limit(Pid_object_t* pid_object, float upper_limit, float floor_limit)
{
	pid_object->output_upper_limit = upper_limit;
	pid_object->output_floor_limit = floor_limit;
}

/**
 * @brief	使能pid的积分限幅
 * @param	pid_object pid对象
 * @return	无
 */
void pid_intergral_limit_enable(Pid_object_t* pid_object)
{
	pid_object->intergral_limit_state = PID_INTEGRAL_LIMIT_ENABLE;
}

/**
 * @brief	禁用pid的积分限幅
 * @param	pid_object pid对象
 * @return	无
 */
void pid_intergral_limit_disable(Pid_object_t* pid_object)
{
	pid_object->intergral_limit_state = PID_INTEGRAL_LIMIT_DISABLE;
}

/**
 * @brief	设置pid的积分限幅上下限
 * @param	pid_object pid对象
 * @param	upper_limit 输出上限
 * @param	floor_limit 输出下限
 * @return	无
 */
void pid_set_intergral_limit(Pid_object_t* pid_object, float upper_limit, float floor_limit)
{
	pid_object->intergral_upper_limit = upper_limit;
	pid_object->intergral_floor_limit = floor_limit;
}

/**
 * @brief	使能pid的输入死区
 * @param	pid_object pid对象
 * @return	无
 */
void pid_input_dead_zone_enable(Pid_object_t* pid_object)
{
	pid_object->input_dead_zone_state = PID_INPUT_DEAD_ZONE_ENABLE;
}

/**
 * @brief	失能pid的输入死区
 * @param	pid_object pid对象
 * @return	无
 */
void pid_input_dead_zone_disable(Pid_object_t* pid_object)
{
	pid_object->input_dead_zone_state = PID_INPUT_DEAD_ZONE_DISABLE;
}

/**
 * @brief	设置pid的输入死区值
 * @param	pid_object pid对象
 * @param	input_dead_zone_val 输入死区
 * @return	无
 */
void pid_set_input_dead_zone(Pid_object_t* pid_object, float input_dead_zone_val)
{
	pid_object->input_dead_zone = input_dead_zone_val;
}

/**
 * @brief	使能pid的输出偏移
 * @param	pid_object pid对象
 * @return	无
 */
void pid_output_offset_enable(Pid_object_t* pid_object)
{
	pid_object->output_offset_state = PID_OUTPUT_OFFSET_ENABLE;
}

/**
 * @brief	禁用pid的输出偏移
 * @param	pid_object pid对象
 * @return	无
 */
void pid_output_offset_disable(Pid_object_t* pid_object)
{
	pid_object->output_offset_state = PID_OUTPUT_OFFSET_DISABLE;
}

/**
 * @brief	设置pid的输出偏移值
 * @param	pid_object pid对象
 * @param	output_offset_val 输出偏移
 * @return	无
 */
void pid_set_output_offset(Pid_object_t* pid_object, float output_offset_val)
{
	pid_object->output_offset = output_offset_val;
}

/**
 * @brief	使能增量式pid的全量输出
 * @param	pid_object pid对象
 * @return	无
 */
void pid_incre_output_fullscale_enable(Pid_object_t* pid_object)
{
	pid_object->incre_output_fullscale_state = PID_INCRE_OUTPUT_FULLSCALE_ENABLE;
}

/**
 * @brief	失能增量式pid的全量输出
 * @param	pid_object pid对象
 * @return	无
 */
void pid_incre_output_fullscale_disable(Pid_object_t* pid_object)
{
	pid_object->incre_output_fullscale_state = PID_INCRE_OUTPUT_FULLSCALE_DISABLE;
}

/**
 * @brief	设置pid为增量式
 * @param	pid_object pid对象
 * @return	无
 */
void pid_set_mode_increment(Pid_object_t* pid_object)
{
	pid_object->pid_mode = PID_MODE_INCREMENT;
}

/**
 * @brief	设置pid为位置式
 * @param	pid_object pid对象
 * @return	无
 */
void pid_set_mode_fullscale(Pid_object_t* pid_object)
{
	pid_object->pid_mode = PID_MODE_FULLSCALE;
}

/**
 * @brief	初始化pid对象
 * @param	pid_object pid对象
 * @return	无
 */
void pid_init(Pid_object_t* pid_object)
{
	pid_object->error0 = 0.0f;
	pid_object->error1 = 0.0f;
	pid_object->error2 = 0.0f;
	pid_object->output = 0.0f;
	pid_object->error_intergral = 0.0f;
	
	/*如果选择为增量式*/
	if(pid_object->pid_mode == PID_MODE_INCREMENT)
	{
		pid_object->pid_compute = increment_pid_compute;		
	}
	else if(pid_object->pid_mode == PID_MODE_FULLSCALE) /*如果选择为位置式*/
	{
		pid_object->pid_compute = fullscale_pid_compute;
	}
	
	pid_object->pid_state = PID_CALCU_ENABLE; /*初始化后的pid默认使能*/
}

/**
 * @brief	pid运算函数
 * @param	pid_object pid对象
 * @param	current_input 当前反馈输入
 * @return	运算结果
 */
float pid_calcu(Pid_object_t* pid_object, float current_input)
{
	float output = 0.0f;
	
	/*如果使能了pid*/
	if(pid_object->pid_state == PID_CALCU_ENABLE)
	{
		output = pid_object->pid_compute(pid_object, current_input);
	}
	else
	{
		output = 0.0f;
	}
	
	return output;
}

/********************静态函数定义********************/

/*计算增量式pid*/
static float increment_pid_compute(Pid_object_t* pid_object, float current_input)
{
	int is_need_pid_calcu = 0;
	float delta_output = 0.0f;
	
	/*输入安全检查*/
	if(isnan(current_input))
	{
		while(1); /*如果您的程序停止在了这里，说明pid出现了非法的输入*/
	}
	
	/*误差传递*/
	pid_object->error2 = pid_object->error1;
	pid_object->error1 = pid_object->error0;
	pid_object->error0 = pid_object->target - current_input;
	
	/*如果使能了输入死区*/
	if(pid_object->input_dead_zone_state == PID_INPUT_DEAD_ZONE_ENABLE)
	{
		/*如果当前误差小于死区*/
		if(fabs(pid_object->error0) < pid_object->input_dead_zone)
		{
			pid_object->output = 0.0f;
			is_need_pid_calcu = false; /*不计算pid*/
		}
		else
		{
			is_need_pid_calcu = true; /*计算pid*/
		}
	}
	else
	{
		is_need_pid_calcu = true; /*计算pid*/
	}
	
	/*如果需要计算pid*/
	if(is_need_pid_calcu == true)
	{
	
		delta_output = pid_object->kp * (pid_object->error0 - pid_object->error1) + pid_object->ki * pid_object->error0
								+ pid_object->kd * (pid_object->error0 - 2 * pid_object->error1 + pid_object->error2);
		
		/*如果使能了增量式pid全量化输出*/
		if(pid_object->incre_output_fullscale_state == PID_INCRE_OUTPUT_FULLSCALE_ENABLE)
		{
			pid_object->output = pid_object->output + delta_output;
		}
		else
		{
			pid_object->output = delta_output;
		}
		
		/*如果使能了输出偏移*/
		if(pid_object->output_offset_state == PID_OUTPUT_OFFSET_ENABLE)
		{
			if(pid_object->output > 0.0f)
			{
				pid_object->output += pid_object->output_offset;
			}
			else if(pid_object->output < 0.0f)
			{
				pid_object->output -= pid_object->output_offset;
			}
		}
		
		/*如果使能了输出限幅*/
		if(pid_object->output_limit_state == PID_OUTPUT_LIMIT_ENABLE)
		{
			if(pid_object->output > pid_object->output_upper_limit)
			{
				pid_object->output = pid_object->output_upper_limit;
			}
			if(pid_object->output < pid_object->output_floor_limit)
			{
				pid_object->output = pid_object->output_floor_limit;
			}
		}		
	}
			
	return pid_object->output;
}

/*计算位置式pid*/
static float fullscale_pid_compute(Pid_object_t* pid_object, float current_input)
{	
	int is_need_pid_calcu = 0;
	
	/*输入安全检查*/
	if(isnan(current_input))
	{
		while(1); /*如果您的程序停止在了这里，说明pid出现了非法的输入*/
	}
	
	/*误差传递*/
	pid_object->error1 = pid_object->error0;
	pid_object->error0 = pid_object->target - current_input;
	
	/*如果使能了输入死区*/
	if(pid_object->input_dead_zone_state == PID_INPUT_DEAD_ZONE_ENABLE)
	{
		/*如果当前误差小于死区*/
		if(fabs(pid_object->error0) < pid_object->input_dead_zone)
		{
			pid_object->output = 0.0f;
			is_need_pid_calcu = false; /*不计算pid*/
		}
		else
		{
			is_need_pid_calcu = true; /*计算pid*/
		}
	}
	else
	{
		is_need_pid_calcu = true; /*计算pid*/
	}
	
	if(is_need_pid_calcu == true)
	{
		/*计算积分*/
		pid_object->error_intergral += pid_object->error0;
		
		/*如果使能了积分限幅*/
		if(pid_object->intergral_limit_state == PID_INTEGRAL_LIMIT_ENABLE)
		{
			if(	pid_object->error_intergral > pid_object->intergral_upper_limit)
			{
				pid_object->error_intergral = pid_object->intergral_upper_limit;
			}
			if(pid_object->error_intergral < pid_object->intergral_floor_limit)
			{
				pid_object->error_intergral = pid_object->intergral_floor_limit;
			}
		}

		pid_object->output = pid_object->kp * pid_object->error0 + pid_object->ki * pid_object->error_intergral 
									+ pid_object->kd * (pid_object->error0 - pid_object->error1);
	
		/*如果使能了输出偏移*/
		if(pid_object->output_offset_state == PID_OUTPUT_OFFSET_ENABLE)
		{
			if(pid_object->output > 0.0f)
			{
				pid_object->output += pid_object->output_offset;
			}
			else if(pid_object->output < 0.0f)
			{
				pid_object->output -= pid_object->output_offset;
			}
		}
			
			/*如果使能了输出限幅*/
		if(pid_object->output_limit_state == PID_OUTPUT_LIMIT_ENABLE)
		{
			if(pid_object->output > pid_object->output_upper_limit)
			{
				pid_object->output = pid_object->output_upper_limit;
			}
			if(pid_object->output < pid_object->output_floor_limit)
			{
				pid_object->output = pid_object->output_floor_limit;
			}
		}
	}
	
	return pid_object->output;	
}
