这是一个轻量级的pid工具库，目前支持位置式pid和增量式pid的计算，同时也具备输出限幅与积分限幅（仅位置式pid）的功能，不依赖于具体的硬件，可在任意C语言平台上使用。

### 如何移植

因为该工具库不依赖于具体的硬件，直接添加进自己的工程就可以编译使用。如果出现头文件报错，可以将`lw_pid.c`文件开头的`#include "./Lw_Pid/lw_pid.h"`修改为`#include "lw_pid.h"`并添加具体的头文件包含目录。如果代码文件内的中文注释出现乱码，那是打开文件的编码不正确所致，本工具库使用utf8编码编写，用户可以将代码文件转换成自己的编码再进行使用。

### 如何使用

1. 定义pid对象，如：

`Pid_object_t motor_pid;`

特别注意，一定要将pid对象定义成具体的一般变量，不能单独定义成指针。如果一定要使用指针，可以这样写：

```c
Pid_object_t motor_pid;
Pid_object_t* p_motor_pid = &motor_pid;
```



2. 根据自己的需求初始化pid对象。用户可以编写一个专门的函数来对pid对象进行初始化。增量式pid因为没有积分限幅，在初始化的时候用户可以不做相应设置。下面以位置式pid为例编写代码：

```c
/*初始化电机的位置式pid*/
void motor_pid_init(void)
{
	pid_set_argument(&motor_pid, 0.23, 0.67, 1.2); /*设置pid参数*/
	pid_set_target(&motor_pid, 0.0f); /*设置目标数值*/
	pid_output_limit_enable(&motor_pid); /*使能输出限幅*/
	pid_set_output_limit(&motor_pid, 500.0, -500.0); /*设置输出限幅的上下限*/
	pid_intergral_limit_enable(&motor_pid); /*使能积分限幅*/
	pid_set_intergral_limit(&motor_pid, 250.0, -250.0); /*设置积分限幅的上下限*/
	pid_set_mode_fullscale(&motor_pid); /*设置使用位置式pid*/
	pid_init(&motor_pid); /*初始化电机pid对象*/
}
```



3. 进行pid计算。pid算法对实时性要求很高，一般建议用户将pid的计算过程放到定时中断里面。本工具库内部所有的数据存储以及计算都是float类型的，在使用的时候用户变量的数据类型最好相匹配。不管用户使用的是增量式pid还是位置式pid，都只需要调用一个固定的回调函数来进行计算即可：

```c
void tim_handler(void)
{
    float pid_out;
	pid_out = motor_pid.pid_compute(&motor_pid, 230.0);
    motor_set_pwm((uint16_t)pid_out);
}
```



