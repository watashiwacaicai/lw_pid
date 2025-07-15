这是一个轻量级的pid工具库，目前支持位置式pid和增量式pid的计算，同时也具备输出限幅，输出偏移，输入死区与积分限幅（仅位置式pid）的功能，不依赖于具体的硬件，可在任意C语言平台上使用。

### 如何移植

因为该lw_pid不依赖于任何具体的硬件，直接添加进自己的工程就可以编译使用。

### 如何使用

1. 定义pid对象，如：

`Pid_object_t motor_pid;`

特别注意，一定要将pid对象定义成具体的一般变量，不能单独定义成指针。如果一定要使用指针，可以这样写：

```c
Pid_object_t motor_pid;
Pid_object_t* p_motor_pid = &motor_pid;
```



2. 根据自己的需求初始化pid对象。用户可以编写一个专门的函数来对pid对象进行初始化。增量式pid因为没有积分限幅，在初始化的时候用户可以不做相应设置。不过增量式pid有一个是否将增量输出为全量的选项，需要用户在初始化时进行设置，将增量输出为全量时每一次pid运算得到的增量都会累加到最终的输出结果中，否则仅输出当前计算的增量，一般只有少数器件可以支持纯增量控制。经过初始化的pid对象默认是使能计算的，被失能计算的pid对象其输出无论如何都为0。同时用户还需要注意一点：特定pid类型支持但自己没有使用到的功能在初始化时必须显示地禁用，避免发生意想不到的问题。下面是两个代码示例：

```c
/**
 * @note 需要依次设置好参数，目标值，输出限幅使用，输入死区使用，输出偏移使用
		 积分限幅使用（仅位置式），增量输出全量（仅增量式），不需要的功能一定要显式禁用，避免意外
 */
void motor_pid_init(void)
{
	pid_set_argument(&l_speed_pid, 4.0f, 0.42f, 1.05f); /*设置参数*/
	pid_set_target(&l_speed_pid, 0.0f); /*设置目标值*/
	pid_output_limit_disable(&l_speed_pid); /*禁用输出限幅*/
	pid_output_offset_enable(&l_speed_pid); /*使能输出偏移*/
	pid_set_output_offset(&l_speed_pid, 300.0f); /*设置输出偏移*/
	pid_intergral_limit_enable(&l_speed_pid);/*启用积分限幅*/
	pid_set_intergral_limit(&l_speed_pid,1800.0f,-1800.0f);/*设置积分限幅*/
	pid_input_dead_zone_disable(&l_speed_pid); /*禁用输入死区*/
	pid_set_mode_fullscale(&l_speed_pid); /*设置为位置式pid*/
	pid_init(&l_speed_pid);
	
	pid_set_argument(&angle_pid, 3.0f, 0.0f, 0.0f); /*设置参数*/
	pid_set_target(&angle_pid, 179.0f); /*设置目标值*/
	pid_output_limit_enable(&angle_pid); /*使能输出限幅*/
	pid_set_output_limit(&angle_pid, 80.0f, -80.0f); /*设置输出限幅*/
	pid_output_offset_enable(&angle_pid); /*使能输出偏移*/
	pid_set_output_offset(&angle_pid, 10.0f); /*设置输出偏移*/
	pid_input_dead_zone_disable(&angle_pid); /*禁用输入死区*/
	pid_incre_output_fullscale_enable(&angle_pid); /*设置将增量输出为全量*/
	pid_set_mode_increment(&angle_pid); /*设置为增量式pid*/
	pid_init(&angle_pid);
}
```



3. 进行pid计算。pid算法对实时性要求很高，一般建议用户将pid的计算过程放到定时中断里面，如果主程序的调度速度足够快，也可以放到主程序里面。lw_pid内部所有的数据存储以及计算都是基于float类型的，在使用的时候，用户变量的数据类型最好与之相匹配。最后，不管用户使用的是增量式pid还是位置式pid，都只需要调用一个固定的函数来计算即可：

```c
void tim_handler(void)
{
    float pid_out = 0.0f;
	pid_out = pid_calcu(&motor_pid, 230.0);
    motor_set_pwm((uint16_t)pid_out);
}
```



## 线程安全

lw_pid原生线程不安全，所有针对同一pid对象的操作，用户需要自行做好中断管理与并发访问保护。



## 代码安全

lw_pid在每一次运算时都会检查输入变量是否为NAN，NAN是浮点数的一个无效值，多为未初始化变量或数值溢出导致，所有由NAN计算得到的浮点数也都会被传染为NAN，最终造成pid控制错乱。当lw_pid发现异常输入，程序将会停止在运算函数内部，方便用户排错。
