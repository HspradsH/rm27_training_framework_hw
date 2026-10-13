10.6
配置板子

10.7
PID
首次编译
PWM
CAN
USART

10.8
LED

10.9
BMI088

10.10
DJIMotorHandler
rpm 转/分钟

10.11
threads
imu control alive

app_azure_rtos_config.h 里修改池子大小

信号量

extern 函数 公有

extern "C" 表示这个函数按 C 编译。

全局变量 表示声明，不定义

static
函数 私有，只能在当前文件中使用。

tx_semaphore_put 放入
tx_semaphore_get 获取

神秘错误
Error: [stm32f4x.cpu] Examination failed
Error: Target not examined yet
第二天换了全套设备就好了

10061 connection refused
可能需要改成绝对路径

10.12
gyro 出错。
和 acc 一样需要把 data 的第一位扔掉

加热了很久发现是没有开 PWM
仍然不行。

10.13
CAN
ESR 16253015 MSR 3080 TSR 2181038080

ESR 0xF80057 TSR 0x82000000 MSR = 0xC08