# GPIO

开关GPIO口的函数本身很简单，所以大部分都是形式上的封装。最主要的是中断处理函数。`HAL_GPIO_EXTI_Callback`目前用于处理IST8310磁场数据。