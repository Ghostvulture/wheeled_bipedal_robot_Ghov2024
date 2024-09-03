# Buzzer

蜂鸣器主要用于提升机器人故障，比如BMI088初始化报错，电机离线等等。

使用示例：

```c++
#include "buzzer.h"

Buzzer_init();
// 蜂鸣器一直响
Buzzer_Start(DoFreq, 0.5)
// 关闭蜂鸣器
Buzzer_Stop();
```

