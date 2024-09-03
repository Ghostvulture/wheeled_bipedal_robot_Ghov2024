# BoardConnectivity

BoardConnectivity，字如其名，用于开发板之间的通讯。现在的整车代码中，云台C板会与底盘C板进行通讯。

云台C板向地盘C板发送地盘控制控制信息，x轴方向速度，y轴方向速度，角速度；

地盘C板向云台C板发送裁判系统的部分信息。

考虑到以后可能需要有更复杂的决策，可能需要发送完整的裁判系统信息，可以考虑单独使用串口直接向云台NUC发送或者改变主控位置（需要看规则）。

使用方法/示例：

```c++
/*--------------------发送消息--------------------*/
#include 'BoardConnectivity.hpp'

//将需要发送的消息加入BoardMemory
BoardMsg Data_send;
Data_send.isUsed = true;
Data_send.id = 0xB1; // id范围是0xB1-0xB8
Data_send.len = 8; // 目前只能设置成8。
Data_send.data[0] = 1;
... // 将八位数组逐一填充
//将数据添加到BoardMsg中
BoardConnectivity_Add2Memory(&Data_send);

...
//在主循环中
BoardConnectivity_Send(); 

// 接受数据需要在在回调函数中处理
if (rx_header.StdId >= 0xB0 && rx_header.StdId <= 0xAF) // 板件通讯信息处理
{
    // 相关处理逻辑
}
```

