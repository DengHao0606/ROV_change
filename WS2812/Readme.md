# 【WS2812】- 串行控制全彩色LED


## WS2812 简介

WS2812是一种集成了RGB LED和控制电路的智能型LED灯珠。它们以串联方式连接，能够通过单一信号线进行控制和通讯。与普通的RGB LED相比：

- WS2812内部集成了驱动电路，只需要为它提供电源即可，而无需三极管、限流电阻
- WS2812内部集成了控制电路，只需要发送R、G、B各自的亮度（0-255）即可实现亮度、色彩的控制
- WS2812支持串联通信，STM32只需要连接第一个WS2812，信号便可以传递至所有WS2812
- WS2812的RGB三色亮度经过匹配，只需要发送对应的RGB值，便可以混合出想要的颜色

#### 十六进制颜色表示

WS2812 的颜色通过RGB三通道混合产生，每个通道的调节范围都是 0 - 255，也就是十六进制的 00 - FF

因此，将R、G、B各自的亮度都用十六进制表示，便可以得到三个十六进制数，例如：白色FF、FF、FF，红色FF、00、00，绿色00、FF、00

将这三个十六进制数按顺序合并起来，就是 FFFFFF、FF0000、00FF00，这就是十六进制颜色表示法

- 获取更多颜色：https://www.w3schools.com/colors/colors_picker.asp

<img src="Doc/取色.png" style="zoom:50%;" />

## 如何使用例程

下载程序，即可看到效果

### 程序效果

- 烧录例程后，即可看到板子背面的WS2812全部点亮，并呈现色彩过渡效果和彩虹渐变效果

<img src="Doc/效果演示.gif" style="zoom:50%;" />


## 例程讲解

下面介绍了如何自己实现该例程的功能

### 1、工程配置

- **开启外部晶振：**在Pinout&Configuration -> System Core -> RCC 页面，将 High Speed Clock (HSE) 配置为 Crystal/Ceramic Resonator

<img src="Doc/配置时钟源.png" style="zoom:65%;" />

- **配置时钟频率：**在Clock Configuration 页面，将PLL Source 选择为 HSE，将System Clock Mux 选择为 PLLCLK，然后在HCLK (MHz) 输入72并回车，将HCLK频率配置为 72 MHz

<img src="Doc/时钟配置.png" style="zoom:55%;" />

- **配置DEBUG方式：**由于PB4引脚默认分配给JTAG调试接口，所以需要先配置DUBUG方式为Serial Wire，以释放PB4引脚。在Pinout&Configuration -> System Core -> SYS 页面，将Debug设置为 Serial Wire

- **配置生成单独.c/.h文件：**在Project Manager -> Code Generator页面中，勾选Generate peripheral initialization as ... per peripheral

- **分配引脚：**在Pinout&Configuration页面，将PB4配置为TIM3_CH1

- **配置TIM3：**在Pinout&Configuration -> Timers -> TIM3

  - 勾选 Internal Clock，开启 TIM3 的内部时钟源

  - Configuration -> Mode，将 Channel1 配置为 PWM Generation CH1

  - Configuration -> Parameter Settings -> Counter Settings，将 Prescaler 配置为 0，Counter Period配置为90-1，即PWM频率 800 kHz

  - Configuration -> GPIO Settings，将PB4的GPIO mode配置为 Alternate Function Open Drain，Maximum output speed 配置为 High

    > 因为WS2812是5V电平的，因此需要用开漏输出模式，外接5V上拉电阻，使信号高电平变成5V
    >
    > 注意，PB4是 5V Tolerance PIN，可以容忍 5V 上拉。普通引脚不可以。

  - Configuration -> DMA Settings，点击Add新增一个通道

    - DMA Request 选择 TIM3_CH1/TRIG
    - Direction 选择 Memory To Peripheral

<img src="Doc/DMA配置.png" style="zoom:70%;" />

### 2、代码

例程提供了 WS2812 驱动库，可以直接调用库函数实现颜色转换和渐变效果

#### (1) 初始化过程

- **拷贝库文件：**将 ws2812.c 文件拷贝到 Core -> Src 目录下，将 ws2812.h 文件拷贝到 Core -> Inc 目录下。
- **添加头文件：**在 main.c 中引用头文件

```c
#include "ws2812.h"
```

#### (2) 库功能

驱动库会创建一个 `ws2812_color[ ]` 数组，设置LED颜色时只会更改此数组，而不会直接操作LED

要使 LED 显示数组中的颜色，需调用 `ws2812_update` 或 `ws2812_gradient` 将数组更新到LED

- **更新LED颜色**

  - 将颜色数组直接更新到 LED，不使用渐变过渡
  - `ws2812_update()`

- **渐变的更新LED颜色**

  - 通过线性插值方式，从当前颜色渐变到新颜色
  - `ws2812_gradient(uint8_t steps, uint16_t delay_ms)`
    - `steps` 过渡步数，范围 1-255
    - `delay_ms` 每一步之间的延时长度，单位 毫秒

- **设置LED颜色（十六进制颜色格式）**

  - 设置某个LED的颜色

  - `ws2812_set(uint8_t led_id, uint32_t color)` 

    - `led_id` LED编号，范围 0-9，分别对应学习板背面的10个WS2812

    - `color` 目标颜色，输入十六进制颜色格式，范围 0x000000 - 0xFFFFFF

      > 参考文档开头的十六进制颜色介绍
  
- **设置LED颜色（R、G、B 格式）**

  - 设置某个LED的颜色
  
  - `ws2812_set(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b)` 
    - `led_id` LED编号，范围 0-9，分别对应学习板背面的10个WS2812
    - `r` 红色亮度，范围 0 - 255
    - `g` 绿色亮度，范围 0 - 255
    - `b` 蓝色亮度，范围 0 - 255
  
- **设置所有LED的颜色（十六进制颜色格式）**

  - 设置所有（学习板共有 10 个）LED的颜色
  - `ws2812_set_all(uint32_t color)` 
    - `color` 目标颜色，输入十六进制颜色格式，范围 0x000000 - 0xFFFFFF
  
- **将RGB颜色转换为十六进制颜色**

  - `rgb_to_color(uint8_t r, uint8_t g, uint8_t b)`
    - `r` 红色亮度，范围 0 - 255
    - `g` 绿色亮度，范围 0 - 255
    - `b` 蓝色亮度，范围 0 - 255
    - `return` 返回值为转换后的十六进制颜色，变量类型 uint32_t

- **将十六进制颜色转换为RGB值**

  - `color_to_rgb(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b)`
    - `color` 十六进制颜色（输入）
    - `r` 红色亮度（输出），需传入指针
    - `g` 绿色亮度（输出），需传入指针
    - `b` 蓝色亮度（输出），需传入指针

- **彩虹效果**

  - `rainbow_effect(uint8_t steps, uint16_t delay_ms)` 

  - `steps` 过渡步数，范围 0 - 255

  - `delay_ms` 色彩流动间隔，单位 毫秒

    > 建议参数： `rainbow_effect(255, 70)`

