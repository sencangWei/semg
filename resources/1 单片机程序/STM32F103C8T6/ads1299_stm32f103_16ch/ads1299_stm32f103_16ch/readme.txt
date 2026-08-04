接线说明
STM32F103C8T6与CX-M1299模块接线
STM32     CX-M1299(第1片)     CX-M1299引脚说明
3.3V         VIN                         电源正极    
GND        GND                       电源地
                P_EN                      电源使能,模块内部默认使能，可不接到单片机IO
A0           DRDY                      数据中断引脚，MCU需设置下降沿触发中断
A6           MISO                      SPI主入从出引脚
               DAISY                     菊花链输入引脚 
A5           SCLK                       SPI时钟引脚
A1           START                     启动ADC转化引脚，拉高启动，拉低停止
A2           RESET                     复位引脚
A3           PWDN                    ADS1299电源控制引脚，拉低芯片关机
A7           MOSI                     SPI主出从入引脚
              CLK                         时钟引脚
A4          CS                           SPI片选引脚 


STM32     CX-M1299(第2片)     CX-M1299引脚说明
3.3V         VIN                         电源正极    
GND        GND                       电源地
                P_EN                      电源使能,模块内部默认使能，可不接到单片机IO
               DRDY                      数据中断引脚，MCU需设置下降沿触发中断
A6           MISO                      SPI主入从出引脚
               DAISY                     菊花链输入引脚 
A5           SCLK                       SPI时钟引脚
A1           START                     启动ADC转化引脚，拉高启动，拉低停止
A2           RESET                     复位引脚
A3           PWDN                    ADS1299电源控制引脚，拉低芯片关机
A7           MOSI                     SPI主出从入引脚
               CLK                         时钟引脚
B0           CS                           SPI片选引脚 

注：两片的1299模块的CLK引脚连接在一起，即使用相同的时钟

STM32F103C8T6与CX-HC3_H/S WIFI模块接线
STM32       CX-M1299    WIFI模块引脚说明
GND         GND
A10(RX)     TX
A9(TX)       RX
5V             VCC




