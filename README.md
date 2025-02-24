# dual-ov5640-sobel-eth-hdmi
项目简介：
设计并开发了一个高效的FPGA系统，集成了图像采集、处理和显示功能。该系统使用双目OV5640摄像头实时捕获图像，并设计Sobel边缘检测对图像进行处理，同时通过AXI4协议与DDR3进行数据交互，最终一路数据通过千兆以太网传给上位机，另一路数据在 HDMI 上进行实时显示。
技术栈与功能:
（1）通过 I2C（SCCB）配置双目OV5640摄像头工作模式和分辨率，实现双目摄像头同步采集并输出 RGB565 等格式；

（2）对采集到的图像进行滤波、Sobel 边缘检测等算法处理，同时对数据进行拼接和拆分，结合行场同步时序，保证实时性与准确性；

（3）设计并实现AXI4通道接口，满足帧缓存机制，采用 Xilinx MIG IP 搭建 DDR3 控制器，支持多路图像数据的突发读写，实现高效的数据传输与缓存调度。

（4）设计并实现UDP收发模块，采用RGMII2GMII原语，实现TMDS 协议完成 HDMI 输出，并支持任意分辨率；

（5）对子模块进行仿真验证，ILA调试，Wireshark抓包测试，并最终上板验证。
