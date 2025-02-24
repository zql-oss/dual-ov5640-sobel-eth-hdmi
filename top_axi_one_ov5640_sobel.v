

module top_axi_one_ov5640_hdmi_sobel(    
    input                 sys_clk      ,  //系统时钟
    input                 sys_rst_n    ,  //系统复位，低电平有效
    //摄像头接口                       
    input                 cam_pclk     ,  //cmos 数据像素时钟
    input                 cam_vsync    ,  //cmos 场同步信号
    input                 cam_href     ,  //cmos 行同步信号
    input   [7:0]         cam_data     ,  //cmos 数据
    output                cam_rst_n    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn     ,  //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl      ,  //cmos SCCB_SCL线
    inout                 cam_sda      ,  //cmos SCCB_SDA线       
    // DDR3                            
    inout   [31:0]        ddr3_dq      ,  //DDR3 数据
    inout   [3:0]         ddr3_dqs_n   ,  //DDR3 dqs负
    inout   [3:0]         ddr3_dqs_p   ,  //DDR3 dqs正  
    output  [13:0]        ddr3_addr    ,  //DDR3 地址   
    output  [2:0]         ddr3_ba      ,  //DDR3 banck 选择
    output                ddr3_ras_n   ,  //DDR3 行选择
    output                ddr3_cas_n   ,  //DDR3 列选择
    output                ddr3_we_n    ,  //DDR3 读写选择
    output                ddr3_reset_n ,  //DDR3 复位
    output  [0:0]         ddr3_ck_p    ,  //DDR3 时钟正
    output  [0:0]         ddr3_ck_n    ,  //DDR3 时钟负
    output  [0:0]         ddr3_cke     ,  //DDR3 时钟使能
    output  [0:0]         ddr3_cs_n    ,  //DDR3 片选
    output  [3:0]         ddr3_dm      ,  //DDR3_dm
    output  [0:0]         ddr3_odt     ,  //DDR3_odt									                            
    //hdmi接口                           
    output                tmds_clk_p   ,  // TMDS 时钟通道
    output                tmds_clk_n   ,
    output  [2:0]         tmds_data_p  ,  // TMDS 数据通道
    output  [2:0]         tmds_data_n  
    );     
                                
parameter  V_CMOS_DISP = 11'd768;                  //CMOS分辨率--行
parameter  H_CMOS_DISP = 11'd1024;                 //CMOS分辨率--列	
parameter  TOTAL_H_PIXEL = H_CMOS_DISP + 12'd1216; //CMOS分辨率--行
parameter  TOTAL_V_PIXEL = V_CMOS_DISP + 12'd504;    										   
							   
//wire define                          
wire         clk_50m                   ;  //50mhz时钟,提供给lcd驱动时钟
wire         locked                    ;  //时钟锁定信号
wire         rst_n                     ;  //全局复位 								            
wire         cam_init_done             ;  //摄像头初始化完成						    
wire         wr_en                     ;  //DDR3控制器模块写使能
wire  [15:0] wr_data                   ;  //DDR3控制器模块写数据
wire         rdata_req                 ;  //DDR3控制器模块读使能
wire  [15:0] rd_data                   ;  //DDR3控制器模块读数据
wire         cmos_frame_valid          ;  //数据有效使能信号
wire         init_calib_complete       ;  //DDR3初始化完成init_calib_complete
wire         sys_init_done             ;  //系统初始化完成(DDR初始化+摄像头初始化)
wire         clk_200m                  ;  //ddr3参考时钟
wire         cmos_frame_vsync          ;  //输出帧有效场同步信号
wire         cmos_frame_href           ;  //输出帧有效行同步信号  
wire  [12:0] h_disp                    ;  //LCD屏水平分辨率
wire  [12:0] v_disp                    ;  //LCD屏垂直分辨率     
wire  [27:0] ddr3_addr_max             ;  //存入DDR3的最大读写地址 
wire  [2:0]  tmds_data_p               ;  // TMDS 数据通道
wire  [2:0]  tmds_data_n               ;
wire  [10:0] pixel_xpos_w              ;
wire  [10:0] pixel_ypos_w              ;
wire         post_frame_vsync          ;
wire         post_frame_hsync          ;
wire         post_frame_de             ;    
wire  [15:0] post_rgb                  ;

wire         pingpang                  ;//乒乓操作                      
wire         datain_valid              ;  //数据有效使能信号
wire         rd_load                   ; // RD FIFO加载信号
wire         wr_load                   ;  // WR FIFO加载信号 
wire  [15:0] datain                    ;  //有效数据 
wire         rd_valid                  ; //读FIFO有效标志
wire         ui_clk                    ;
wire         ui_rst                    ;
//*****************************************************
//**                    main code
//*****************************************************

//待时钟锁定后产生复位结束信号
assign  rst_n = sys_rst_n & locked;

//系统初始化完成：DDR3初始化完成
assign  sys_init_done = init_calib_complete;

//存入DDR3的最大读写地址 
assign  ddr3_addr_max =  V_CMOS_DISP*H_CMOS_DISP; 

 //ov5640 驱动
ov5640_dri u_ov5640_dri(
    .clk               (clk_50m),
    .rst_n             (rst_n),

    .cam_pclk          (cam_pclk ),
    .cam_vsync         (cam_vsync),
    .cam_href          (cam_href ),
    .cam_data          (cam_data ),
    .cam_rst_n         (cam_rst_n),
    .cam_pwdn          (cam_pwdn ),
    .cam_scl           (cam_scl  ),
    .cam_sda           (cam_sda  ),
    
    .capture_start     (init_calib_complete),
    .cmos_h_pixel      (H_CMOS_DISP),
    .cmos_v_pixel      (V_CMOS_DISP),
    .total_h_pixel     (TOTAL_H_PIXEL),
    .total_v_pixel     (TOTAL_V_PIXEL),
    .cmos_frame_vsync  (cmos_frame_vsync),
    .cmos_frame_href   (cmos_frame_href),
    .cmos_frame_valid  (cmos_frame_valid),
    .cmos_frame_data   (wr_data)
    );  
    
 //图像处理模块
vip u_vip(
    //module clock
    .clk              (cam_pclk),           // 时钟信号
    .rst_n            (rst_n    ),          // 复位信号（低有效）
    //图像处理前的数据接口
    .pre_frame_vsync  (cmos_frame_vsync   ),
    .pre_frame_href   (cmos_frame_href   ),
    .pre_frame_de     (cmos_frame_valid   ),
    .pre_rgb          (wr_data),
    .xpos             (pixel_xpos_w   ),
    .ypos             (pixel_ypos_w   ),
    //图像处理后的数据接口
    .post_frame_vsync (post_frame_vsync ),  // 场同步信号
    .post_frame_href  ( ),                  // 行同步信号
    .post_frame_de    (post_frame_de ),     // 数据输入使能
    .post_rgb         (post_rgb)            // RGB565颜色数据

);      
            
//DDR3控制接口
    ddr_interface
    #(.FIFO_WR_WIDTH(256),  //用户端FIFO读写位宽
      .FIFO_RD_WIDTH(256),
      .AXI_WIDTH    (256),  //AXI总线读写数据位宽
      .AXI_AXSIZE   (3'b101)   //AXI总线的axi_awsize, 需要与AXI_WIDTH对应
      )
      ddr_interface_inst
        (
        .clk                 (clk_200m                          ), //DDR3时钟, 也就是DDR3 MIG IP核参考时钟
        .rst_n               (rst_n                             ), //模块内部会进行异步复位、同步释放处理
        .pingpang            (1'b1                              ),
        .datain_valid        (post_frame_de                     ),
        .rd_load             (rd_vsync                          ),
        .wr_load             (post_frame_vsync                  ),
        .datain              (post_rgb                          ),
        .pic_data            (rd_data                           ),
   
        //用户端                       
        .wr_clk              (cam_pclk                          ), //写FIFO写时钟
        .wr_rst              (1'b0                              ), //写复位
        .wr_beg_addr         (29'd0                             ), //写起始地址
        .wr_end_addr         (ddr3_addr_max[27:0]               ), //写终止地址
        .wr_burst_len        (H_CMOS_DISP[10:4]                 ), //写突发长度
        .rd_clk              (pixel_clk                         ), //读FIFO读时钟
        .rd_rst              (1'b0                              ), //读复位, 没有开始使能读时,读地址处于0
        .rd_mem_enable       (1'b1                              ), //读存储器使能,防止存储器未写先读
        .rd_beg_addr         (29'd0                             ), //读起始地址
        .rd_end_addr         (ddr3_addr_max[27:0]               ), //读终止地址
        .rd_burst_len        (H_CMOS_DISP[10:4]                 ), //读突发长度
        .rd_en               (rdata_req                         ), //读FIFO读请求
        //.rd_data             (), //读FIFO读数据
        .rd_valid            (rd_valid                          ), //读FIFO有效标志
        .ui_clk              (ui_clk                            ), //MIG IP核输出的用户时钟, 用作AXI控制器时钟
        .ui_rst              (ui_rst                            ), //MIG IP核输出的复位信号, 高电平有效
        .calib_done          (init_calib_complete               ), //DDR3初始化完成
        
        //DDR3接口                              
        .ddr3_addr           (ddr3_addr           ),  
        .ddr3_ba             (ddr3_ba             ),
        .ddr3_cas_n          (ddr3_cas_n          ),
        .ddr3_ck_n           (ddr3_ck_n           ),
        .ddr3_ck_p           (ddr3_ck_p           ),
        .ddr3_cke            (ddr3_cke            ),
        .ddr3_ras_n          (ddr3_ras_n          ),
        .ddr3_reset_n        (ddr3_reset_n        ),
        .ddr3_we_n           (ddr3_we_n           ),
        .ddr3_dq             (ddr3_dq             ),
        .ddr3_dqs_n          (ddr3_dqs_n          ),
        .ddr3_dqs_p          (ddr3_dqs_p          ),
        .ddr3_cs_n           (ddr3_cs_n           ),
        .ddr3_dm             (ddr3_dm             ),
        .ddr3_odt            (ddr3_odt            )
        
    );
 clk_wiz_0 u_clk_wiz_0
   (
    // Clock out ports
    .clk_out1              (clk_200m),     
    .clk_out2              (clk_50m),
    .clk_out3              (pixel_clk_5x),
    .clk_out4              (pixel_clk),
    // Status and control signals
    .reset                 (1'b0), 
    .locked                (locked),       
   // Clock in ports
    .clk_in1               (sys_clk)
    );     
 
//HDMI驱动显示模块    
hdmi_top u_hdmi_top(
    .pixel_clk            (pixel_clk),
    .pixel_clk_5x         (pixel_clk_5x),    
    .sys_rst_n            (sys_init_done & rst_n),
    //hdmi接口                   
    .tmds_clk_p           (tmds_clk_p   ),   // TMDS 时钟通道
    .tmds_clk_n           (tmds_clk_n   ),
    .tmds_data_p          (tmds_data_p  ),   // TMDS 数据通道
    .tmds_data_n          (tmds_data_n  ),
    //用户接口 
    .video_vs             (rd_vsync     ),   //HDMI场信号  
    .h_disp               (h_disp),          //HDMI屏水平分辨率
    .v_disp               (v_disp),          //HDMI屏垂直分辨率   
    .pixel_xpos           (pixel_xpos_w),
    .pixel_ypos           (pixel_ypos_w),      
    .data_in              (rd_data),         //数据输入 
    .data_req             (rdata_req)        //请求数据输入   
);   

endmodule