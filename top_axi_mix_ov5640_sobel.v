
//****************************************************************************************//

module top_axi_ov5640_sobel_hdmi(    
    input                 sys_clk        ,  //系统时钟
    input                 sys_rst_n      ,  //系统复位，低电平有效
    //摄像夿1接口                       
    input                 cam_pclk_1     ,  //cmos 数据像素时钟
    input                 cam_vsync_1    ,  //cmos 场同步信叿
    input                 cam_href_1     ,  //cmos 行同步信叿
    input   [7:0]         cam_data_1     ,  //cmos 数据
    output                cam_rst_n_1    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn_1 ,      //电源休眠模式选择 0：正常模弿 1：电源休眠模弿
    output                cam_scl_1      ,  //cmos SCCB_SCL线
    inout                 cam_sda_1      ,  //cmos SCCB_SDA线
    //摄像夿2接口     
    input                 cam_pclk_2     ,  //cmos 数据像素时钟
    input                 cam_vsync_2    ,  //cmos 场同步信叿
    input                 cam_href_2     ,  //cmos 行同步信叿
    input   [7:0]         cam_data_2     ,  //cmos 数据
    output                cam_rst_n_2    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn_2     ,  //电源休眠模式选择 0：正常模弿 1：电源休眠模弿
    output                cam_scl_2      ,  //cmos SCCB_SCL线
    inout                 cam_sda_2      ,  //cmos SCCB_SDA线   
       
    // DDR3                            
    inout   [31:0]        ddr3_dq        ,   //ddr3 数据
    inout   [3:0]         ddr3_dqs_n     ,   //ddr3 dqs贿
    inout   [3:0]         ddr3_dqs_p     ,   //ddr3 dqs歿  
    output  [13:0]        ddr3_addr      ,   //ddr3 地址   
    output  [2:0]         ddr3_ba        ,   //ddr3 banck 选择
    output                ddr3_ras_n     ,   //ddr3 行鿉择
    output                ddr3_cas_n     ,   //ddr3 列鿉择
    output                ddr3_we_n      ,   //ddr3 读写选择
    output                ddr3_reset_n   ,   //ddr3 复位
    output  [0:0]         ddr3_ck_p      ,   //ddr3 时钟歿
    output  [0:0]         ddr3_ck_n      ,   //ddr3 时钟贿
    output  [0:0]         ddr3_cke       ,   //ddr3 时钟使能
    output  [0:0]         ddr3_cs_n      ,   //ddr3 片鿿
    output  [3:0]         ddr3_dm        ,   //ddr3_dm
    output  [0:0]         ddr3_odt       ,   //ddr3_odt  								   
    //hdmi接口                           
    output                tmds_clk_p     ,  // TMDS 时钟通道
    output                tmds_clk_n     ,
    output  [2:0]         tmds_data_p    ,  // TMDS 数据通道
    output  [2:0]         tmds_data_n    
    );                                 

parameter  V_CMOS_DISP = 11'd768;                  //CMOS分辨玿--衿
parameter  H_CMOS_DISP = 11'd1024;                 //CMOS分辨玿--刿	
parameter  TOTAL_H_PIXEL = H_CMOS_DISP + 12'd1216; //CMOS分辨玿--衿
parameter  TOTAL_V_PIXEL = V_CMOS_DISP + 12'd504;      								   
							   
//wire define                          
wire         clk_50m                   ;  //50mhz时钟
wire         locked                    ;  //时钟锁定信号
wire         rst_n                     ;  //全局复位 								    						    
wire         wr_en                     ;  //DDR3控制器模块写使能
wire         rdata_req                 ;  //DDR3控制器模块读使能
wire  [15:0] rd_data                   ;  //DDR3控制器模块读数据
wire         cmos_frame_valid_1        ;  //数据1有效使能信号
wire  [15:0] wr_data_1                 ;  //DDR3控制器模块写数据1
wire         cmos_frame_valid_2        ;  //数据2有效使能信号
wire  [15:0] wr_data_2                 ;  //DDR3控制器模块写数据2
wire         init_calib_complete       ;  //DDR3初始化完成init_calib_complete
wire         sys_init_done             ;  //系统初始化完房(DDR初始匿+摄像头初始化)
wire         clk_200m                  ;  //ddr3参迃时钿
wire         cmos_frame_vsync_1        ;  //输出帿1有效场同步信叿
wire         cmos_frame_vsync_2        ;  //输出帿2有效场同步信叿
wire         cmos_frame_href_1         ;  //输出帧有效行同步信号 
wire         cmos_frame_href_2         ;  //输出帧有效行同步信号 
wire  [10:0] pixel_xpos_w              ;
wire  [10:0] pixel_ypos_w              ;
wire  [12:0] h_disp                    ;  //LCD屏水平分辨率
wire  [12:0] v_disp                    ;  //LCD屏垂直分辨率   
wire  [15:0] post_rgb_1                ;  //处理后的图像数据
wire         post_frame_vsync_1        ;  //处理后的场信叿
wire         post_frame_de_1           ;  //处理后的数据有效使能 
wire  [15:0] post_rgb_2                ;  //处理后的图像数据
wire         post_frame_vsync_2        ;  //处理后的场信叿
wire         post_frame_de_2           ;  //处理后的数据有效使能 
wire         rd_vsync                  ;
// 定义DDR3地址朿大忿
wire [27:0] ddr3_addr_max              ;
wire         pingpang                  ;//乒乓操作                      
wire         datain_valid              ;  //数据有效使能信号
wire         rd_load                   ; // RD FIFO加载信号
wire         wr_load                   ;  // WR FIFO加载信号 

wire         rd_valid                ; //读FIFO有效标志
wire         ui_clk                    ;
wire         ui_rst                    ;
//*****************************************************
//**                    main code
//*****************************************************

//*****************************************************
//**                    main code
//*****************************************************

//待时钟锁定后产生复位结束信号
assign  rst_n = sys_rst_n & locked;

//系统初始化完成：DDR3初始化完房
assign  sys_init_done = init_calib_complete;

//存入DDR3的最大读写地坿 
assign  ddr3_addr_max =  V_CMOS_DISP*H_CMOS_DISP; 
   
 //ov5640 驱动
ov5640_dri u_ov5640_dri_1(
    .clk               (clk_50m),
    .rst_n             (rst_n),

    .cam_pclk          (cam_pclk_1),
    .cam_vsync         (cam_vsync_1),
    .cam_href          (cam_href_1 ),
    .cam_data          (cam_data_1 ),
    .cam_rst_n         (cam_rst_n_1),
    .cam_pwdn          (cam_pwdn_1),
    .cam_scl           (cam_scl_1  ),
    .cam_sda           (cam_sda_1  ),
    
    .capture_start     (init_calib_complete),
    //.cmos_h_pixel      (H_CMOS_DISP/2),
    .cmos_h_pixel      (H_CMOS_DISP),
    .cmos_v_pixel      (V_CMOS_DISP),
    .total_h_pixel     (TOTAL_H_PIXEL),
    .total_v_pixel     (TOTAL_V_PIXEL),
    .cmos_frame_vsync  (cmos_frame_vsync_1),
    .cmos_frame_href   (cmos_frame_href_1),
    .cmos_frame_valid  (cmos_frame_valid_1),
    .cmos_frame_data   (wr_data_1)
    );   
    
  //ov5640 驱动
ov5640_dri u_ov5640_dri_2(
    .clk               (clk_50m),
    .rst_n             (rst_n),

    .cam_pclk          (cam_pclk_2 ),
    .cam_vsync         (cam_vsync_2),
    .cam_href          (cam_href_2 ),
    .cam_data          (cam_data_2),
    .cam_rst_n         (cam_rst_n_2),
    .cam_pwdn          (cam_pwdn_2 ),
    .cam_scl           (cam_scl_2  ),
    .cam_sda           (cam_sda_2 ),
    
    .capture_start     (init_calib_complete),
    .cmos_h_pixel      (H_CMOS_DISP),
//    .cmos_h_pixel      (H_CMOS_DISP/2),
    .cmos_v_pixel      (V_CMOS_DISP),
    .total_h_pixel     (TOTAL_H_PIXEL),
    .total_v_pixel     (TOTAL_V_PIXEL),
    .cmos_frame_vsync  (cmos_frame_vsync_2),
    .cmos_frame_href   (cmos_frame_href_2),
    .cmos_frame_valid  (cmos_frame_valid_2),
    .cmos_frame_data   (wr_data_2)
    );    

 //图像处理模块
vip u_vip1(
    //module clock
    .clk              (cam_pclk_1),           // 时钟信号
    .rst_n            (rst_n    ),          // 复位信号（低有效＿
    //图像处理前的数据接口
    .pre_frame_vsync  (cmos_frame_vsync_1   ),
    .pre_frame_href   (cmos_frame_href_1   ),
    .pre_frame_de     (cmos_frame_valid_1   ),
    .pre_rgb          (wr_data_1),
    .xpos             (pixel_xpos_w   ),
    .ypos             (pixel_ypos_w   ),
    //图像处理后的数据接口
    .post_frame_vsync (post_frame_vsync_1 ),  // 场同步信叿
    .post_frame_href ( ),                  // 行同步信叿
    .post_frame_de    (post_frame_de_1 ),     // 数据输入使能
    .post_rgb         (post_rgb_1)            // RGB565颜色数据

);   
/* 
 //图像处理模块
vip u_vip2(
    //module clock
    .clk              (cam_pclk_2),           // 时钟信号
    .rst_n            (rst_n    ),          // 复位信号（低有效＿
    //图像处理前的数据接口
    .pre_frame_vsync  (cmos_frame_vsync_2   ),
    .pre_frame_href   (cmos_frame_href_2   ),
    .pre_frame_de     (cmos_frame_valid_2   ),
    .pre_rgb          (wr_data_2),
    .xpos             (pixel_xpos_w   ),
    .ypos             (pixel_ypos_w   ),
    //图像处理后的数据接口
    .post_frame_vsync (post_frame_vsync_2 ),  // 场同步信叿
    .post_frame_href ( ),                  // 行同步信叿
    .post_frame_de    (post_frame_de_2 ),     // 数据输入使能
    .post_rgb         (post_rgb_2)            // RGB565颜色数据

);   
  */   
ddr_interface #(
    .FIFO_WR_WIDTH(256),   // 用户端FIFO读写位宽
    .FIFO_RD_WIDTH(256),
    .AXI_WIDTH(256),       // AXI总线读写数据位宽
    .AXI_AXSIZE(3'b101)    // AXI总线的axi_awsize, 霿要与AXI_WIDTH对应
) u_ddr_interface (
    .clk(clk_200m),                        // DDR3时钟, 也就是DDR3 MIG IP核参考时钿
    .rst_n(rst_n),                    // 全局复位信号
    .pingpang(1'b1),              // 乒乓操作
    .datain_valid_1(post_frame_de_1),  // 数据有效使能信号 1
    .datain_valid_2(cmos_frame_valid_2),  // 数据有效使能信号 2
    .rd_load_1(rd_vsync),                // RD FIFO加载信号
    .rd_load_2(rd_vsync),    
    .wr_load_1(post_frame_vsync_1),            // WR FIFO加载信号 1
    .wr_load_2(cmos_frame_vsync_2),            // WR FIFO加载信号 2
    .datain_1(post_rgb_1),              // 输入数据 1
    .datain_2(wr_data_2),              // 输入数据 2
    .wr_clk_1(cam_pclk_1),              // 写FIFO写时钿 1
    .wr_rst_1(1'b0),              // 写复使 1
    .wr_clk_2(cam_pclk_2),              // 写FIFO写时钿 1
    .wr_rst_2(1'b0),              // 写复使 1
    .wr_beg_addr_1(29'd0                ),    // 写起始地坿 1
    .wr_end_addr_1(ddr3_addr_max[27:0]  ),    // 写终止地坿 1
    .wr_beg_addr_2(ddr3_addr_max[27:0]*5),    // 写起始地坿 2
    .wr_end_addr_2(ddr3_addr_max[27:0]*6),    // 写终止地坿 2
    .wr_burst_len (H_CMOS_DISP[10:4]     ),      // 写突发长庿
    //.wr_en_1(wr_en_1),                // 写FIFO写请汿 1
    //.wr_data_1(wr_data_1),            // 写FIFO写数捿 1
   // .wr_en_2(wr_en_2),                // 写FIFO写请汿 2
   // .wr_data_2(wr_data_2),            // 写FIFO写数捿 2
    .rd_clk_1(pixel_clk),                  // 读FIFO读时钿
    .rd_rst_1(1'b0),                  // 读复使
    .rd_clk_2(pixel_clk),                  // 读FIFO读时钿
    .rd_rst_2(1'b0),                  // 读复使
    .rd_mem_enable(1'b1                 ),    // 读存储器使能
    .rd_beg_addr_1(29'd0                ),    // 读起始地坿 1
    .rd_end_addr_1(ddr3_addr_max[27:0]  ),    // 读终止地坿 1
    .rd_beg_addr_2(ddr3_addr_max[27:0]*5),    // 读起始地坿 2
    .rd_end_addr_2(ddr3_addr_max[27:0]*6),    // 读终止地坿 2
    .rd_burst_len (H_CMOS_DISP[10:4]    ),      // 读突发长庿
    .rd_en(rdata_req),                // 读FIFO读请汿 1
    //.rd_en_2(rd_en_2),                // 读FIFO读请汿 2
    .rd_valid(rd_valid),          // 读FIFO有效标志 2
    .ui_clk(ui_clk),                  // MIG IP核输出的用户时钟
    .ui_rst(ui_rst),                  // MIG IP核输出的复位信号
    .calib_done(init_calib_complete),          // DDR3初始化完成标忿
    .pic_data(rd_data),          // 输出有效数据 
    .h_disp  (h_disp),          //HDMI屏水平分辨率
    
    .ddr3_addr(ddr3_addr),            // DDR3 地址
    .ddr3_ba(ddr3_ba),                // DDR3 Bank地址
    .ddr3_cas_n(ddr3_cas_n),          // DDR3 CAS信号
    .ddr3_ck_n(ddr3_ck_n),            // DDR3 CK信号（负极）
    .ddr3_ck_p(ddr3_ck_p),            // DDR3 CK信号（正极）
    .ddr3_cke(ddr3_cke),              // DDR3 CKE信号
    .ddr3_ras_n(ddr3_ras_n),          // DDR3 RAS信号
    .ddr3_reset_n(ddr3_reset_n),      // DDR3复位信号
    .ddr3_we_n(ddr3_we_n),            // DDR3写使能信叿
    .ddr3_dq(ddr3_dq),                // DDR3数据总线
    .ddr3_dqs_n(ddr3_dqs_n),          // DDR3数据选鿚信号（负极＿
    .ddr3_dqs_p(ddr3_dqs_p),          // DDR3数据选鿚信号（正极＿
    .ddr3_cs_n(ddr3_cs_n),            // DDR3片鿉信叿
    .ddr3_dm(ddr3_dm),                // DDR3数据掩码
    .ddr3_odt(ddr3_odt)               // DDR3终端电阻
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
    .video_vs             (rd_vsync     ),   //HDMI场信叿  
    .h_disp               (h_disp),          //HDMI屏水平分辨率
    .v_disp               (v_disp),          //HDMI屏垂直分辨率   
    .pixel_xpos           (pixel_xpos_w),
    .pixel_ypos           (pixel_ypos_w),      
    .data_in              (rd_data),         //数据输入 
    .data_req             (rdata_req)        //请求数据输入   
);  
    
endmodule