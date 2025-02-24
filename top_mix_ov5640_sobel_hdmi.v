

module top_dual_ov5640_sobel_hdmi(    
    input                 sys_clk        ,  //系统时钟
    input                 sys_rst_n      ,  //系统复位，低电平有效
    //摄像头1接口                       
    input                 cam_pclk_1     ,  //cmos 数据像素时钟
    input                 cam_vsync_1    ,  //cmos 场同步信号
    input                 cam_href_1     ,  //cmos 行同步信号
    input   [7:0]         cam_data_1     ,  //cmos 数据
    output                cam_rst_n_1    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn_1 ,      //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl_1      ,  //cmos SCCB_SCL线
    inout                 cam_sda_1      ,  //cmos SCCB_SDA线
    //摄像头2接口     
    input                 cam_pclk_2     ,  //cmos 数据像素时钟
    input                 cam_vsync_2    ,  //cmos 场同步信号
    input                 cam_href_2     ,  //cmos 行同步信号
    input   [7:0]         cam_data_2     ,  //cmos 数据
    output                cam_rst_n_2    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn_2     ,  //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl_2      ,  //cmos SCCB_SCL线
    inout                 cam_sda_2      ,  //cmos SCCB_SDA线   
       
    // DDR3                            
    inout   [31:0]        ddr3_dq        ,   //ddr3 数据
    inout   [3:0]         ddr3_dqs_n     ,   //ddr3 dqs负
    inout   [3:0]         ddr3_dqs_p     ,   //ddr3 dqs正  
    output  [13:0]        ddr3_addr      ,   //ddr3 地址   
    output  [2:0]         ddr3_ba        ,   //ddr3 banck 选择
    output                ddr3_ras_n     ,   //ddr3 行选择
    output                ddr3_cas_n     ,   //ddr3 列选择
    output                ddr3_we_n      ,   //ddr3 读写选择
    output                ddr3_reset_n   ,   //ddr3 复位
    output  [0:0]         ddr3_ck_p      ,   //ddr3 时钟正
    output  [0:0]         ddr3_ck_n      ,   //ddr3 时钟负
    output  [0:0]         ddr3_cke       ,   //ddr3 时钟使能
    output  [0:0]         ddr3_cs_n      ,   //ddr3 片选
    output  [3:0]         ddr3_dm        ,   //ddr3_dm
    output  [0:0]         ddr3_odt       ,   //ddr3_odt  								   
    //hdmi接口                           
    output                tmds_clk_p     ,  // TMDS 时钟通道
    output                tmds_clk_n     ,
    output  [2:0]         tmds_data_p    ,  // TMDS 数据通道
    output  [2:0]         tmds_data_n    
    );                                 

parameter  V_CMOS_DISP = 11'd768;                  //CMOS分辨率--行
parameter  H_CMOS_DISP = 11'd1024;                 //CMOS分辨率--列	
parameter  TOTAL_H_PIXEL = H_CMOS_DISP + 12'd1216; //CMOS分辨率--行
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
wire         sys_init_done             ;  //系统初始化完成(DDR初始化+摄像头初始化)
wire         clk_200m                  ;  //ddr3参考时钟
wire         cmos_frame_vsync_1        ;  //输出帧1有效场同步信号
wire         cmos_frame_vsync_2        ;  //输出帧2有效场同步信号
wire         cmos_frame_href_1         ;  //输出帧有效行同步信号 
wire         cmos_frame_href_2         ;  //输出帧有效行同步信号 
wire  [10:0] pixel_xpos_w              ;
wire  [10:0] pixel_ypos_w              ;
wire  [12:0] h_disp                    ;  //LCD屏水平分辨率
wire  [12:0] v_disp                    ;  //LCD屏垂直分辨率   
wire  [15:0] post_rgb_1                ;  //处理后的图像数据
wire         post_frame_vsync_1        ;  //处理后的场信号
wire         post_frame_de_1           ;  //处理后的数据有效使能 
wire  [15:0] post_rgb_2                ;  //处理后的图像数据
wire         post_frame_vsync_2        ;  //处理后的场信号
wire         post_frame_de_2           ;  //处理后的数据有效使能 
wire         rd_vsync                  ;
// 定义DDR3地址最大值
wire [27:0] ddr3_addr_max              ;
//*****************************************************
//**                    main code
//*****************************************************

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
    //.cmos_h_pixel      (H_CMOS_DISP),
    .cmos_h_pixel      (H_CMOS_DISP/2),
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
    .cmos_h_pixel      (H_CMOS_DISP/2),
//    .cmos_h_pixel      (H_CMOS_DISP),
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
    .rst_n            (rst_n    ),          // 复位信号（低有效）
    //图像处理前的数据接口
    .pre_frame_vsync  (cmos_frame_vsync_1   ),
    .pre_frame_href   (cmos_frame_href_1   ),
    .pre_frame_de     (cmos_frame_valid_1   ),
    .pre_rgb          (wr_data_1),
    .xpos             (pixel_xpos_w   ),
    .ypos             (pixel_ypos_w   ),
    //图像处理后的数据接口
    .post_frame_vsync (post_frame_vsync_1 ),  // 场同步信号
    .post_frame_href ( ),                  // 行同步信号
    .post_frame_de    (post_frame_de_1 ),     // 数据输入使能
    .post_rgb         (post_rgb_1)            // RGB565颜色数据

);
/*    
 //图像处理模块
vip u_vip2(
    //module clock
    .clk              (cam_pclk_2),           // 时钟信号
    .rst_n            (rst_n    ),          // 复位信号（低有效）
    //图像处理前的数据接口
    .pre_frame_vsync  (cmos_frame_vsync_2   ),
    .pre_frame_href   (cmos_frame_href_2   ),
    .pre_frame_de     (cmos_frame_valid_2   ),
    .pre_rgb          (wr_data_2),
    .xpos             (pixel_xpos_w   ),
    .ypos             (pixel_ypos_w   ),
    //图像处理后的数据接口
    .post_frame_vsync (post_frame_vsync_2 ),  // 场同步信号
    .post_frame_href ( ),                  // 行同步信号
    .post_frame_de    (post_frame_de_2 ),     // 数据输入使能
    .post_rgb         (post_rgb_2)            // RGB565颜色数据

);   
*/
ddr3_top u_ddr3_top (
    .clk_200m              (clk_200m),                 //系统时钟
    .sys_rst_n             (rst_n),                     //复位,低有效
    .sys_init_done         (sys_init_done),             //系统初始化完成
    .init_calib_complete   (init_calib_complete),       //ddr3初始化完成信号    
    //ddr3接口信号                                      
    .app_addr_rd_min     (28'd0),                     //读DDR3的起始地址
    .app_addr_rd_max     (ddr3_addr_max[27:1]),       //读DDR3的结束地址
    .rd_bust_len         (H_CMOS_DISP[10:4]),         //从DDR3中读数据时的突发长度
    .app_addr_wr_min     (28'd0),                     //写DDR3的起始地址
    .app_addr_wr_max     (ddr3_addr_max[27:1]),       //写DDR3的结束地址
    .wr_bust_len         (H_CMOS_DISP[10:4]),         //从DDR3中写数据时的突发长度  
    // DDR3 IO接口                
    .ddr3_dq               (ddr3_dq),                   //DDR3 数据
    .ddr3_dqs_n            (ddr3_dqs_n),                //DDR3 dqs负
    .ddr3_dqs_p            (ddr3_dqs_p),                //DDR3 dqs正  
    .ddr3_addr             (ddr3_addr),                 //DDR3 地址   
    .ddr3_ba               (ddr3_ba),                   //DDR3 banck 选择
    .ddr3_ras_n            (ddr3_ras_n),                //DDR3 行选择
    .ddr3_cas_n            (ddr3_cas_n),                //DDR3 列选择
    .ddr3_we_n             (ddr3_we_n),                 //DDR3 读写选择
    .ddr3_reset_n          (ddr3_reset_n),              //DDR3 复位
    .ddr3_ck_p             (ddr3_ck_p),                 //DDR3 时钟正
    .ddr3_ck_n             (ddr3_ck_n),                 //DDR3 时钟负  
    .ddr3_cke              (ddr3_cke),                  //DDR3 时钟使能
    .ddr3_cs_n             (ddr3_cs_n),                 //DDR3 片选
    .ddr3_dm               (ddr3_dm),                   //DDR3_dm
    .ddr3_odt              (ddr3_odt),                  //DDR3_odt
    //用户                                              
    .wr_clk_1              (cam_pclk_1),                //摄像头1时钟
    .wr_load_1             (post_frame_vsync_1),        //摄像头1场信号    
	.datain_valid_1        (post_frame_de_1),        //数据1有效使能信号
    .datain_1              (post_rgb_1),                 //有效数据1 
    .wr_clk_2              (cam_pclk_2),                //摄像头2时钟
    .wr_load_2             (cmos_frame_vsync_2),        //摄像头2场信号    
	.datain_valid_2        (cmos_frame_valid_2),        //数据有效使能信号
    .datain_2              (wr_data_2),                 //有效数据    

    .h_disp                (h_disp),    
    .rd_clk                (pixel_clk),                 //rfifo的读时钟 
    .rd_load               (rd_vsync),                  //lcd场信号    
    .dataout               (rd_data),                   //rfifo输出数据
    .rdata_req             (rdata_req)                  //请求数据输入   
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