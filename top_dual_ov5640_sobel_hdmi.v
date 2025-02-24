

module top_dual_ov5640_sobel_hdmi(    
    input                 sys_clk        ,  //ϵͳʱ��
    input                 sys_rst_n      ,  //ϵͳ��λ���͵�ƽ��Ч
    //����ͷ1�ӿ�                       
    input                 cam_pclk_1     ,  //cmos ��������ʱ��
    input                 cam_vsync_1    ,  //cmos ��ͬ���ź�
    input                 cam_href_1     ,  //cmos ��ͬ���ź�
    input   [7:0]         cam_data_1     ,  //cmos ����
    output                cam_rst_n_1    ,  //cmos ��λ�źţ��͵�ƽ��Ч
    output                cam_pwdn_1 ,      //��Դ����ģʽѡ�� 0������ģʽ 1����Դ����ģʽ
    output                cam_scl_1      ,  //cmos SCCB_SCL��
    inout                 cam_sda_1      ,  //cmos SCCB_SDA��
    //����ͷ2�ӿ�     
    input                 cam_pclk_2     ,  //cmos ��������ʱ��
    input                 cam_vsync_2    ,  //cmos ��ͬ���ź�
    input                 cam_href_2     ,  //cmos ��ͬ���ź�
    input   [7:0]         cam_data_2     ,  //cmos ����
    output                cam_rst_n_2    ,  //cmos ��λ�źţ��͵�ƽ��Ч
    output                cam_pwdn_2     ,  //��Դ����ģʽѡ�� 0������ģʽ 1����Դ����ģʽ
    output                cam_scl_2      ,  //cmos SCCB_SCL��
    inout                 cam_sda_2      ,  //cmos SCCB_SDA��   
       
    // DDR3                            
    inout   [31:0]        ddr3_dq        ,   //ddr3 ����
    inout   [3:0]         ddr3_dqs_n     ,   //ddr3 dqs��
    inout   [3:0]         ddr3_dqs_p     ,   //ddr3 dqs��  
    output  [13:0]        ddr3_addr      ,   //ddr3 ��ַ   
    output  [2:0]         ddr3_ba        ,   //ddr3 banck ѡ��
    output                ddr3_ras_n     ,   //ddr3 ��ѡ��
    output                ddr3_cas_n     ,   //ddr3 ��ѡ��
    output                ddr3_we_n      ,   //ddr3 ��дѡ��
    output                ddr3_reset_n   ,   //ddr3 ��λ
    output  [0:0]         ddr3_ck_p      ,   //ddr3 ʱ����
    output  [0:0]         ddr3_ck_n      ,   //ddr3 ʱ�Ӹ�
    output  [0:0]         ddr3_cke       ,   //ddr3 ʱ��ʹ��
    output  [0:0]         ddr3_cs_n      ,   //ddr3 Ƭѡ
    output  [3:0]         ddr3_dm        ,   //ddr3_dm
    output  [0:0]         ddr3_odt       ,   //ddr3_odt  								   
    //hdmi�ӿ�                           
    output                tmds_clk_p     ,  // TMDS ʱ��ͨ��
    output                tmds_clk_n     ,
    output  [2:0]         tmds_data_p    ,  // TMDS ����ͨ��
    output  [2:0]         tmds_data_n    
    );                                 

parameter  V_CMOS_DISP = 11'd768;                  //CMOS�ֱ���--��
parameter  H_CMOS_DISP = 11'd1024;                 //CMOS�ֱ���--��	
parameter  TOTAL_H_PIXEL = H_CMOS_DISP + 12'd1216; //CMOS�ֱ���--��
parameter  TOTAL_V_PIXEL = V_CMOS_DISP + 12'd504;      								   
							   
//wire define                          
wire         clk_50m                   ;  //50mhzʱ��
wire         locked                    ;  //ʱ�������ź�
wire         rst_n                     ;  //ȫ�ָ�λ 								    						    
wire         wr_en                     ;  //DDR3������ģ��дʹ��
wire         rdata_req                 ;  //DDR3������ģ���ʹ��
wire  [15:0] rd_data                   ;  //DDR3������ģ�������
wire         cmos_frame_valid_1        ;  //����1��Чʹ���ź�
wire  [15:0] wr_data_1                 ;  //DDR3������ģ��д����1
wire         cmos_frame_valid_2        ;  //����2��Чʹ���ź�
wire  [15:0] wr_data_2                 ;  //DDR3������ģ��д����2
wire         init_calib_complete       ;  //DDR3��ʼ�����init_calib_complete
wire         sys_init_done             ;  //ϵͳ��ʼ�����(DDR��ʼ��+����ͷ��ʼ��)
wire         clk_200m                  ;  //ddr3�ο�ʱ��
wire         cmos_frame_vsync_1        ;  //���֡1��Ч��ͬ���ź�
wire         cmos_frame_vsync_2        ;  //���֡2��Ч��ͬ���ź�
wire         cmos_frame_href_1         ;  //���֡��Ч��ͬ���ź� 
wire         cmos_frame_href_2         ;  //���֡��Ч��ͬ���ź� 
wire  [10:0] pixel_xpos_w              ;
wire  [10:0] pixel_ypos_w              ;
wire  [12:0] h_disp                    ;  //LCD��ˮƽ�ֱ���
wire  [12:0] v_disp                    ;  //LCD����ֱ�ֱ���   
wire  [15:0] post_rgb_1                ;  //������ͼ������
wire         post_frame_vsync_1        ;  //�����ĳ��ź�
wire         post_frame_de_1           ;  //������������Чʹ�� 
wire  [15:0] post_rgb_2                ;  //������ͼ������
wire         post_frame_vsync_2        ;  //�����ĳ��ź�
wire         post_frame_de_2           ;  //������������Чʹ�� 
wire         rd_vsync                  ;
// ����DDR3��ַ���ֵ
wire [27:0] ddr3_addr_max              ;
//*****************************************************
//**                    main code
//*****************************************************

//*****************************************************
//**                    main code
//*****************************************************

//��ʱ�������������λ�����ź�
assign  rst_n = sys_rst_n & locked;

//ϵͳ��ʼ����ɣ�DDR3��ʼ�����
assign  sys_init_done = init_calib_complete;

//����DDR3������д��ַ 
assign  ddr3_addr_max =  V_CMOS_DISP*H_CMOS_DISP; 
   
 //ov5640 ����
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
    
  //ov5640 ����
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

 //ͼ����ģ��
vip u_vip1(
    //module clock
    .clk              (cam_pclk_1),           // ʱ���ź�
    .rst_n            (rst_n    ),          // ��λ�źţ�����Ч��
    //ͼ����ǰ�����ݽӿ�
    .pre_frame_vsync  (cmos_frame_vsync_1   ),
    .pre_frame_href   (cmos_frame_href_1   ),
    .pre_frame_de     (cmos_frame_valid_1   ),
    .pre_rgb          (wr_data_1),
    .xpos             (pixel_xpos_w   ),
    .ypos             (pixel_ypos_w   ),
    //ͼ���������ݽӿ�
    .post_frame_vsync (post_frame_vsync_1 ),  // ��ͬ���ź�
    .post_frame_href ( ),                  // ��ͬ���ź�
    .post_frame_de    (post_frame_de_1 ),     // ��������ʹ��
    .post_rgb         (post_rgb_1)            // RGB565��ɫ����

);    
 //ͼ����ģ��
vip u_vip2(
    //module clock
    .clk              (cam_pclk_2),           // ʱ���ź�
    .rst_n            (rst_n    ),          // ��λ�źţ�����Ч��
    //ͼ����ǰ�����ݽӿ�
    .pre_frame_vsync  (cmos_frame_vsync_2   ),
    .pre_frame_href   (cmos_frame_href_2   ),
    .pre_frame_de     (cmos_frame_valid_2   ),
    .pre_rgb          (wr_data_2),
    .xpos             (pixel_xpos_w   ),
    .ypos             (pixel_ypos_w   ),
    //ͼ���������ݽӿ�
    .post_frame_vsync (post_frame_vsync_2 ),  // ��ͬ���ź�
    .post_frame_href ( ),                  // ��ͬ���ź�
    .post_frame_de    (post_frame_de_2 ),     // ��������ʹ��
    .post_rgb         (post_rgb_2)            // RGB565��ɫ����

);   

ddr3_top u_ddr3_top (
    .clk_200m              (clk_200m),                 //ϵͳʱ��
    .sys_rst_n             (rst_n),                     //��λ,����Ч
    .sys_init_done         (sys_init_done),             //ϵͳ��ʼ�����
    .init_calib_complete   (init_calib_complete),       //ddr3��ʼ������ź�    
    //ddr3�ӿ��ź�                                      
    .app_addr_rd_min     (28'd0),                     //��DDR3����ʼ��ַ
    .app_addr_rd_max     (ddr3_addr_max[27:1]),       //��DDR3�Ľ�����ַ
    .rd_bust_len         (H_CMOS_DISP[10:4]),         //��DDR3�ж�����ʱ��ͻ������
    .app_addr_wr_min     (28'd0),                     //дDDR3����ʼ��ַ
    .app_addr_wr_max     (ddr3_addr_max[27:1]),       //дDDR3�Ľ�����ַ
    .wr_bust_len         (H_CMOS_DISP[10:4]),         //��DDR3��д����ʱ��ͻ������  
    // DDR3 IO�ӿ�                
    .ddr3_dq               (ddr3_dq),                   //DDR3 ����
    .ddr3_dqs_n            (ddr3_dqs_n),                //DDR3 dqs��
    .ddr3_dqs_p            (ddr3_dqs_p),                //DDR3 dqs��  
    .ddr3_addr             (ddr3_addr),                 //DDR3 ��ַ   
    .ddr3_ba               (ddr3_ba),                   //DDR3 banck ѡ��
    .ddr3_ras_n            (ddr3_ras_n),                //DDR3 ��ѡ��
    .ddr3_cas_n            (ddr3_cas_n),                //DDR3 ��ѡ��
    .ddr3_we_n             (ddr3_we_n),                 //DDR3 ��дѡ��
    .ddr3_reset_n          (ddr3_reset_n),              //DDR3 ��λ
    .ddr3_ck_p             (ddr3_ck_p),                 //DDR3 ʱ����
    .ddr3_ck_n             (ddr3_ck_n),                 //DDR3 ʱ�Ӹ�  
    .ddr3_cke              (ddr3_cke),                  //DDR3 ʱ��ʹ��
    .ddr3_cs_n             (ddr3_cs_n),                 //DDR3 Ƭѡ
    .ddr3_dm               (ddr3_dm),                   //DDR3_dm
    .ddr3_odt              (ddr3_odt),                  //DDR3_odt
    //�û�                                              
    .wr_clk_1              (cam_pclk_1),                //����ͷ1ʱ��
    .wr_load_1             (post_frame_vsync_1),        //����ͷ1���ź�    
	.datain_valid_1        (post_frame_de_1),        //����1��Чʹ���ź�
    .datain_1              (post_rgb_1),                 //��Ч����1 
    .wr_clk_2              (cam_pclk_2),                //����ͷ2ʱ��
    .wr_load_2             (post_frame_vsync_2),        //����ͷ2���ź�    
	.datain_valid_2        (post_frame_de_2),        //������Чʹ���ź�
    .datain_2              (post_rgb_2),                 //��Ч����    

    .h_disp                (h_disp),    
    .rd_clk                (pixel_clk),                 //rfifo�Ķ�ʱ�� 
    .rd_load               (rd_vsync),                  //lcd���ź�    
    .dataout               (rd_data),                   //rfifo�������
    .rdata_req             (rdata_req)                  //������������   
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
 
//HDMI������ʾģ��    
hdmi_top u_hdmi_top(
    .pixel_clk            (pixel_clk),
    .pixel_clk_5x         (pixel_clk_5x),    
    .sys_rst_n            (sys_init_done & rst_n),
    //hdmi�ӿ�                   
    .tmds_clk_p           (tmds_clk_p   ),   // TMDS ʱ��ͨ��
    .tmds_clk_n           (tmds_clk_n   ),
    .tmds_data_p          (tmds_data_p  ),   // TMDS ����ͨ��
    .tmds_data_n          (tmds_data_n  ),
    //�û��ӿ� 
    .video_vs             (rd_vsync     ),   //HDMI���ź�  
    .h_disp               (h_disp),          //HDMI��ˮƽ�ֱ���
    .v_disp               (v_disp),          //HDMI����ֱ�ֱ���   
    .pixel_xpos           (pixel_xpos_w),
    .pixel_ypos           (pixel_ypos_w),      
    .data_in              (rd_data),         //�������� 
    .data_req             (rdata_req)        //������������   
);  
    
endmodule