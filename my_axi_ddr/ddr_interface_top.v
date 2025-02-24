`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer: lauchinyuan
// Email: lauchinyuan@yeah.net
// Create Date: 2023/09/20 21:43:45
// Module Name: ddr_interface
// Description: DDR3顶层模块, 将MIG IP核与axi_ddr_ctrl模块封装起来
// 其中axi_ddr_ctrl模块包含AXI主机, 读FIFO、写FIFO及AXI读写控制器axi_ctrl
// 外接DDR3存储�?,即可实现对DDR3存储器的FIFO式读�?
//////////////////////////////////////////////////////////////////////////////////


module ddr_interface
    #(parameter FIFO_WR_WIDTH = 'd256    ,  //用户端FIFO读写位宽
                FIFO_RD_WIDTH = 'd256    ,
                AXI_WIDTH     = 'd256    ,  //AXI总线读写数据位宽
                AXI_AXSIZE    = 3'b101     //AXI总线的axi_awsize, �?要与AXI_WIDTH对应
                )
        (
        input   wire                        clk                 , //DDR3时钟, 也就是DDR3 MIG IP核参考时�?
        input   wire                        rst_n               , 
        input   wire                        pingpang            ,//乒乓操作                      
        input                               datain_valid_1        ,  //数据有效使能信号
        input                               datain_valid_2        ,  //数据有效使能信号
        input   wire                        rd_load               , // RD FIFO加载信号
        input   wire                        wr_load_1             ,  // WR FIFO加载信号
        input   wire                        wr_load_2             ,  // WR FIFO加载信号         
        input        [15:0]                 datain_1              ,  //有效数据
        input        [15:0]                 datain_2              ,  //有效数据        
        //用户�?                       
        input   wire                        wr_clk_1              , //写FIFO写时�?
        input   wire                        wr_rst_1              , //写复�?
        input   wire                        wr_clk_2              , //写FIFO写时�?
        input   wire                        wr_rst_2              , //写复�?
        input   wire [28:0]                 wr_beg_addr_1         , //写起始地�?
        input   wire [28:0]                 wr_end_addr_1         , //写终止地�?
        input   wire [28:0]                 wr_beg_addr_2         , //写起始地�?
        input   wire [28:0]                 wr_end_addr_2         , //写终止地�?
        input   wire [7:0]                  wr_burst_len        , //写突发长�?
        //input   wire                        wr_en_1               , //写FIFO写请�?
        //input   wire [FIFO_WR_WIDTH-1:0]    wr_data_1             , //写FIFO写数�?            
        //input   wire                        wr_en_2               , //写FIFO写请�?            
       // input   wire [FIFO_WR_WIDTH-1:0]    wr_data_2             , //写FIFO写数�?
        input   wire                        rd_clk              , //读FIFO读时�?
        input   wire                        rd_rst              , //读复�?
        input   wire                        rd_mem_enable       , //读存储器使能,防止存储器未写先�?
        input   wire [28:0]                 rd_beg_addr_1         , //读起始地�?
        input   wire [28:0]                 rd_end_addr_1         , //读终止地�?
        input   wire [28:0]                 rd_beg_addr_2         , //读起始地�?
        input   wire [28:0]                 rd_end_addr_2         , //读终止地�?
        input   wire [7:0]                  rd_burst_len        , //读突发长�?
        input   wire                        rd_en               , //读FIFO读请�?
        //input   wire                        rd_en_2               , //读FIFO读请�?
        //output  wire [FIFO_RD_WIDTH-1:0]    rd_data             , //读FIFO读数�?
        output  wire                        rd_valid_1            , //读FIFO有效标志,高电平代表当前处理的数据有效
        output  wire                        rd_valid_2            , //读FIFO有效标志,高电平代表当前处理的数据有效
        output  wire                        ui_clk              , //MIG IP核输出的用户时钟, 用作AXI控制器时�?
        output  wire                        ui_rst              , //MIG IP核输出的复位信号, 高电平有�?
        output  wire                        calib_done          , //DDR3初始化完�?
        output       [15:0]                  pic_data            ,    //有效数据
       // output       [15:0]                 pic_data_2          ,    //有效数据
        input         [12:0]                 h_disp              ,
        //DDR3接口                              
        output  wire [14:0]                 ddr3_addr           ,  
        output  wire [2:0]                  ddr3_ba             ,
        output  wire                        ddr3_cas_n          ,
        output  wire                        ddr3_ck_n           ,
        output  wire                        ddr3_ck_p           ,
        output  wire                        ddr3_cke            ,
        output  wire                        ddr3_ras_n          ,
        output  wire                        ddr3_reset_n        ,
        output  wire                        ddr3_we_n           ,
        inout   wire [31:0]                 ddr3_dq             ,
        inout   wire [3:0]                  ddr3_dqs_n          ,
        inout   wire [3:0]                  ddr3_dqs_p          ,
        output  wire                        ddr3_cs_n           ,
        output  wire [3:0]                  ddr3_dm             ,
        output  wire                        ddr3_odt            
        
    );
    
    localparam AXI_WSTRB_W   = AXI_WIDTH >> 3   ; //axi_wstrb的位�?, AXI_WIDTH/8
    reg  [12:0]             rd_cnt        ;
    //AXI连线
    //AXI4写地�?通道
    wire [15:0]             pic_data_1    ;    //有效数据
    wire [15:0]             pic_data_2    ;    //有效数据
    wire [3:0]              axi_awid      ; 
    wire [28:0]             axi_awaddr    ;
    wire [7:0]              axi_awlen     ; //突发传输长度
    wire [2:0]              axi_awsize    ; //突发传输大小(Byte)
    wire [1:0]              axi_awburst   ; //突发类型
    wire                    axi_awlock    ; 
    wire [3:0]              axi_awcache   ; 
    wire [2:0]              axi_awprot    ;
    wire [3:0]              axi_awqos     ;
    wire                    axi_awvalid   ; //写地�?valid
    wire                    axi_awready   ; //从机发出的写地址ready
    
    //写数据�?�道
    wire [AXI_WIDTH-1:0]    axi_wdata_1   ; //写数�?
    wire [AXI_WIDTH-1:0]    axi_wdata_2   ; //写数�?
    wire [AXI_WSTRB_W-1:0]  axi_wstrb     ; //写数据有效字节线
    wire                    axi_wlast     ; //�?后一个数据标�?
    wire                    axi_wvalid    ; //写数据有效标�?
    wire                    axi_wready    ; //从机发出的写数据ready
                
    //写响应�?�道         
    wire [3:0]              axi_bid       ;
    wire [1:0]              axi_bresp     ; //响应信号,表征写传输是否成�?
    wire                    axi_bvalid    ; //响应信号valid标志
    wire                    axi_bready    ; //主机响应ready信号
    
    //读地�?通道
    wire [3:0]              axi_arid      ; 
    wire [28:0]             axi_araddr    ; 
    wire [7:0]              axi_arlen     ; //突发传输长度
    wire [2:0]              axi_arsize    ; //突发传输大小(Byte)
    wire [1:0]              axi_arburst   ; //突发类型
    wire                    axi_arlock    ; 
    wire [3:0]              axi_arcache   ; 
    wire [2:0]              axi_arprot    ;
    wire [3:0]              axi_arqos     ;
    wire                    axi_arvalid   ; //读地�?valid
    wire                    axi_arready   ; //从机准备接收读地�?
    
    //读数据�?�道
    wire [AXI_WIDTH-1:0]    axi_rdata_1   ; //读数�?
    wire [AXI_WIDTH-1:0]    axi_rdata_2   ; //读数�?
    wire [1:0]              axi_rresp     ; //收到的读响应
    wire                    axi_rlast     ; //�?后一个数据标�?
    wire                    axi_rvalid    ; //读数据有效标�?
    wire                    axi_rready    ; //主机发出的读数据ready
    
    //输入系统时钟异步复位、同步释放处�?
    reg                     rst_n_d1      ;
    reg                     rst_n_sync    ;
    wire                    axi_reading_1 ;
    wire                    axi_reading_2 ;
    wire                    axi_writing_1 ;
    wire                    axi_writing_2 ;
    wire            [255:0] axi_wdata     ;
    wire                    rd_en_1       ;
    wire                    rd_en_2       ;
    
    
    //rst_n_d1、rst_n_sync
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin  //异步复位
            rst_n_d1    <= 1'b0;
            rst_n_sync  <= 1'b0;
        end else begin   //同步释放
            rst_n_d1    <= 1'b1;
            rst_n_sync  <= rst_n_d1;
        end
    end
    
   //*****************************************************
//**                    main code
//*****************************************************
//像素显示请求信号切换，即显示器左侧请求FIFO1显示，右侧请求FIFO2显示
assign rd_en_1  = (rd_cnt <= h_disp[12:1]-1) ? rd_en :1'b0;//右移/2
assign rd_en_2  = (rd_cnt <= h_disp[12:1]-1) ? 1'b0 :rd_en;



//assign rd_en_1  =  (rd_cnt <= h_disp[12:0]-1) ? rd_en :1'b0;//
//像素在显示器显示位置的切换，即显示器左侧显示FIFO0,右侧显示FIFO1
assign pic_data =  (rd_cnt <= h_disp[12:1]) ? pic_data_1: pic_data_2;
//assign pic_data =     (rd_cnt <= h_disp[12:0]) ? pic_data_1 : 16'd0;
//写入DDR3的像素数据切�?
assign axi_wdata = axi_writing_1 ? axi_wdata_1 : axi_wdata_2; 

//对读请求信号计数
always @(posedge rd_clk or negedge rst_n) begin
    if(!rst_n)
        rd_cnt <= 13'd0;
    else if(rd_en)
        rd_cnt <= rd_cnt + 1'b1;
    else
        rd_cnt <= 13'd0;
end
    
    
   
    
    
endmodule
