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
        input   wire                        rd_load_1       , // RD FIFO加载信号
        input   wire                        rd_load_2       , // RD FIFO加载信号
        input   wire                        wr_load_1           ,  // WR FIFO加载信号 
        input   wire                        wr_load_2           ,  // WR FIFO加载信号 
        input                               datain_valid_1      ,  //数据有效使能信号
        input        [15:0]                 datain_1            ,  //有效数据
        // RGB 数据输入2                                        
        input                               datain_valid_2      ,  //数据有效使能信号
        input        [15:0]                 datain_2            ,  //有效数据
        //用户�?                       
        input   wire [12:0]                 h_disp              ,
        input   wire                        wr_clk_1          , //写FIFO写时钿
        input   wire                        wr_rst_1          , //写复使,模块中是同步复位
        input   wire                        wr_clk_2          , //写FIFO写时钿
        input   wire                        wr_rst_2          , //写复使,模块中是同步复位
        input   wire [28:0]                 wr_beg_addr_1       , //写起始地�?
        input   wire [28:0]                 wr_end_addr_1       , //写终止地�?
        input   wire [28:0]                 wr_beg_addr_2       , //写起始地�?
        input   wire [28:0]                 wr_end_addr_2       , //写终止地�?
        input   wire [7:0]                  wr_burst_len        , //写突发长�?
        input   wire                        wr_en               , //写FIFO写请�?
        input   wire [FIFO_WR_WIDTH-1:0]    wr_data             , //写FIFO写数�? 
        input   wire                        rd_clk_1          , //读FIFO读时钿
        input   wire                        rd_rst_1          , //读复使
        input   wire                        rd_clk_2          , //读FIFO读时钿
        input   wire                        rd_rst_2          , //读复使
        input   wire                        rd_mem_enable       , //读存储器使能,防止存储器未写先�?
        input   wire [28:0]                 rd_beg_addr_1       , //读起始地�?
        input   wire [28:0]                 rd_end_addr_1       , //读终止地�?
        input   wire [28:0]                 rd_beg_addr_2       , //读起始地�?
        input   wire [28:0]                 rd_end_addr_2       , //读终止地�?
        input   wire [7:0]                  rd_burst_len        , //读突发长�?
        input   wire                        rd_en               , //读FIFO读请�?
        //output  wire [FIFO_RD_WIDTH-1:0]    rd_data             , //读FIFO读数�?
        output  wire                        rd_valid            , //读FIFO有效标志,高电平代表当前处理的数据有效
        output  wire                        ui_clk              , //MIG IP核输出的用户时钟, 用作AXI控制器时�?
        output  wire                        ui_rst              , //MIG IP核输出的复位信号, 高电平有�?
        output  wire                        calib_done          , //DDR3初始化完�?
        output       [15:0]                 pic_data            ,    //有效数据
        
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
    
    //AXI连线
    //AXI4写地�?通道
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
    wire [AXI_WIDTH-1:0]    axi_wdata     ; //写数�?
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
    wire [AXI_WIDTH-1:0]    axi_rdata     ; //读数�?
    wire [1:0]              axi_rresp     ; //收到的读响应
    wire                    axi_rlast     ; //�?后一个数据标�?
    wire                    axi_rvalid    ; //读数据有效标�?
    wire                    axi_rready    ; //主机发出的读数据ready
    
    //输入系统时钟异步复位、同步释放处�?
    reg                     rst_n_d1      ;
    reg                     rst_n_sync    ;
    
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
    
   
    
    
    // axi_ddr_ctrl模块
    axi_ddr_ctrl 
        #(.FIFO_WR_WIDTH(FIFO_WR_WIDTH),  
          .FIFO_RD_WIDTH(FIFO_RD_WIDTH),
          .AXI_WIDTH    (AXI_WIDTH    ),
          .AXI_AXSIZE   (AXI_AXSIZE   ),
          .AXI_WSTRB_W  (AXI_WSTRB_W  ) 
          
          )
          axi_ddr_ctrl_inst
         (
        .clk             (ui_clk             ), //AXI读写主机时钟
        .rst_n           (~ui_rst            ), 
        .pingpang        (pingpang           ),
        .clk_100         (ui_clk             ),
        .datain_valid_1  (datain_valid_1     ),
        .datain_valid_2  (datain_valid_2     ),
        .rd_load_1         (rd_load_1         ),
        .rd_load_2         (rd_load_2         ),
        .wr_load_1       (wr_load_1            ),
        .wr_load_2       (wr_load_2            ),
        .datain_1        (datain_1           ),
        .datain_2        (datain_2           ),
        .pic_data        (pic_data           ),
                        
        //用户�?    
        .wr_clk_1          (wr_clk_1          ), //写FIFO写时钿                  
        .wr_rst_1          (wr_rst_1          ), //写复使
        .wr_clk_2          (wr_clk_2          ), //写FIFO写时钿                  
        .wr_rst_2          (wr_rst_2          ), //写复使
        .wr_beg_addr_1   (wr_beg_addr_1      ), //写起始地�?
        .wr_end_addr_1   (wr_end_addr_1      ), //写终止地�?
        .wr_beg_addr_2   (wr_beg_addr_2      ), //写起始地�?
        .wr_end_addr_2   (wr_end_addr_2      ), //写终止地�?
        .wr_burst_len    (wr_burst_len       ), //写突发长�?
         
        .h_disp          (h_disp           ), 
        .rd_clk_1          (rd_clk_1          ), //读FIFO读时钿1
        .rd_rst_1          (rd_rst_1          ), //读复使
        .rd_clk_2          (rd_clk_2          ), //读FIFO读时钿1
        .rd_rst_2          (rd_rst_2          ), //读复使  
        .rd_mem_enable   (rd_mem_enable    ), //读存储器使能,防止存储器未写先�?
        .rd_beg_addr_1   (rd_beg_addr_1      ), //读起始地�?
        .rd_end_addr_1   (rd_end_addr_1      ), //读终止地�?
        .rd_beg_addr_2   (rd_beg_addr_2      ), //读起始地�?
        .rd_end_addr_2   (rd_end_addr_2     ), //读终止地�?
        .rd_burst_len    (rd_burst_len     ), //读突发长�?
        .rd_en           (rd_en            ), //读FIFO读请�?
        //.rd_data         (rd_data          ), //读FIFO读数�?
        .rd_valid        (rd_valid         ), //读FIFO可读标志,表示读FIFO中有数据可以对外输出
        
        //AXI总线
        //AXI4写地�?通道
        .m_axi_awid      (axi_awid         ), 
        .m_axi_awaddr    (axi_awaddr       ),
        .m_axi_awlen     (axi_awlen        ), //突发传输长度
        .m_axi_awsize    (axi_awsize       ), //突发传输大小(Byte)
        .m_axi_awburst   (axi_awburst      ), //突发类型
        .m_axi_awlock    (axi_awlock       ), 
        .m_axi_awcache   (axi_awcache      ), 
        .m_axi_awprot    (axi_awprot       ),
        .m_axi_awqos     (axi_awqos        ),
        .m_axi_awvalid   (axi_awvalid      ), //写地�?valid
        .m_axi_awready   (axi_awready      ), //从机发出的写地址ready
        
        //写数据�?�道
        .m_axi_wdata     (axi_wdata        ), //写数�?
        .m_axi_wstrb     (axi_wstrb        ), //写数据有效字节线
        .m_axi_wlast     (axi_wlast        ), //�?后一个数据标�?
        .m_axi_wvalid    (axi_wvalid       ), //写数据有效标�?
        .m_axi_wready    (axi_wready       ), //从机发出的写数据ready
        
        //写响应�?�道
        .m_axi_bid       (axi_bid          ),
        .m_axi_bresp     (axi_bresp        ), //响应信号,表征写传输是否成�?
        .m_axi_bvalid    (axi_bvalid       ), //响应信号valid标志
        .m_axi_bready    (axi_bready       ), //主机响应ready信号
        
        //AXI4读地�?通道
        .m_axi_arid      (axi_arid         ), 
        .m_axi_araddr    (axi_araddr       ),
        .m_axi_arlen     (axi_arlen        ), //突发传输长度
        .m_axi_arsize    (axi_arsize       ), //突发传输大小(Byte)
        .m_axi_arburst   (axi_arburst      ), //突发类型
        .m_axi_arlock    (axi_arlock       ), 
        .m_axi_arcache   (axi_arcache      ), 
        .m_axi_arprot    (axi_arprot       ),
        .m_axi_arqos     (axi_arqos        ),
        .m_axi_arvalid   (axi_arvalid      ), //读地�?valid
        .m_axi_arready   (axi_arready      ), //从机准备接收读地�?
        
        //读数据�?�道
        .m_axi_rdata     (axi_rdata        ), //读数�?
        .m_axi_rresp     (axi_rresp        ), //收到的读响应
        .m_axi_rlast     (axi_rlast        ), //�?后一个数据标�?
        .m_axi_rvalid    (axi_rvalid       ), //读数据有效标�?
        .m_axi_rready    (axi_rready       )  //主机发出的读数据ready
    );
    
    
    // Vivado MIG IP�?
      mig_7series_0 axi_ddr3_mig_inst (
        // DDR3存储器接�?
        .ddr3_addr              (ddr3_addr          ),  // output [14:0]    ddr3_addr
        .ddr3_ba                (ddr3_ba            ),  // output [2:0]     ddr3_ba
        .ddr3_cas_n             (ddr3_cas_n         ),  // output           ddr3_cas_n
        .ddr3_ck_n              (ddr3_ck_n          ),  // output [0:0]     ddr3_ck_n
        .ddr3_ck_p              (ddr3_ck_p          ),  // output [0:0]     ddr3_ck_p
        .ddr3_cke               (ddr3_cke           ),  // output [0:0]     ddr3_cke
        .ddr3_ras_n             (ddr3_ras_n         ),  // output           ddr3_ras_n
        .ddr3_reset_n           (ddr3_reset_n       ),  // output           ddr3_reset_n
        .ddr3_we_n              (ddr3_we_n          ),  // output           ddr3_we_n
        .ddr3_dq                (ddr3_dq            ),  // inout [31:0]     ddr3_dq
        .ddr3_dqs_n             (ddr3_dqs_n         ),  // inout [3:0]      ddr3_dqs_n
        .ddr3_dqs_p             (ddr3_dqs_p         ),  // inout [3:0]      ddr3_dqs_p
        .init_calib_complete    (calib_done         ),  // output           init_calib_complete
        .ddr3_cs_n              (ddr3_cs_n          ),  // output [0:0]     ddr3_cs_n
        .ddr3_dm                (ddr3_dm            ),  // output [3:0]     ddr3_dm
        .ddr3_odt               (ddr3_odt           ),  // output [0:0]     ddr3_odt
        
        // 用户接口
        .ui_clk                 (ui_clk             ),  // output           ui_clk
        .ui_clk_sync_rst        (ui_rst             ),  // output           ui_clk_sync_rst
        .mmcm_locked            (                   ),  // output           mmcm_locked
        .aresetn                (rst_n_sync         ),  // input            aresetn
        .app_sr_req             (1'b0               ),  // input            app_sr_req
        .app_ref_req            (1'b0               ),  // input            app_ref_req
        .app_zq_req             (1'b0               ),  // input            app_zq_req
        .app_sr_active          (                   ),  // output           app_sr_active
        .app_ref_ack            (                   ),  // output           app_ref_ack
        .app_zq_ack             (                   ),  // output           app_zq_ack
        
        // AXI写地�?通道
        .s_axi_awid             (axi_awid           ),  // input [3:0]      s_axi_awid
        .s_axi_awaddr           (axi_awaddr         ),  // input [29:0]     s_axi_awaddr
        .s_axi_awlen            (axi_awlen          ),  // input [7:0]      s_axi_awlen
        .s_axi_awsize           (axi_awsize         ),  // input [2:0]      s_axi_awsize
        .s_axi_awburst          (axi_awburst        ),  // input [1:0]      s_axi_awburst
        .s_axi_awlock           (axi_awlock         ),  // input [0:0]      s_axi_awlock
        .s_axi_awcache          (axi_awcache        ),  // input [3:0]      s_axi_awcache
        .s_axi_awprot           (axi_awprot         ),  // input [2:0]      s_axi_awprot
        .s_axi_awqos            (axi_awqos          ),  // input [3:0]      s_axi_awqos
        .s_axi_awvalid          (axi_awvalid        ),  // input            s_axi_awvalid
        .s_axi_awready          (axi_awready        ),  // output           s_axi_awready
    
        // AXI写数据�?�道
        .s_axi_wdata            (axi_wdata          ),  // input [AXI_WIDTH-1:0]     s_axi_wdata
        .s_axi_wstrb            (axi_wstrb          ),  // input [AXI_WSTRB_W-1:0]   s_axi_wstrb
        .s_axi_wlast            (axi_wlast          ),  // input                     s_axi_wlast
        .s_axi_wvalid           (axi_wvalid         ),  // input                     s_axi_wvalid
        .s_axi_wready           (axi_wready         ),  // output                    s_axi_wready
                   
        // AXI写响应�?�道        
        .s_axi_bid              (axi_bid            ),  // output [3:0]              s_axi_bid
        .s_axi_bresp            (axi_bresp          ),  // output [1:0]              s_axi_bresp
        .s_axi_bvalid           (axi_bvalid         ),  // output                    s_axi_bvalid
        .s_axi_bready           (axi_bready         ),  // input                     s_axi_bready
                   
        // AXI读地�?通道        
        .s_axi_arid             (axi_arid           ),  // input [3:0]               s_axi_arid
        .s_axi_araddr           (axi_araddr         ),  // input [29:0]              s_axi_araddr
        .s_axi_arlen            (axi_arlen          ),  // input [7:0]               s_axi_arlen
        .s_axi_arsize           (axi_arsize         ),  // input [2:0]               s_axi_arsize
        .s_axi_arburst          (axi_arburst        ),  // input [1:0]               s_axi_arburst
        .s_axi_arlock           (axi_arlock         ),  // input [0:0]               s_axi_arlock
        .s_axi_arcache          (axi_arcache        ),  // input [3:0]               s_axi_arcache
        .s_axi_arprot           (axi_arprot         ),  // input [2:0]               s_axi_arprot
        .s_axi_arqos            (axi_arqos          ),  // input [3:0]               s_axi_arqos
        .s_axi_arvalid          (axi_arvalid        ),  // input                     s_axi_arvalid
        .s_axi_arready          (axi_arready        ),  // output                    s_axi_arready
        
        // AXI读数据�?�道
        .s_axi_rid              (axi_rid            ),  // output [3:0]              s_axi_rid
        .s_axi_rdata            (axi_rdata          ),  // output [AXI_WIDTH-1:0]    s_axi_rdata
        .s_axi_rresp            (axi_rresp          ),  // output [1:0]              s_axi_rresp
        .s_axi_rlast            (axi_rlast          ),  // output                    s_axi_rlast
        .s_axi_rvalid           (axi_rvalid         ),  // output                    s_axi_rvalid
        .s_axi_rready           (axi_rready         ),  // input                     s_axi_rready
        
        // AXI从机系统时钟
        .sys_clk_i              (clk                ),
        // 参�?�时�?
        .clk_ref_i              (clk                ),
        .sys_rst                (rst_n_sync         )   // input            sys_rst
    );
    
    
    
    
    
endmodule
