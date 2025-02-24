`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer: lauchinyuan
// Email: lauchinyuan@yeah.net
// Create Date: 2023/09/18 21:49:34
// Module Name: axi_ddr_ctrl
// Description: AXI接口DDR控制顶层模块,集成AXI读主机�?�AXI写主机�?�AXI控制�?(包含读写FIFO)
//////////////////////////////////////////////////////////////////////////////////


module axi_ddr_ctrl
    #(parameter FIFO_WR_WIDTH = 'd256            ,  //用户端FIFO读写位宽
                FIFO_RD_WIDTH = 'd256            ,
                AXI_WIDTH     = 'd256            ,  //AXI总线读写数据位宽
                AXI_AXSIZE    = 3'b101           ,  //AXI总线的axi_axsize, �?要与AXI_WIDTH对应
                AXI_WSTRB_W   = AXI_WIDTH>>3    )   //axi_wstrb的位�?, AXI_WIDTH/8
        (
        input   wire                        clk             , //AXI读写主机时钟(ui_clk)
        input   wire                        rst_n           , 
        input   wire                        pingpang        ,//乒乓操作
        input                               clk_100         ,  //用户时钟
        //用户�?                   
        input   wire                        wr_clk_1          , //写FIFO写时钿
        input   wire                        wr_rst_1          , //写复使,模块中是同步复位
        input   wire                        wr_clk_2          , //写FIFO写时钿
        input   wire                        wr_rst_2          , //写复使,模块中是同步复位
        input   wire [28:0]                 wr_beg_addr_1     , //写起始地坿
        input   wire [28:0]                 wr_end_addr_1     , //写终止地坿
        input   wire [28:0]                 wr_beg_addr_2     , //写起始地坿
        input   wire [28:0]                 wr_end_addr_2     , //写终止地坿
        input   wire [7:0]                  wr_burst_len    , //写突发长�?
        input                               datain_valid_1  ,  //数据有效使能信号
        input        [15:0]                 datain_1        ,  //有效数据
        // RGB 数据输入2
        input                               datain_valid_2  ,  //数据有效使能信号
        input        [15:0]                 datain_2        ,  //有效数据
        input   wire                        rd_load_1       , // RD FIFO加载信号
        input   wire                        rd_load_2       , // RD FIFO加载信号
        input   wire                        wr_load_1       ,  // WR FIFO加载信号 
        input   wire                        wr_load_2       ,  // WR FIFO加载信号 
        input   wire [12:0]                 h_disp          ,
        input   wire                        rd_clk_1          , //读FIFO读时�?
        input   wire                        rd_rst_1          , //读复�?
        input   wire                        rd_clk_2          , //读FIFO读时钿
        input   wire                        rd_rst_2          , //读复使
        input   wire                        rd_mem_enable   , //读存储器使能,防止存储器未写先�?
        input   wire [28:0]                 rd_beg_addr_1     , //读起始地坿
        input   wire [28:0]                 rd_end_addr_1     , //读终止地坿
        input   wire [28:0]                 rd_beg_addr_2     , //读起始地坿
        input   wire [28:0]                 rd_end_addr_2     , //读终止地坿
        input   wire [7:0]                  rd_burst_len    , //读突发长�?
        input   wire                        rd_en           , //读FIFO读请�?
        //output  wire [FIFO_RD_WIDTH-1:0]    rd_data         , //读FIFO读数�?
        output  wire                        rd_valid        , //读FIFO可读标志,表示读FIFO中有数据可以对外输出
        output       [15:0]                 pic_data        ,    //有效数据
                        
        //AXI总线             
        //AXI4写地�?通道             
        input   wire [3:0]                  m_axi_awid      , 
        output  wire [28:0]                 m_axi_awaddr    ,
        output  wire [7:0]                  m_axi_awlen     , //突发传输长度
        output  wire [2:0]                  m_axi_awsize    , //突发传输大小(Byte)
        output  wire [1:0]                  m_axi_awburst   , //突发类型
        output  wire                        m_axi_awlock    , 
        output  wire [3:0]                  m_axi_awcache   , 
        output  wire [2:0]                  m_axi_awprot    ,
        output  wire [3:0]                  m_axi_awqos     ,
        output  wire                        m_axi_awvalid   , //写地�?valid
        input   wire                        m_axi_awready   , //从机发出的写地址ready
                        
        //写数据�?�道
        output  wire                        axi_writing     ,
        output  wire [AXI_WIDTH-1:0]        m_axi_wdata     , //写数�?
        output  wire [AXI_WSTRB_W-1:0]      m_axi_wstrb     , //写数据有效字节线
        output  wire                        m_axi_wlast     , //�?后一个数据标�?
        output  wire                        m_axi_wvalid    , //写数据有效标�?
        input   wire                        m_axi_wready    , //从机发出的写数据ready
                        
        //写响应�?�道             
        output  wire [3:0]                  m_axi_bid       ,
        input   wire [1:0]                  m_axi_bresp     , //响应信号,表征写传输是否成�?
        input   wire                        m_axi_bvalid    , //响应信号valid标志
        output  wire                        m_axi_bready    , //主机响应ready信号
                        
        //AXI4读地�?通道             
        output  wire [3:0]                  m_axi_arid      , 
        output  wire [28:0]                 m_axi_araddr    ,
        output  wire [7:0]                  m_axi_arlen     , //突发传输长度
        output  wire [2:0]                  m_axi_arsize    , //突发传输大小(Byte)
        output  wire [1:0]                  m_axi_arburst   , //突发类型
        output  wire                        m_axi_arlock    , 
        output  wire [3:0]                  m_axi_arcache   , 
        output  wire [2:0]                  m_axi_arprot    ,
        output  wire [3:0]                  m_axi_arqos     ,
        output  wire                        m_axi_arvalid   , //读地�?valid
        input   wire                        m_axi_arready   , //从机准备接收读地�?
                        
        //读数据�?�道   
        input   wire                        axi_reading     ,       
        input   wire [AXI_WIDTH-1:0]        m_axi_rdata     , //读数�?
        input   wire [1:0]                  m_axi_rresp     , //收到的读响应
        input   wire                        m_axi_rlast     , //�?后一个数据标�?
        input   wire                        m_axi_rvalid    , //读数据有效标�?
        output  wire                        m_axi_rready      //主机发出的读数据ready
        

        
    );
    
    
    //连线
    //AXI控制器到AXI写主�?
    wire                    axi_writing     ;
    wire                    axi_wr_ready    ;
    wire                    axi_wr_start    ;
    wire [AXI_WIDTH-1:0]    axi_wr_data     ;
    wire [28:0]             axi_wr_addr     ;
    wire [7:0]              axi_wr_len      ;
    wire                    axi_wr_done     ;
    
    //读AXI主机
    wire                    axi_reading     ;
    wire                    axi_rd_ready    ;
    wire                    axi_rd_start    ;
    wire [AXI_WIDTH-1:0]    axi_rd_data     ;
    wire [28:0]             axi_rd_addr     ;
    wire [7:0]              axi_rd_len      ;
    wire                    axi_rd_done     ;
    
    //AXI控制�?
    axi_ctrl 
    #(.FIFO_WR_WIDTH(FIFO_WR_WIDTH),  //用户端FIFO读写位宽
      .FIFO_RD_WIDTH(FIFO_RD_WIDTH),
      .AXI_WIDTH    (AXI_WIDTH    )
      )                                                                      
      axi_ctrl_inst                                                          
    (                                                                        
        .clk             (clk             ), //AXI读写主机时钟               
        .rst_n           (rst_n           ),                                 
        .pingpang        (pingpang        ),
        .clk_100         (clk_100         ),
        .datain_valid_1    (datain_valid_1    ),
        .datain_valid_2    (datain_valid_2    ),
        .rd_load_1         (rd_load_1         ),
        .rd_load_2         (rd_load_2         ),
        .wr_load_1         (wr_load_1         ),
        .wr_load_2         (wr_load_2         ),
        .datain_1          (datain_1          ),
        .datain_2          (datain_2          ),
        .pic_data        (pic_data        ),  
        .wr_clk_1          (wr_clk_1          ), //写FIFO写时�?                  
        .wr_rst_1          (wr_rst_1          ), //写复�?
        .wr_clk_2          (wr_clk_2          ), //写FIFO写时钿                  
        .wr_rst_2          (wr_rst_2          ), //写复使
        .wr_beg_addr_1     (wr_beg_addr_1     ), //写起始地�?
        .wr_end_addr_1     (wr_end_addr_1     ), //写终止地�?
        .wr_beg_addr_2     (wr_beg_addr_2     ), //写起始地坿
        .wr_end_addr_2     (wr_end_addr_2     ), //写终止地坿
        .wr_burst_len    (wr_burst_len    ), //写突发长�?
  //      .wr_en           (wr_en           ), //写FIFO写请�?
  //      .wr_data         (wr_data         ), //写FIFO写数�? 
        .h_disp            (h_disp            ),
        .rd_clk_1          (rd_clk_1          ), //读FIFO读时�?1
        .rd_rst_1          (rd_rst_1          ), //读复�?
        .rd_clk_2          (rd_clk_2          ), //读FIFO读时钿1
        .rd_rst_2          (rd_rst_2          ), //读复使     
        .rd_mem_enable   (rd_mem_enable   ), //读存储器使能,防止存储器未写先�?
        .rd_beg_addr_1     (rd_beg_addr_1     ), //读起始地�?
        .rd_end_addr_1     (rd_end_addr_1     ), //读终止地�?
        .rd_beg_addr_2     (rd_beg_addr_2     ), //读起始地坿
        .rd_end_addr_2     (rd_end_addr_2     ), //读终止地坿
        .rd_burst_len    (rd_burst_len    ), //读突发长�?
        .rd_en           (rd_en           ), //读FIFO读请�?
        //.rd_data         (rd_data         ), //读FIFO读数�?
        .rd_valid        (rd_valid        ), //读FIFO可读标志,表示读FIFO中有数据可以对外输出
        
        //写AXI主机
        .axi_writing     (axi_writing     ), //AXI主机写正在进�?
        .axi_wr_ready    (axi_wr_ready    ), //AXI主机写准备好
        .axi_wr_start    (axi_wr_start    ), //AXI主机写请�?
        .axi_wr_data     (axi_wr_data     ), //从写FIFO中读取的数据,写入AXI写主�?
        .axi_wr_addr     (axi_wr_addr     ), //AXI主机写地�?
        .axi_wr_len      (axi_wr_len      ), //AXI主机写突发长�?
        .axi_wr_done     (axi_wr_done     ),
        
        //读AXI主机
        .axi_reading     (axi_reading     ), //AXI主机读正在进�?
        .axi_rd_ready    (axi_rd_ready    ), //AXI主机读准备好
        .axi_rd_start    (axi_rd_start    ), //AXI主机读请�?
        .axi_rd_data     (axi_rd_data     ), //从AXI读主机读到的数据,写入读FIFO
        .axi_rd_addr     (axi_rd_addr     ), //AXI主机读地�?
        .axi_rd_len      (axi_rd_len      ), //AXI主机读突发长�?   
        .axi_rd_done     (axi_rd_done     )
    );
    
    
    
    
    //AXI读主�?
    axi_master_rd 
    #(  .AXI_WIDTH     (AXI_WIDTH     ),  //AXI总线读写数据位宽
        .AXI_AXSIZE    (AXI_AXSIZE    ))   //AXI总线的axi_axsize, �?要与AXI_WIDTH对应    
        axi_master_rd_inst
    (
        //用户�?
        .clk              (clk              ),
        .rst_n            (rst_n            ),
        .rd_start         (axi_rd_start     ), //�?始读信号
        .rd_addr          (axi_rd_addr      ), //读首地址
        .rd_data          (axi_rd_data      ), //读出的数�?
        .rd_len           (axi_rd_len       ), //突发传输长度
        .rd_done          (axi_rd_done      ), //读完成标�?
        .rd_ready         (axi_rd_ready     ), //准备好读标志
        .m_axi_r_handshake(axi_reading      ), //读�?�道成功握手
        
        //AXI4读地�?通道
        .m_axi_arid       (m_axi_arid       ), 
        .m_axi_araddr     (m_axi_araddr     ),
        .m_axi_arlen      (m_axi_arlen      ), //突发传输长度
        .m_axi_arsize     (m_axi_arsize     ), //突发传输大小(Byte)
        .m_axi_arburst    (m_axi_arburst    ), //突发类型
        .m_axi_arlock     (m_axi_arlock     ), 
        .m_axi_arcache    (m_axi_arcache    ), 
        .m_axi_arprot     (m_axi_arprot     ),
        .m_axi_arqos      (m_axi_arqos      ),
        .m_axi_arvalid    (m_axi_arvalid    ), //读地�?valid
        .m_axi_arready    (m_axi_arready    ), //从机准备接收读地�?
                                            
        //读数据�?�道                        
        .m_axi_rdata      (m_axi_rdata      ), //读数�?
        .m_axi_rresp      (m_axi_rresp      ), //收到的读响应
        .m_axi_rlast      (m_axi_rlast      ), //�?后一个数据标�?
        .m_axi_rvalid     (m_axi_rvalid     ), //读数据有效标�?
        .m_axi_rready     (m_axi_rready     )  //主机发出的读数据ready
    );
    
    //AXI写主�?
    axi_master_wr 
    #(.AXI_WIDTH     (AXI_WIDTH     ),  //AXI总线读写数据位宽
      .AXI_AXSIZE    (AXI_AXSIZE    ),  //AXI总线的axi_axsize, �?要与AXI_WIDTH对应
      .AXI_WSTRB_W   (AXI_WSTRB_W   ))   //axi_wstrb的位�?, AXI_WIDTH/8
    axi_master_wr_inst(
        //用户�?
        .clk              (clk              ),
        .rst_n            (rst_n            ),
        .wr_start         (axi_wr_start     ), //�?始写信号
        .wr_addr          (axi_wr_addr      ), //写首地址
        .wr_data          (axi_wr_data      ),
        .wr_len           (axi_wr_len       ), //突发传输长度
        .wr_done          (axi_wr_done      ), //写完成标�?
        .m_axi_w_handshake(axi_writing      ), //写�?�道成功握手
        .wr_ready         (axi_wr_ready     ), //写准备信�?,拉高时可以发起wr_start
        
        //AXI4写地�?通道
        .m_axi_awid       (m_axi_awid       ), 
        .m_axi_awaddr     (m_axi_awaddr     ),
        .m_axi_awlen      (m_axi_awlen      ), //突发传输长度
        .m_axi_awsize     (m_axi_awsize     ), //突发传输大小(Byte)
        .m_axi_awburst    (m_axi_awburst    ), //突发类型
        .m_axi_awlock     (m_axi_awlock     ), 
        .m_axi_awcache    (m_axi_awcache    ), 
        .m_axi_awprot     (m_axi_awprot     ),
        .m_axi_awqos      (m_axi_awqos      ),
        .m_axi_awvalid    (m_axi_awvalid    ), //写地�?valid
        .m_axi_awready    (m_axi_awready    ), //从机发出的写地址ready
                                            
        //写数据�?�道                        
        .m_axi_wdata      (m_axi_wdata      ), //写数�?
        .m_axi_wstrb      (m_axi_wstrb      ), //写数据有效字节线
        .m_axi_wlast      (m_axi_wlast      ), //�?后一个数据标�?
        .m_axi_wvalid     (m_axi_wvalid     ), //写数据有效标�?
        .m_axi_wready     (m_axi_wready     ), //从机发出的写数据ready
                                            
        //写响应�?�道                        
        .m_axi_bid        (m_axi_bid        ),
        .m_axi_bresp      (m_axi_bresp      ), //响应信号,表征写传输是否成�?
        .m_axi_bvalid     (m_axi_bvalid     ), //响应信号valid标志
        .m_axi_bready     (m_axi_bready     )  //主机响应ready信号
    );
endmodule
