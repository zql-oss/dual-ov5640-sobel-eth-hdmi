`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer: zql
// Email: lauchinyuan@yeah.net
// Create Date: 2025/01/06 13:09:46
// Module Name: axi_ctrl
// Description: AXI控制�?, 依据AXI读写主机发来的读写信�?, 自动产生AXI读写请求、读写地坿以及读写突发长�?
//////////////////////////////////////////////////////////////////////////////////

module axi_ctrl
    #(parameter FIFO_WR_WIDTH = 'd256,   //用户端FIFO读写位宽
                FIFO_RD_WIDTH = 'd256,
                AXI_WIDTH     = 'd256
    )
    (
        input   wire                        clk             , //AXI读写主机时钟
        input   wire                        rst_n           , 
        input                               clk_100         ,  //用户时钟
        input   wire                        pingpang        ,//乒乓操作
        // RGB 数据输入1
        input                               datain_valid_1  ,  //数据有效使能信号
        input        [15:0]                 datain_1        ,  //有效数据
        // RGB 数据输入2
        input                               datain_valid_2  ,  //数据有效使能信号
        input        [15:0]                 datain_2        ,  //有效数据
        // 额外同步信号
        input   wire                        rd_load_1       , // RD FIFO加载信号
        input   wire                        rd_load_2       , // RD FIFO加载信号
        input   wire                        wr_load_1       , // WR FIFO加载信号
        input   wire                        wr_load_2       , // WR FIFO加载信号        
        //用户�? �?                   
        input   wire                        wr_clk_1          , //写FIFO写时�?
        input   wire                        wr_rst_1          , //写复�?,模块中是同步复位
        input   wire                        wr_clk_2          , //写FIFO写时�?
        input   wire                        wr_rst_2          , //写复�?,模块中是同步复位
        input   wire [28:0]                 wr_beg_addr_1     , //写起始地�?
        input   wire [28:0]                 wr_end_addr_1     , //写终止地�?
        input   wire [28:0]                 wr_beg_addr_2     , //写起始地�?
        input   wire [28:0]                 wr_end_addr_2     , //写终止地�?
        input   wire [7:0]                  wr_burst_len    , //写突发长�?
        //用户�? �?
        input   wire                        rd_clk_1          , //读FIFO读时�?
        input   wire                        rd_rst_1          , //读复�?,模块中是同步复位
        input   wire                        rd_clk_2          , //读FIFO读时�?
        input   wire                        rd_rst_2          , //读复�?,模块中是同步复位
        input   wire                        rd_mem_enable   , //读存储器使能,防止存储器未写先�?
        input   wire [28:0]                 rd_beg_addr_1     , //读起始地�?
        input   wire [28:0]                 rd_end_addr_1     , //读终止地�?
        input   wire [28:0]                 rd_beg_addr_2     , //读起始地�?
        input   wire [28:0]                 rd_end_addr_2     , //读终止地�?
        input   wire [7:0]                  rd_burst_len    , //读突发长�?
        input   wire                        rd_en           , //读FIFO读请�?
        
        input   wire [12:0]                 h_disp          ,
        //output  wire [FIFO_RD_WIDTH-1:0]    rd_data         , //读FIFO读数�?
        output  wire                        rd_valid        , //读FIFO可读标志,表示读FIFO中有数据可以对外输出
        output       [15:0]                 pic_data        ,    //有效数据 
        //写AXI主机
        input   wire                        axi_writing     , //AXI主机写正在进�?   =m_axi_w_handshake
        input   wire                        axi_wr_ready    , //AXI主机写准备好
        output  reg                         axi_wr_start    , //AXI主机写请�?
        output  wire [AXI_WIDTH-1:0]        axi_wr_data     , //从写FIFO中读取的数据,写入AXI写主�?
        output  wire  [28:0]                axi_wr_addr     , //AXI主机写地�?
        output  wire [7:0]                  axi_wr_len      , //AXI主机写突发长�?
        input   wire                        axi_wr_done     , //AXI主机完成丿次写操�?
                        
        //读AXI主机                
        input   wire                        axi_reading     , //AXI主机读正在进�?   =m_axi_r_handshake
        input   wire                        axi_rd_ready    , //AXI主机读准备好
        output  reg                         axi_rd_start    , //AXI主机读请�?
        input   wire [AXI_WIDTH-1:0]        axi_rd_data     , //从AXI读主机读到的数据,写入读FIFO                       1
        output  wire [28:0]                 axi_rd_addr     , //AXI主机读地�?
        output  wire [7:0]                  axi_rd_len      , //AXI主机读突发长�? 
        input   wire                        axi_rd_done       //AXI主机完成丿次写操�?
        
    );
     //reg define
     reg  [255:0] datain_t_1        ;  //�?16bit输入源数据移位拼接得�?
     reg  [255:0] datain_t_2        ;  //�?16bit输入源数据移位拼接得�?
     reg  [4:0]   i_d0              ;
     reg  [4:0]   i_d1              ;
     reg  [15:0]  rd_load_d         ;  //由输出源场信号移位拼接得�?  
     reg  [15:0]  rd_load_dd        ;  //由输出源场信号移位拼接得�?     
     reg  [6:0]   byte_cnt_1        ;  //写数据移位计数器
     reg  [6:0]   byte_cnt_2        ;  //写数据移位计数器
     reg  [255:0] data_1            ;  //rfifo输出数据打拍得到
     reg  [255:0] data_2            ;  //rfifo输出数据打拍得到
     reg  [15:0]  pic_data_1        ;  //有效数据
     reg  [15:0]  pic_data_2        ;  //有效数据 
     reg  [4:0]   i                 ;  //读数据移位计数器
     reg  [4:0]   ii                ;  //读数据移位计数器
     reg          wr_load_d0        ;
     reg          wr_load_dd0       ;
     reg          rd_load_d0        ;
     reg          rd_load_d2        ;
     reg          rdfifo_rst_h      ;  //rfifo复位信号，高有效
     reg          rdfifo_rst_h2     ;  //rfifo复位信号，高有效
     reg   [15:0] wr_load_d         ;    // Declare as 16-bit vector
     reg   [15:0] wr_load_dd        ;   // Declare as 16-bit vector

     reg          wfifo_rst_h0       ;  //wfifo复位信号，高有效
     reg          wfifo_rst_h1       ;  //wfifo复位信号，高有效
     reg          wfifo_wren_1      ;  //wfifo写使能信�?
     reg          wfifo_wren_2      ;  //wfifo写使能信�?     
     //wire define 
    // wire [255:0] rfifo_dout        ;  //rfifo输出数据    
     wire [255:0] wfifo_din_1         ;  //wfifo写数�?
     wire [255:0] wfifo_din_2         ;  //wfifo写数�?
     wire [15:0]  dataout_1[0:15]     ;  //定义输出数据的二维数�?
     wire [15:0]  dataout_2[0:15]     ;  //定义输出数据的二维数�?
     wire         rfifo_rden_1        ;  //rfifo的读使能
     wire         rfifo_rden_2        ;  //rfifo的读使能         
    //FIFO数据数量计数�?   
    wire [10:0]  cnt_rd_fifo_wrport_1      ;  //读FIFO写端�?(对接AXI读主�?)数据数量
    wire [10:0]  cnt_wr_fifo_rdport_1      ;  //写FIFO读端�?(对接AXI写主�?)数据数量    
    
    wire        rd_fifo_empty_1           ;  //读FIFO空标�?
    wire        rd_fifo_wr_rst_busy_1     ;  //读FIFO正在初始�?,此时先不向SDRAM发出读取请求, 否则将有数据丢失
        //FIFO数据数量计数�?   
    wire [10:0]  cnt_rd_fifo_wrport_2      ;  //读FIFO写端�?(对接AXI读主�?)数据数量
    wire [10:0]  cnt_wr_fifo_rdport_2      ;  //写FIFO读端�?(对接AXI写主�?)数据数量    
    
    wire        rd_fifo_empty_2           ;  //读FIFO空标�?
    wire        rd_fifo_wr_rst_busy_2     ;  //读FIFO正在初始�?,此时先不向SDRAM发出读取请求, 否则将有数据丢失        
    //真实的读写突发长�?
    wire  [7:0] real_wr_len             ;  //真实的写突发长度,是wr_burst_len+1
    wire  [7:0] real_rd_len             ;  //真实的读突发长度,是rd_burst_len+1
    
    //突发地址增量, 每次进行丿次连续突发传输地坿的增�?, 在外边计�?, 方便后续复用
    wire  [28:0]burst_wr_addr_inc       ;
    wire  [28:0]burst_rd_addr_inc       ;
    
    //复位信号处理(异步复位同步释放)
    reg     rst_n_sync  ;  //同步释放处理后的rst_n
    reg     rst_n_d1    ;  //同步释放处理rst_n, 同步器第丿级输凿 

    //读复位同步到clk
    reg     rd_rst_sync ;  //读复位打两拍
    reg     rd_rst_d1   ;  //读复位打丿拿
    wire   [255:0] rfifo_dout_1           ;  //rfifo输出数据 
    wire   [255:0] rfifo_dout_2           ;  //rfifo输出数据 
    wire    rd_en_1;
    wire    rd_en_2;
/*
---------------------------------------------------------------------------------------------

*/
//*****************************************************
//像素显示请求信号切换，即显示器左侧请求FIFO1显示，右侧请求FIFO2显示
reg [12:0] rd_cnt;
assign rd_en_1  = (rd_cnt <= h_disp[12:1]-1) ? rd_en :1'b0;//右移/2
assign rd_en_2  = (rd_cnt <= h_disp[12:1]-1) ? 1'b0 :rd_en;
//assign rd_en_1  = rd_en ;//右移/2
//assign rdata_req_1  =  (rd_cnt <= h_disp[12:0]-1) ? rdata_req :1'b0;//
//像素在显示器显示位置的切换，即显示器左侧显示FIFO0,右侧显示FIFO1
assign pic_data = (rd_cnt <= h_disp[12:1]) ? pic_data_1 : pic_data_2;
//assign pic_data = pic_data_1 ;


//对读请求信号计数
always @(posedge rd_clk_1 or negedge rst_n) begin
    if(!rst_n)
        rd_cnt <= 13'd0;
    else if(rd_en)
        rd_cnt <= rd_cnt + 1'b1;
    else
        rd_cnt <= 13'd0;
end

//*****************************************************
//**                    main code
//*****************************************************  
//rfifo输出的数据存到二维数�?
    assign dataout_1[0]  = data_1[255:240];
    assign dataout_1[1]  = data_1[239:224];
    assign dataout_1[2]  = data_1[223:208];
    assign dataout_1[3]  = data_1[207:192];
    assign dataout_1[4]  = data_1[191:176];
    assign dataout_1[5]  = data_1[175:160];
    assign dataout_1[6]  = data_1[159:144];
    assign dataout_1[7]  = data_1[143:128];
    assign dataout_1[8]  = data_1[127:112];
    assign dataout_1[9]  = data_1[111:96];
    assign dataout_1[10] = data_1[95:80];
    assign dataout_1[11] = data_1[79:64];
    assign dataout_1[12] = data_1[63:48];
    assign dataout_1[13] = data_1[47:32];
    assign dataout_1[14] = data_1[31:16];
    assign dataout_1[15] = data_1[15:0];
    assign wfifo_din_1 = datain_t_1 ;
    //移位寄存器计满时，从rfifo读出丿个数捿
    assign rfifo_rden_1 = (rd_en_1&&(i==15)) ? 1'b1  :  1'b0;   
    //assign wr_data   = wfifo_din;
//16位数据转256位RGB565数据        
always @(posedge wr_clk_1 or negedge rst_n) begin
    if(!rst_n) begin
        datain_t_1 <= 0;
        byte_cnt_1 <= 0;
    end
    else if(datain_valid_1) begin
        if(byte_cnt_1 == 15)begin
            byte_cnt_1 <= 0;
            datain_t_1 <= {datain_t_1[239:0],datain_1};
        end
        else begin
            byte_cnt_1 <= byte_cnt_1 + 1;
            datain_t_1 <= {datain_t_1[239:0],datain_1};
        end
    end
    else begin
        byte_cnt_1 <= byte_cnt_1;
        datain_t_1 <= datain_t_1;
    end    
end 

//wfifo写使能产�?
always @(posedge wr_clk_1 or negedge rst_n) begin
    if(!rst_n) 
        wfifo_wren_1 <= 0;
    else if(wfifo_wren_1 == 1)
        wfifo_wren_1 <= 0;
    else if(byte_cnt_1 == 15 && datain_valid_1 )  //输入源数据传�?16次，写使能拉高一�?
        wfifo_wren_1 <= 1;
    else 
        wfifo_wren_1 <= 0;
 end

always @(posedge rd_clk_1 or negedge rst_n) begin
    if(!rst_n)
        data_1 <= 256'b0;
    else 
        data_1 <= rfifo_dout_1; 
end     

//对rfifo出来�?256bit数据拆解�?16�?16bit数据
always @(posedge rd_clk_1 or negedge rst_n) begin
    if(!rst_n) begin
        pic_data_1 <= 16'b0;
        i <=0;
        i_d0 <= 0;
    end
    else if(rd_en_1) begin
        if(i == 15)begin
            pic_data_1 <= dataout_1[i_d0];
            i <= 0;
            i_d0 <= i;
        end
        else begin
            pic_data_1 <= dataout_1[i_d0];
            i <= i + 1;
            i_d0 <= i;
        end
    end 
    else begin
        pic_data_1 <= pic_data_1;
        i <=0;
        i_d0 <= 0;
    end
end  


//*****************************************************
//**                    main code
//*****************************************************  
//rfifo输出的数据存到二维数�?
    assign dataout_2[0]  = data_2[255:240];
    assign dataout_2[1]  = data_2[239:224];
    assign dataout_2[2]  = data_2[223:208];
    assign dataout_2[3]  = data_2[207:192];
    assign dataout_2[4]  = data_2[191:176];
    assign dataout_2[5]  = data_2[175:160];
    assign dataout_2[6]  = data_2[159:144];
    assign dataout_2[7]  = data_2[143:128];
    assign dataout_2[8]  = data_2[127:112];
    assign dataout_2[9]  = data_2[111:96];
    assign dataout_2[10] = data_2[95:80];
    assign dataout_2[11] = data_2[79:64];
    assign dataout_2[12] = data_2[63:48];
    assign dataout_2[13] = data_2[47:32];
    assign dataout_2[14] = data_2[31:16];
    assign dataout_2[15] = data_2[15:0];
    assign wfifo_din_2 = datain_t_2 ;
    //移位寄存器计满时，从rfifo读出丿个数捿
    assign rfifo_rden_2 = (rd_en_2&&(ii==15)) ? 1'b1  :  1'b0;   
    //assign wr_data   = wfifo_din;
//16位数据转256位RGB565数据        
always @(posedge wr_clk_2 or negedge rst_n) begin
    if(!rst_n) begin
        datain_t_2 <= 0;
        byte_cnt_2 <= 0;
    end
    else if(datain_valid_2) begin
        if(byte_cnt_2 == 15)begin
            byte_cnt_2 <= 0;
            datain_t_2 <= {datain_t_2[239:0],datain_2};
        end
        else begin
            byte_cnt_2 <= byte_cnt_2 + 1;
            datain_t_2 <= {datain_t_2[239:0],datain_2};
        end
    end
    else begin
        byte_cnt_2 <= byte_cnt_2;
        datain_t_2 <= datain_t_2;
    end    
end 

//wfifo写使能产�?
always @(posedge wr_clk_2 or negedge rst_n) begin
    if(!rst_n) 
        wfifo_wren_2 <= 0;
    else if(wfifo_wren_2 == 1)
        wfifo_wren_2 <= 0;
    else if(byte_cnt_2 == 15 && datain_valid_2 )  //输入源数据传�?16次，写使能拉高一�?
        wfifo_wren_2 <= 1;
    else 
        wfifo_wren_2 <= 0;
 end

always @(posedge rd_clk_2 or negedge rst_n) begin
    if(!rst_n)
        data_2 <= 256'b0;
    else 
        data_2 <= rfifo_dout_2; 
end     

//对rfifo出来�?256bit数据拆解�?16�?16bit数据
always @(posedge rd_clk_2 or negedge rst_n) begin
    if(!rst_n) begin
        pic_data_2 <= 16'b0;
        ii <=0;
        i_d1 <= 0;
    end
    else if(rd_en_2) begin
        if(ii == 15)begin
            pic_data_2 <= dataout_2[i_d1];
            ii <= 0;
            i_d1 <= ii;
        end
        else begin
            pic_data_2 <= dataout_2[i_d1];
            ii <= ii + 1;
            i_d1 <= ii;
        end
    end 
    else begin
        pic_data_2 <= pic_data_2;
        ii <=0;
        i_d1 <= 0;
    end
end  


//-----------------------------------------------------------------------------------------
always @(posedge clk_100 or negedge rst_n) begin
    if(!rst_n)
        rd_load_d0 <= 1'b0;
    else
        rd_load_d0 <= rd_load_1;      
end 

//对输出源场信号进行移位寄�?
always @(posedge clk_100 or negedge rst_n) begin
    if(!rst_n)
        rd_load_d <= 1'b0;
    else
        rd_load_d <= {rd_load_d[14:0],rd_load_d0};       //延迟�?16周期，对�?256bit拆分16bit�?�?�?16周期
end 

//产生丿段复位电平，满足fifo复位时序  
always @(posedge clk_100 or negedge rst_n) begin
    if(!rst_n)
        rdfifo_rst_h <= 1'b0;
    else if(rd_load_d[0] && !rd_load_d[14])
        rdfifo_rst_h <= 1'b1;   
    else
        rdfifo_rst_h <= 1'b0;              
end  

//----------------------------------------------------------------
always @(posedge clk_100 or negedge rst_n) begin
    if(!rst_n)
        rd_load_d2 <= 1'b0;
    else
        rd_load_d2 <= rd_load_2;      
end 

//对输出源场信号进行移位寄�?
always @(posedge clk_100 or negedge rst_n) begin
    if(!rst_n)
        rd_load_dd <= 1'b0;
    else
        rd_load_dd <= {rd_load_dd[14:0],rd_load_d2};       //延迟�?16周期，对�?256bit拆分16bit承霿�?16周期
end 

//产生丿段复位电平，满足fifo复位时序  
always @(posedge clk_100 or negedge rst_n) begin
    if(!rst_n)
        rdfifo_rst_h2 <= 1'b0;
    else if(rd_load_dd[0] && !rd_load_dd[14])
        rdfifo_rst_h2 <= 1'b1;   
    else
        rdfifo_rst_h2 <= 1'b0;              
end  

//对输入源场信号进行移位寄�?
 always @(posedge wr_clk_1 or negedge rst_n) begin
    if(!rst_n)begin
        wr_load_d0 <= 1'b0;
        wr_load_d  <= 16'b0;        
    end     
    else begin
        wr_load_d0 <= wr_load_1;
        wr_load_d <= {wr_load_d[14:0],wr_load_d0};      
    end                 
end  

//产生�?段复位电平，满足fifo复位时序 
 always @(posedge wr_clk_1 or negedge rst_n) begin
    if(!rst_n)
      wfifo_rst_h0 <= 1'b0;          
    else if(wr_load_d[0] && !wr_load_d[15])
      wfifo_rst_h0 <= 1'b1;       
    else
      wfifo_rst_h0 <= 1'b0;                      
end   


//对输入源场信号进行移位寄�?  cam
 always @(posedge wr_clk_2 or negedge rst_n) begin
    if(!rst_n)begin
        wr_load_dd0 <= 1'b0;
        wr_load_dd  <= 16'b0;        
    end     
    else begin
        wr_load_dd0 <= wr_load_2;
        wr_load_dd <= {wr_load_dd[14:0],wr_load_dd0};      
    end                 
end  
//产生丿段复位电平，满足fifo复位时序 
 always @(posedge wr_clk_2 or negedge rst_n) begin
    if(!rst_n)
      wfifo_rst_h1 <= 1'b0;          
    else if(wr_load_dd[0] && !wr_load_dd[15])
      wfifo_rst_h1 <= 1'b1;       
    else
      wfifo_rst_h1 <= 1'b0;                      
end 
//------------------------------------------------------------------------------------------------------------------------
    //真实的读写突发长�?
    assign real_wr_len = wr_burst_len + 8'd1;
    assign real_rd_len = rd_burst_len + 8'd1;
    
    //突发地址增量, 右移3�?
    assign burst_wr_addr_inc = real_wr_len * AXI_WIDTH >> 3;
    assign burst_rd_addr_inc = real_rd_len * AXI_WIDTH >> 3;
    
    
    //向AXI主机发出的读写突发长�?
    assign axi_wr_len = wr_burst_len;
    assign axi_rd_len = rd_burst_len;
    assign rd_valid = (~rd_fifo_empty_1) || (~rd_fifo_empty_2);

//写burst请求产生
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        axi_wr_start<=1'b0;
    end else if(~axi_wr_ready) begin  //axi_wr_ready�?,代表AXI写主机正在进行数据发�?, start信号已经被响�?
            axi_wr_start <= 1'b0;
    //fifo数据长度大于�?次突发长度并且axi写空�?
    end else if((cnt_wr_fifo_rdport_1 > wr_burst_len-2 || cnt_wr_fifo_rdport_2 > wr_burst_len-2)&& axi_wr_ready ) 
    begin 
        axi_wr_start<=1'b1;      
    end
    else begin
        axi_wr_start<=1'b0;
    end

end
//读burst请求产生
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        axi_rd_start<=1'b0;
    end
    //fifo可写长度大于�?次突发长度并且axi读空闲，fifo总长�?1024
    else if((rd_mem_enable && (cnt_rd_fifo_wrport_1 < 500 - rd_burst_len || cnt_rd_fifo_wrport_2 < 500 - rd_burst_len)) && axi_rd_ready)
    begin
        axi_rd_start<=1'b1; 
    end
    else begin
        axi_rd_start<=1'b0;
    end
end
//------------------------------------------------------------------------------------
//读写地址复位打拍寄存�?
reg wr_rst_reg1;
reg wr_rst_reg2;
reg wr_rst_reg11;
reg wr_rst_reg22;
reg rd_rst_reg1;
reg rd_rst_reg2;
//对写复位信号的跨时钟域打2�?
reg rd_judge_fifo_flag   ;
wire [255:0] axi_rd_data_1;
wire [255:0] axi_rd_data_2;
assign axi_rd_data_1 = (~rd_judge_fifo_flag ) ? axi_rd_data : 256'b0;
assign axi_rd_data_2 = (rd_judge_fifo_flag) ? axi_rd_data : 256'b0;
//assign axi_rd_data_1 =  axi_rd_data ;
//assign axi_rd_data_2 =  256'b0;

wire   axi_reading_1 ;
wire   axi_reading_2 ;
assign axi_reading_1 = (~rd_judge_fifo_flag ) ? axi_reading : 1'b0  ;
assign axi_reading_2 = (rd_judge_fifo_flag) ? axi_reading : 1'b0  ;
//assign axi_reading_1 =  axi_reading  ;
//assign axi_reading_2 =  1'b0  ;
reg wr_judge_fifo_flag   ;
wire [255:0] axi_wr_data_1;
wire [255:0] axi_wr_data_2;
assign axi_wr_data = (~wr_judge_fifo_flag ) ? axi_wr_data_1 : axi_wr_data_2;
//assign axi_wr_data =  axi_wr_data_1 ;

wire   axi_writing_1 ;
wire   axi_writing_2 ;
assign axi_writing_1 = (~wr_judge_fifo_flag ) ? axi_writing : 1'b0  ;
assign axi_writing_2 = (wr_judge_fifo_flag) ? axi_writing : 1'b0  ;
//assign axi_writing_1 = axi_writing ;
//assign axi_writing_2 = 1'b0 ;
//------------------------------------------------------------------------------------

    // Correcting WR_FIFO flag management logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_judge_fifo_flag <= 1'b0;
        end else if ((cnt_wr_fifo_rdport_1 > wr_burst_len-2 && cnt_wr_fifo_rdport_2 < wr_burst_len - 2)&& axi_wr_ready) begin
                wr_judge_fifo_flag <= 1'b0;
            end else if ((cnt_wr_fifo_rdport_1 < wr_burst_len-2 && cnt_wr_fifo_rdport_2 > wr_burst_len - 2)&& axi_wr_ready) begin
                wr_judge_fifo_flag <= 1'b1;
            end else if (cnt_wr_fifo_rdport_1 > wr_burst_len - 2 && cnt_wr_fifo_rdport_2 > wr_burst_len - 2) begin
                wr_judge_fifo_flag <= ~wr_judge_fifo_flag;
            end
        end


    // Correcting RD_FIFO flag management logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_judge_fifo_flag <= 1'b0;
        end else if ((rd_mem_enable && (cnt_rd_fifo_wrport_1 < 500 - rd_burst_len && cnt_rd_fifo_wrport_2 > 500 - rd_burst_len)) && axi_rd_ready) begin
                rd_judge_fifo_flag <= 1'b0;
            end else if ((rd_mem_enable && (cnt_rd_fifo_wrport_1 > 500 - rd_burst_len && cnt_rd_fifo_wrport_2 < 500 - rd_burst_len)) && axi_rd_ready) begin
                rd_judge_fifo_flag <= 1'b1;
            end else if ((rd_mem_enable && (cnt_rd_fifo_wrport_1 < 500 - rd_burst_len && cnt_rd_fifo_wrport_2 < 500 - rd_burst_len)) && axi_rd_ready) begin
                rd_judge_fifo_flag <= ~rd_judge_fifo_flag;
            end
        end


reg  [28:0] axi_wr_addr_1;
reg  [28:0] axi_wr_addr_2;
reg  [28:0] axi_rd_addr_1;
reg  [28:0] axi_rd_addr_2;
//assign axi_wr_addr   = axi_wr_addr_1;
//assign axi_rd_addr   = axi_rd_addr_1;
// 写区域选择
assign axi_wr_addr = (wr_judge_fifo_flag == 1'b0) ? axi_wr_addr_1 : axi_wr_addr_2;

// 读区域选择
assign axi_rd_addr = (rd_judge_fifo_flag == 1'b0) ? axi_rd_addr_1 : axi_rd_addr_2;
//------------------------------------------------------------------------------------
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        wr_rst_reg1<=1'b0;
        wr_rst_reg2<=1'b0;
    end
    else begin
        wr_rst_reg1<=wr_load_1;
        wr_rst_reg2<=wr_rst_reg1;
    end

end
//------------------------------------------------------------------------------------
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        wr_rst_reg11<=1'b0;
        wr_rst_reg22<=1'b0;
    end
    else begin
        wr_rst_reg11<=wr_load_2;
        wr_rst_reg22<=wr_rst_reg11;
    end

end
reg rd_rst_reg11;
reg rd_rst_reg22;
//对读复位信号的跨时钟域打2�?
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        rd_rst_reg1<=1'b0;
        rd_rst_reg2<=1'b0;
    end
    else begin
        rd_rst_reg1<=rd_load_1;
        rd_rst_reg2<=rd_rst_reg1;
    end

end
//对读复位信号的跨时钟域打2�?
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        rd_rst_reg11<=1'b0;
        rd_rst_reg22<=1'b0;
    end
    else begin
        rd_rst_reg11<=rd_load_2;
        rd_rst_reg22<=rd_rst_reg11;
    end

end

reg pingpang_reg_1;
reg pingpang_reg_2;
//完成�?次突发对地址进行相加
//相加地址长度=突发长度x8,64位等�?8字节
//128*8=1024
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        axi_wr_addr_1<=wr_beg_addr_1;
        pingpang_reg_1<=1'b0;
    end
    //写复位信号上升沿
    else if(wr_rst_reg1&(~wr_rst_reg2)) begin
        axi_wr_addr_1<=wr_beg_addr_1;
    end 
    else if(axi_wr_done==1'b1 && wr_judge_fifo_flag == 1'b0) begin
        axi_wr_addr_1<=axi_wr_addr_1+burst_wr_addr_inc;
        //判断是否是乒乓操�?
        if(pingpang==1'b1) begin
        //结束地址�?2倍的接受地址，有两块区域
            if(axi_wr_addr_1>=((wr_end_addr_1-wr_beg_addr_1)*2+wr_beg_addr_1-burst_wr_addr_inc)) 
            begin
                axi_wr_addr_1<=wr_beg_addr_1;
            end
            //根据地址，pingpang_reg�?0或�??1
            //用于指示读操作与写操作地�?不冲�?
            if(axi_wr_addr_1<wr_end_addr_1) begin
                pingpang_reg_1<=1'b0;  //第一�?
            end
            else begin
                pingpang_reg_1<=1'b1;  //第二�?
            end
        
        end
        
        //非乒乓操�?
        else begin
            if(axi_wr_addr_1>=(wr_end_addr_1-burst_wr_addr_inc)) 
            begin
                axi_wr_addr_1<=wr_beg_addr_1;
            end
        end
    end
    else begin
        axi_wr_addr_1<=axi_wr_addr_1;
    end

end
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        axi_wr_addr_2<=wr_beg_addr_2;
        pingpang_reg_2<=1'b0;
    end
    //写复位信号上升沿
    else if(wr_rst_reg11&(~wr_rst_reg22)) begin
        axi_wr_addr_2<=wr_beg_addr_2;
    end 
    else if(axi_wr_done==1'b1 && wr_judge_fifo_flag == 1'b1) begin
        axi_wr_addr_2<=axi_wr_addr_2+burst_wr_addr_inc;
        //判断是否是乒乓操�?
        if(pingpang==1'b1) begin
        //结束地址�?2倍的接受地址，有两块区域
            if(axi_wr_addr_2>=((wr_end_addr_2-wr_beg_addr_2)*2+wr_beg_addr_2-burst_wr_addr_inc)) 
            begin
                axi_wr_addr_2<=wr_beg_addr_2;
            end
            //根据地址，pingpang_reg�?0或迿1
            //用于指示读操作与写操作地坿不冲窿
            if(axi_wr_addr_2<wr_end_addr_2) begin
                pingpang_reg_2<=1'b0;  //第一�?
            end
            else begin
                pingpang_reg_2<=1'b1;  //第二�?
            end
        
        end
        //非乒乓操�?
        else begin
            if(axi_wr_addr_2>=(wr_end_addr_2-burst_wr_addr_inc)) 
            begin
                axi_wr_addr_2<=wr_beg_addr_2;
            end
        end
    end
    else begin
        axi_wr_addr_2<=axi_wr_addr_2;
    end

end


//完成�?次突发对地址进行相加
//相加地址长度=突发长度x8,64位等�?8字节
//128*8=1024
always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        if(pingpang==1'b1) axi_rd_addr_1<=rd_end_addr_1;
        else axi_rd_addr_1<=rd_beg_addr_1;
    end
     else if(rd_rst_reg1&(~rd_rst_reg2)) begin
        axi_rd_addr_1<=rd_beg_addr_1;
    end 
    else if(axi_rd_done==1'b1  && rd_judge_fifo_flag == 1'b0) begin
        axi_rd_addr_1<=axi_rd_addr_1+burst_rd_addr_inc;//地址累加
        //乒乓操作
         if(pingpang==1'b1) begin
           //到达结束地址 
           if((axi_rd_addr_1==(rd_end_addr_1-burst_rd_addr_inc))||
              (axi_rd_addr_1==((rd_end_addr_1-rd_beg_addr_1)*2+rd_beg_addr_1-burst_rd_addr_inc))) 
           begin
                //根据写指示地�?信号，对读信号进行复�?
               if(pingpang_reg_1==1'b1) axi_rd_addr_1<=rd_beg_addr_1;
               else axi_rd_addr_1<=rd_end_addr_1;
           end
                    
        end
        else begin  //非乒乓操�?
            if(axi_rd_addr_1>=(rd_end_addr_1-burst_rd_addr_inc)) 
            begin
            axi_rd_addr_1<=rd_beg_addr_1;
            end
        end
    end
    else begin
        axi_rd_addr_1<=axi_rd_addr_1;
    end

end

always@(posedge clk or posedge rst_n) begin
    if(~rst_n)begin
        if(pingpang==1'b1) axi_rd_addr_2<=rd_end_addr_2;
        else axi_rd_addr_2<=rd_beg_addr_2;
    end
     else if(rd_rst_reg11&(~rd_rst_reg22)) begin
        axi_rd_addr_2<=rd_beg_addr_2;
    end 
    else if(axi_rd_done==1'b1 && rd_judge_fifo_flag == 1'b1) begin
        axi_rd_addr_2<=axi_rd_addr_2+burst_rd_addr_inc;//地址累加
        //乒乓操作
         if(pingpang==1'b1) begin
           //到达结束地址 
           if((axi_rd_addr_2==(rd_end_addr_2-burst_rd_addr_inc))||
              (axi_rd_addr_2==((rd_end_addr_2-rd_beg_addr_2)*2+rd_beg_addr_2-burst_rd_addr_inc))) 
           begin
                //根据写指示地坿信号，对读信号进行复使
               if(pingpang_reg_2==1'b1) axi_rd_addr_2<=rd_beg_addr_2;
               else axi_rd_addr_2<=rd_end_addr_2;
           end
                    
        end
        else begin  //非乒乓操�?
            if(axi_rd_addr_2>=(rd_end_addr_2-burst_rd_addr_inc)) 
            begin
            axi_rd_addr_2<=rd_beg_addr_2;
            end
        end
    end
    else begin
        axi_rd_addr_2<=axi_rd_addr_2;
    end

end
    //读FIFO, 从SDRAM中读出的数据先暂存于�?
    //使用FIFO IP�?
	rd_fifo rd_fifo256x512_inst1 (
        .rst                (~rst_n | rdfifo_rst_h      ),  //读复位时霿要复位读FIFO
        //.rst                (~rst_n                     ),
        .wr_clk             (clk_100                    ),  //写端口时钟是AXI主机时钟, 从axi_master_rd模块写入数据
        .rd_clk             (rd_clk_1                     ),  //读端口时�?
        .din                (axi_rd_data_1                ),  //从axi_master_rd模块写入数据
        .wr_en              (axi_reading_1                ),  //axi_master_rd正在读时,FIFO也在写入
        .rd_en              (rfifo_rden_1                 ),  //读FIFO读使�?    //-----------------读不用改------------------
        .dout               (rfifo_dout_1                 ),  //读FIFO读取的数�?
        .full               (                           ),  
        .almost_full        (                           ),  
        .empty              (rd_fifo_empty_1              ),  
        .almost_empty       (                           ),  
        .rd_data_count      (                           ),  
        .wr_data_count      (cnt_rd_fifo_wrport_1         ),  //读FIFO写端�?(对接AXI读主�?)数据数量
        .wr_rst_busy        (rd_fifo_wr_rst_busy_1        ),     
        .rd_rst_busy        (                           )      
);



    //写FIFO, 待写入SDRAM的数据先暂存于此
    //使用FIFO IP�?
    wr_fifo wr_fifo256x512_inst1 (
        .rst                (~rst_n | wfifo_rst_h0    ),  
        //.rst                (~rst_n        ),
        .wr_clk             (wr_clk_1                     ),  //写端口时�?
        .rd_clk             (clk_100                    ),  //读端口时钟是AXI主机时钟, AXI写主机读取数�?
        .din                (wfifo_din_1                  ),  
        .wr_en              (wfifo_wren_1                 ),  
        .rd_en              (axi_writing_1                ),  //axi_master_wr正在写时,从写FIFO中不断读出数�?
        .dout               (axi_wr_data_1                ),  //读出的数据作为AXI写主机的输入数据
        .full               (                           ),  
        .almost_full        (                           ),  
        .empty              (                           ),  
        .almost_empty       (                           ),  
        .rd_data_count      (cnt_wr_fifo_rdport_1         ),  //写FIFO读端�?(对接AXI写主�?)数据数量
        .wr_data_count      (                           ),  
        .wr_rst_busy        (                           ),  
        .rd_rst_busy        (                           )   
    );


    //读FIFO, 从SDRAM中读出的数据先暂存于�?
    //使用FIFO IP�?
	rd_fifo rd_fifo256x512_inst2 (
        .rst                (~rst_n | rdfifo_rst_h2      ),  //读复位时霿要复位读FIFO
        //.rst                (1'b0                       ),
        .wr_clk             (clk_100                    ),  //写端口时钟是AXI主机时钟, 从axi_master_rd模块写入数据
        .rd_clk             (rd_clk_2                     ),  //读端口时�?
        .din                (axi_rd_data_2              ),  //从axi_master_rd模块写入数据
        .wr_en              (axi_reading_2              ),  //axi_master_rd正在读时,FIFO也在写入
        .rd_en              (rfifo_rden_2                 ),  //读FIFO读使�?
        .dout               (rfifo_dout_2                 ),  //读FIFO读取的数�?
        .full               (                           ),  
        .almost_full        (                           ),  
        .empty              (rd_fifo_empty_2              ),  
        .almost_empty       (                           ),  
        .rd_data_count      (                           ),  
        .wr_data_count      (cnt_rd_fifo_wrport_2         ),  //读FIFO写端�?(对接AXI读主�?)数据数量
        .wr_rst_busy        (rd_fifo_wr_rst_busy_2        ),     
        .rd_rst_busy        (                           )      
);



    //写FIFO, 待写入SDRAM的数据先暂存于此
    //使用FIFO IP�?
    wr_fifo wr_fifo256x512_inst2 (
        .rst                (~rst_n | wfifo_rst_h1          ),  
        //.rst                (1'b0        ),
        .wr_clk             (wr_clk_2                     ),  //写端口时�?
        .rd_clk             (clk_100                    ),  //读端口时钟是AXI主机时钟, AXI写主机读取数�?
        .din                (wfifo_din_2                  ),  
        .wr_en              (wfifo_wren_2                 ),  
        .rd_en              (axi_writing_2                ),  //axi_master_wr正在写时,从写FIFO中不断读出数�?
        .dout               (axi_wr_data_2                ),  //读出的数据作为AXI写主机的输入数据
        .full               (                           ),  
        .almost_full        (                           ),  
        .empty              (                           ),  
        .almost_empty       (                           ),  
        .rd_data_count      (cnt_wr_fifo_rdport_2         ),  //写FIFO读端�?(对接AXI写主�?)数据数量
        .wr_data_count      (                           ),  
        .wr_rst_busy        (                           ),  
        .rd_rst_busy        (                           )   
    );

endmodule
