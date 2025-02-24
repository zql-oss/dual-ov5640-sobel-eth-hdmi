`define HDMI_768P
module video_driver(
    input                       pixel_clk ,
    input                       sys_rst_n ,
                                
    //RGB接口                   
    output                      video_hs  ,     //行同步信号
    output                      video_vs  ,     //场同步信号
    output                      video_de  ,     //数据使能
    output  [15:0]              video_rgb ,    //RGB888颜色数据
                                
    input   [15:0]              pixel_data,   //像素点数据
    output  [10:0]              pixel_xpos, //像素点横坐标
    output  [10:0]              pixel_ypos, //像素点纵坐标
    output  [10:0]              h_disp    ,     //像素点横分辨率
    output  [10:0]              v_disp    ,     //像素点纵分辨率  
    output                      data_req
);
        localparam  H_TOTAL     =   H_SYNC + H_BACK + H_DISP + H_FRONT ;//行扫描周期;
        localparam  V_TOTAL     =   V_SYNC + V_BACK + V_DISP + V_FRONT;//场扫描周期; 
        localparam  H_TOTAL_W   =   clogb2(H_TOTAL - 1);
        localparam  V_TOTAL_W   =   clogb2(V_TOTAL - 1);           
`ifdef HDMI_720P//1280*720 分辨率时序参数;60HZ刷新率对应时钟频率74.25MHZ
        localparam  H_SYNC      =   11'd40      ;//行同步;
        localparam  H_BACK      =   11'd220     ;//行显示后沿;
        localparam  H_DISP      =   11'd1280    ;//行有效数据;
        localparam  H_FRONT     =   11'd110     ;//行显示前沿;

        localparam  V_SYNC      =   11'd5       ;//场同步;
        localparam  V_BACK      =   11'd20      ;//场显示后沿;
        localparam  V_DISP      =   11'd720     ;//场有效数据;
        localparam  V_FRONT     =   11'd5       ;//场显示前沿;
    `elsif HDMI_768P//1024*768,60HZ分辨率时序参数;对应时钟频率65MHz；
        localparam  H_SYNC      =  12'd136      ;//行同步;
        localparam  H_BACK      =  12'd160      ;//行显示后沿;
        localparam  H_DISP      =  12'd1024     ;//行有效数据;
        localparam  H_FRONT     =  12'd24       ;//行显示前沿;

        localparam  V_SYNC      =  12'd6        ;//场同步;
        localparam  V_BACK      =  12'd29       ;//场显示后沿;
        localparam  V_DISP      =  12'd768      ;//场有效数据;
        localparam  V_FRONT     =  12'd3        ;//场显示前沿;
    `elsif HDMI1080P//1920*1080分辨率时序参数，60HZ刷新率对应时钟频率148.5MHZ
        localparam  H_SYNC      =  12'd44       ;//行同步;
        localparam  H_BACK      =  12'd148      ;//行显示后沿;
        localparam  H_DISP      =  12'd1920     ;//行有效数据;
        localparam  H_FRONT     =  12'd88       ;//行显示前沿;

        localparam  V_SYNC      =  12'd5        ;//场同步;
        localparam  V_BACK      =  12'd36       ;//场显示后沿;
        localparam  V_DISP      =  12'd1080     ;//场有效数据;
        localparam  V_FRONT     =  12'd4        ;//场显示前沿;
    `else//1024*600分辨率时序参数;
        localparam  H_SYNC      =  12'd20       ;//行同步;
        localparam  H_BACK      =  12'd140      ;//行显示后沿;
        localparam  H_DISP      =  12'd1024     ;//行有效数据;
        localparam  H_FRONT     =  12'd160      ;//行显示前沿;

        localparam  V_SYNC      =  12'd3        ;//场同步;
        localparam  V_BACK      =  12'd20       ;//场显示后沿;
        localparam  V_DISP      =  12'd600      ;//场有效数据;
        localparam  V_FRONT     =  12'd12       ;//场显示前沿;
    `endif

    //自动计算位宽函数
    function integer clogb2(input integer depth);begin
        if(depth == 0)
            clogb2 = 1;
        else if(depth != 0)
            for(clogb2=0 ; depth>0 ; clogb2=clogb2+1)
                depth=depth >> 1;
        end
    endfunction
//reg define
reg  [H_TOTAL_W - 1 : 0]    cnt_h       ;
reg  [V_TOTAL_W - 1 : 0]    cnt_v       ;

//wire define
wire        video_en;
wire        data_req;
// 根据 H_DISP 和 V_DISP 动态设置位宽
wire [10 : 0]  h_disp;
wire [10 : 0]  v_disp;

//*****************************************************
//**                    main code
//*****************************************************

assign video_de  = video_en;

assign video_hs  = ( cnt_h < H_SYNC ) ? 1'b0 : 1'b1;  //行同步信号赋值
assign video_vs  = ( cnt_v < V_SYNC ) ? 1'b0 : 1'b1;  //场同步信号赋值

//使能RGB数据输出
assign video_en  = (((cnt_h >= H_SYNC+H_BACK) && (cnt_h < H_SYNC+H_BACK+H_DISP))
                 &&((cnt_v >= V_SYNC+V_BACK) && (cnt_v < V_SYNC+V_BACK+V_DISP)))
                 ?  1'b1 : 1'b0;

//RGB888数据输出
assign video_rgb = video_en ? pixel_data : 24'd0;

//请求像素点颜色数据输入
assign data_req = (((cnt_h >= H_SYNC+H_BACK-1'b1) && 
                    (cnt_h < H_SYNC+H_BACK+H_DISP-1'b1))
                  && ((cnt_v >= V_SYNC+V_BACK) && (cnt_v < V_SYNC+V_BACK+V_DISP)))
                  ?  1'b1 : 1'b0;

//像素点坐标
assign pixel_xpos = data_req ? (cnt_h - (H_SYNC + H_BACK - 1'b1)) : 11'b0;
assign pixel_ypos = data_req ? (cnt_v - (V_SYNC + V_BACK - 1'b1)) : 11'b0;

//行场分辨率
assign h_disp = H_DISP;
assign v_disp = V_DISP; 

//行计数器对像素时钟计数
always @(posedge pixel_clk ) begin
    if (!sys_rst_n)
        cnt_h <= {{H_TOTAL_W}{1'b0}};
    else begin
        if(cnt_h < H_TOTAL - 1'b1)
            cnt_h <= cnt_h + 1'b1;
        else 
            cnt_h <= {{H_TOTAL_W}{1'b0}};
    end
end

//场计数器对行计数
always @(posedge pixel_clk ) begin
    if (!sys_rst_n)
       cnt_v <= {{V_TOTAL_W}{1'b0}};
    else if(cnt_h == H_TOTAL - 1'b1) begin
        if(cnt_v < V_TOTAL - 1'b1)
            cnt_v <= cnt_v + 1'b1;
        else 
            cnt_v <= {{V_TOTAL_W}{1'b0}};
    end
end

endmodule