`define HDMI_768P
module video_driver(
    input                       pixel_clk ,
    input                       sys_rst_n ,
                                
    //RGB�ӿ�                   
    output                      video_hs  ,     //��ͬ���ź�
    output                      video_vs  ,     //��ͬ���ź�
    output                      video_de  ,     //����ʹ��
    output  [15:0]              video_rgb ,    //RGB888��ɫ����
                                
    input   [15:0]              pixel_data,   //���ص�����
    output  [10:0]              pixel_xpos, //���ص������
    output  [10:0]              pixel_ypos, //���ص�������
    output  [10:0]              h_disp    ,     //���ص��ֱ���
    output  [10:0]              v_disp    ,     //���ص��ݷֱ���  
    output                      data_req
);
        localparam  H_TOTAL     =   H_SYNC + H_BACK + H_DISP + H_FRONT ;//��ɨ������;
        localparam  V_TOTAL     =   V_SYNC + V_BACK + V_DISP + V_FRONT;//��ɨ������; 
        localparam  H_TOTAL_W   =   clogb2(H_TOTAL - 1);
        localparam  V_TOTAL_W   =   clogb2(V_TOTAL - 1);           
`ifdef HDMI_720P//1280*720 �ֱ���ʱ�����;60HZˢ���ʶ�Ӧʱ��Ƶ��74.25MHZ
        localparam  H_SYNC      =   11'd40      ;//��ͬ��;
        localparam  H_BACK      =   11'd220     ;//����ʾ����;
        localparam  H_DISP      =   11'd1280    ;//����Ч����;
        localparam  H_FRONT     =   11'd110     ;//����ʾǰ��;

        localparam  V_SYNC      =   11'd5       ;//��ͬ��;
        localparam  V_BACK      =   11'd20      ;//����ʾ����;
        localparam  V_DISP      =   11'd720     ;//����Ч����;
        localparam  V_FRONT     =   11'd5       ;//����ʾǰ��;
    `elsif HDMI_768P//1024*768,60HZ�ֱ���ʱ�����;��Ӧʱ��Ƶ��65MHz��
        localparam  H_SYNC      =  12'd136      ;//��ͬ��;
        localparam  H_BACK      =  12'd160      ;//����ʾ����;
        localparam  H_DISP      =  12'd1024     ;//����Ч����;
        localparam  H_FRONT     =  12'd24       ;//����ʾǰ��;

        localparam  V_SYNC      =  12'd6        ;//��ͬ��;
        localparam  V_BACK      =  12'd29       ;//����ʾ����;
        localparam  V_DISP      =  12'd768      ;//����Ч����;
        localparam  V_FRONT     =  12'd3        ;//����ʾǰ��;
    `elsif HDMI1080P//1920*1080�ֱ���ʱ�������60HZˢ���ʶ�Ӧʱ��Ƶ��148.5MHZ
        localparam  H_SYNC      =  12'd44       ;//��ͬ��;
        localparam  H_BACK      =  12'd148      ;//����ʾ����;
        localparam  H_DISP      =  12'd1920     ;//����Ч����;
        localparam  H_FRONT     =  12'd88       ;//����ʾǰ��;

        localparam  V_SYNC      =  12'd5        ;//��ͬ��;
        localparam  V_BACK      =  12'd36       ;//����ʾ����;
        localparam  V_DISP      =  12'd1080     ;//����Ч����;
        localparam  V_FRONT     =  12'd4        ;//����ʾǰ��;
    `else//1024*600�ֱ���ʱ�����;
        localparam  H_SYNC      =  12'd20       ;//��ͬ��;
        localparam  H_BACK      =  12'd140      ;//����ʾ����;
        localparam  H_DISP      =  12'd1024     ;//����Ч����;
        localparam  H_FRONT     =  12'd160      ;//����ʾǰ��;

        localparam  V_SYNC      =  12'd3        ;//��ͬ��;
        localparam  V_BACK      =  12'd20       ;//����ʾ����;
        localparam  V_DISP      =  12'd600      ;//����Ч����;
        localparam  V_FRONT     =  12'd12       ;//����ʾǰ��;
    `endif

    //�Զ�����λ����
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
// ���� H_DISP �� V_DISP ��̬����λ��
wire [10 : 0]  h_disp;
wire [10 : 0]  v_disp;

//*****************************************************
//**                    main code
//*****************************************************

assign video_de  = video_en;

assign video_hs  = ( cnt_h < H_SYNC ) ? 1'b0 : 1'b1;  //��ͬ���źŸ�ֵ
assign video_vs  = ( cnt_v < V_SYNC ) ? 1'b0 : 1'b1;  //��ͬ���źŸ�ֵ

//ʹ��RGB�������
assign video_en  = (((cnt_h >= H_SYNC+H_BACK) && (cnt_h < H_SYNC+H_BACK+H_DISP))
                 &&((cnt_v >= V_SYNC+V_BACK) && (cnt_v < V_SYNC+V_BACK+V_DISP)))
                 ?  1'b1 : 1'b0;

//RGB888�������
assign video_rgb = video_en ? pixel_data : 24'd0;

//�������ص���ɫ��������
assign data_req = (((cnt_h >= H_SYNC+H_BACK-1'b1) && 
                    (cnt_h < H_SYNC+H_BACK+H_DISP-1'b1))
                  && ((cnt_v >= V_SYNC+V_BACK) && (cnt_v < V_SYNC+V_BACK+V_DISP)))
                  ?  1'b1 : 1'b0;

//���ص�����
assign pixel_xpos = data_req ? (cnt_h - (H_SYNC + H_BACK - 1'b1)) : 11'b0;
assign pixel_ypos = data_req ? (cnt_v - (V_SYNC + V_BACK - 1'b1)) : 11'b0;

//�г��ֱ���
assign h_disp = H_DISP;
assign v_disp = V_DISP; 

//�м�����������ʱ�Ӽ���
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

//�����������м���
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