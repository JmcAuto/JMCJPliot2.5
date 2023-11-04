%选择使用的CAN设备
%candevice=1为VN1640A(3,4通道可用)
%candevice=2为VN5620(5,6通道可用)
candevice=1;
slxname='ALL_JP360_DBW_V6_5_1103';
%slxname填写当前运行的slx文件名称
%本m文件运行两遍！两遍！两遍！%
%*****************************************************%
%*****************************************************%
if candevice==1
    de='VN1640A 1';
    a=2;
elseif candevice==2
    de='VN5620 1';
    a=4;
end
for m=1:2
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'ArbitrationBusSpeed','500000');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'Device',['Vector ',de,' (Channel ',num2str(m+a),')']);
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'DataBusSpeed','2000000');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'ArbitrationSJW','8');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'DataSJW','2');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'ArbitrationTSEG1','31');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'DataTSEG1','7');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'ArbitrationTSEG2','8');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'DataTSEG2','2');
    set_param([slxname,'/CAN FD Configuration',num2str(m)],'AckMode','Normal');
end
%CCAN的数据接收
for n=1:17
    set_param([slxname,'/MsgFromCAN/ADCU_data/CAN FD Receive',num2str(n)],'Device',['Vector ',de,' (Channel ',num2str(1+a),')']);
end
%IPM的车道线的数据接收
for n=1:4
    set_param([slxname,'/MsgFromCAN/IPM_Line_data/CAN FD Receive',num2str(n)],'Device',['Vector ',de,' (Channel ',num2str(2+a),')']);
end
%IPM的OBJ的数据接收
for n=1:21
    set_param([slxname,'/MsgFromCAN/IPM_Obj_data/CAN FD Receive',num2str(n)],'Device',['Vector ',de,' (Channel ',num2str(2+a),')']);
end
%MRR的OBJ的数据接收
for n=1:17
    set_param([slxname,'/MsgFromCAN/MRR_Obj_data/CAN FD Receive',num2str(n)],'Device',['Vector ',de,' (Channel ',num2str(2+a),')']);
end


for k=1:11
    set_param([slxname,'/MsgToCAN/CAN FD Transmit',num2str(k)],'Device',['Vector ',de,' (Channel ',num2str(1+a),')']);
end