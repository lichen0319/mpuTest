figure

%%
try
    s=serial('COM2');
catch
    error('canot serial');
end
set(s,'BaudRate', 115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');%设置串口属性等
s.BytesAvailableFcnMode = 'terminator';
s.BytesAvailableFcn = {@callback3d};%设置串口回调函数，串口有数据传输过来就会转到回调函数，p、q为参数

fopen(s);%打开串口
pause;%按任一按键结束
fclose(s);
delete(s);
clear s
close all;
clear all;
