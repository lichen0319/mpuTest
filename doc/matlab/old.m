figure

%%
try
    s=serial('COM2');
catch
    error('canot serial');
end
set(s,'BaudRate', 115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');%���ô������Ե�
s.BytesAvailableFcnMode = 'terminator';
s.BytesAvailableFcn = {@callback3d};%���ô��ڻص����������������ݴ�������ͻ�ת���ص�������p��qΪ����

fopen(s);%�򿪴���
pause;%����һ��������
fclose(s);
delete(s);
clear s
close all;
clear all;
