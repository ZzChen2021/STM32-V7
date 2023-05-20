% ********************************************************************************************************************
% ********************************************************************************************************************
close all
clear all

%ɾ�������Ѿ��򿪵Ĵ��ڣ���������Ҫ����ֹ֮ǰ����û�йرմ���
delete(instrfindall);  

%�򿪴���COM7��������115200��8λ����λ��1λֹͣλ������żУ�飬��������
s = serial('COM7', 'BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'FlowControl', 'none');
%set(s,'Timeout',15);
s.ReadAsyncMode = 'continuous';
fopen(s);

fig = figure(1);

% ********************************************************************************************************************
AxisMax =  65536;    %���������ֵ
AxisMin = -65536;    %��������Сֵ
window_width = 800;  %���ڿ��

g_Count =0;          %���յ������ݼ���
SOF = 0;             %ͬ��֡��־
AxisValue = 1;       %����ֵ
RecDataDisp = zeros(1,10000000); %����100000�����ݵ�Ԫ�����ڴ洢���յ������ݡ�
RecData = zeros(1,10000);        %����100�����ݵ�Ԫ���������ݴ���
Axis = zeros(1,10000000);        %����100000�����ݵ�Ԫ������X�ᡣ

window = window_width * (-0.9); %����X����ʼ����
axis([window, window + window_width, AxisMin, AxisMax]); %���ô������귶Χ

%��ͼ1��ʾ�����ϴ�������
subplot(2,1,1); 
grid on;
title('�������ݽ���');
xlabel('ʱ��');
ylabel('����');

%��ͼ2��ʾ���εķ�Ƶ��Ӧ
subplot(2,1,2);
grid on;
title( 'FFT');
xlabel( 'Ƶ��');
ylabel( '����');

Fs = 100;        % ������
N = 50;         % ��������
n = 0:N-1;      % ��������
f = n * Fs / N; %��ʵ��Ƶ��

% ********************************************************************************************************************

while ishandle(fig)
    
    %����ͬ���źű�־�� = 1��ʾ���յ���λ�����͵�ͬ��֡
    SOF = 0;  
    
    %����ͬ��֡
    fwrite(s, 13);
    
    %��ȡ�Ƿ�������
    bytes = get(s, 'BytesAvailable');
    if bytes == 0
         bytes = 1;
    end
    
    %��ȡ��λ�����ص���������
    RecData = fread(s, bytes, 'uint8');
   
    %������λ�����ص��������Ƿ����ַ�$
    StartData = find(RecData == 13);
    
    %���������$����ȡ10���ֽڵ����ݣ�Ҳ����5��uint16������
    if(StartData >= 1)
        RecData = fread(s, 5, 'uint16');
        SOF =1;
        StartData = 0;
    end
    
%���½��յ������ݲ���
if(SOF == 1)
	%��������
	RecDataDisp(AxisValue) =  RecData(1);
	RecDataDisp(AxisValue + 1) =  RecData(2);
	RecDataDisp(AxisValue + 2) =  RecData(3);
	RecDataDisp(AxisValue + 3) =  RecData(4);
	RecDataDisp(AxisValue + 4) =  RecData(5);
	
	%����X��
	Axis(AxisValue) = AxisValue;
	Axis(AxisValue + 1) = AxisValue + 1;
	Axis(AxisValue + 2) = AxisValue + 2;
	Axis(AxisValue + 3) = AxisValue + 3;
	Axis(AxisValue + 4) = AxisValue + 4;

	%���±���
	AxisValue = AxisValue + 5;
	g_Count = g_Count + 5;
	
	%���Ʋ���
	subplot(2,1,1);
	plot(Axis(1:AxisValue-1),  RecDataDisp(1:AxisValue-1), 'r');
	window = window + 5;
	axis([window, window + window_width, AxisMin, AxisMax]);
	grid on;
	title('�������ݽ���');
	xlabel('ʱ��');
	ylabel('����');
	drawnow
end
    
if(g_Count== 50)
   subplot(2,1,2); 
   %��ԭʼ�ź��� FFT �任
   y = fft(RecDataDisp(AxisValue-50:AxisValue-1), 50); 
   
   %�� FFT ת�������ģֵ
   Mag = abs(y)*2/N;  
   
   %���Ʒ�Ƶ��Ӧ����
   plot(f, Mag, 'r'); 
   grid on;
   title( 'FFT');
   xlabel( 'Ƶ��');
   ylabel( '����');
   g_Count = 0;
   drawnow
end

end

fclose(s);
delete(s);

% ********************************************************************************************************************
