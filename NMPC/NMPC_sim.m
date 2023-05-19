% NMPC Latral Controller Simulation

clear
global path;
Np = 60;          %Ԥ��ʱ��
T = 0.1;          %Ԥ�ⲽ��
sample_t = 0.05;  %�������������ڣ�����ʱ�䣩
q = 10;           %Ȩ�ؾ���ϵ��
t = 0.01;         %��ɢ·���������
path = path_generate(t);  %����·��
N = length(path);

sim('MPCtest');                   %����Simulink����

% ��������ͼ
% ����·����ʵ��·��,ֱ����Բ��
figure;
set(gcf,'unit','centimeters','position',[3,14,7,6]);%����figure��λ�ã����½�����Ļ���3���ף����±�14���ף����7*6
set(0,'defaultfigurecolor','w');
plot(yout(:,1),yout(:,2));
hold on ;
set(0,'defaultfigurecolor','w');
plot(path(1:N-1000,1),path(1:N-1000,2));
xlabel('λ��X/m');
ylabel('λ��Y/m');
legend('�ο�·��','ʵ��·��');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% ���������ת��
figure;
set(gcf,'unit','centimeters','position',[11,14,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,4)*180/pi);
xlabel('ʱ��/t');
ylabel('ת��/��');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% �������
figure;
set(gcf,'unit','centimeters','position',[19,14,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,5));
xlabel('ʱ��/t');
ylabel('���/m');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% �ٶ�
figure;
set(gcf,'unit','centimeters','position',[27,14,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,3));
xlabel('ʱ��/t');
ylabel('�ٶ�/(m/s)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% ���ٶ�
figure;
set(gcf,'unit','centimeters','position',[27,4,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,7)*180/pi);
xlabel('ʱ��/t');
ylabel('��ڽ��ٶ�/(deg/s)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% ���ʱ��
figure;
set(gcf,'unit','centimeters','position',[19,4,7,6]);
set(0,'defaultfigurecolor','w');
% plot(tout,1000*yout(:,10));
plot(tout,1000*yout(:,11));
xlabel('ʱ��/t');
ylabel('���ʱ��/(ms)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
hold on
legend('cpu time');
% ��������
figure;
set(gcf,'unit','centimeters','position',[11,4,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,12));
xlabel('ʱ��/t');
ylabel('��������/(��)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');

