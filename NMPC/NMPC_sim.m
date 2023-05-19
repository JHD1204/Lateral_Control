% NMPC Latral Controller Simulation

clear
global path;
Np = 60;          %预测时域
T = 0.1;          %预测步长
sample_t = 0.05;  %控制器运行周期（采样时间）
q = 10;           %权重矩阵系数
t = 0.01;         %离散路径间隔长度
path = path_generate(t);  %生成路径
N = length(path);

sim('MPCtest');                   %调用Simulink仿真

% 仿真结果绘图
% 期望路径与实际路径,直线与圆弧
figure;
set(gcf,'unit','centimeters','position',[3,14,7,6]);%设置figure的位置，左下角离屏幕左边3厘米，离下边14厘米，宽高7*6
set(0,'defaultfigurecolor','w');
plot(yout(:,1),yout(:,2));
hold on ;
set(0,'defaultfigurecolor','w');
plot(path(1:N-1000,1),path(1:N-1000,2));
xlabel('位置X/m');
ylabel('位置Y/m');
legend('参考路径','实际路径');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% 控制器输出转角
figure;
set(gcf,'unit','centimeters','position',[11,14,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,4)*180/pi);
xlabel('时间/t');
ylabel('转角/°');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% 横向误差
figure;
set(gcf,'unit','centimeters','position',[19,14,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,5));
xlabel('时间/t');
ylabel('误差/m');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% 速度
figure;
set(gcf,'unit','centimeters','position',[27,14,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,3));
xlabel('时间/t');
ylabel('速度/(m/s)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% 加速度
figure;
set(gcf,'unit','centimeters','position',[27,4,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,7)*180/pi);
xlabel('时间/t');
ylabel('横摆角速度/(deg/s)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
% 求解时间
figure;
set(gcf,'unit','centimeters','position',[19,4,7,6]);
set(0,'defaultfigurecolor','w');
% plot(tout,1000*yout(:,10));
plot(tout,1000*yout(:,11));
xlabel('时间/t');
ylabel('求解时间/(ms)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
hold on
legend('cpu time');
% 迭代次数
figure;
set(gcf,'unit','centimeters','position',[11,4,7,6]);
set(0,'defaultfigurecolor','w');
plot(tout,yout(:,12));
xlabel('时间/t');
ylabel('迭代次数/(次)');
grid on;
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');

