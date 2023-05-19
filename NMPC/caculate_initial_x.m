%���������ֵ
%Stanley����
%���ص�����ֵ��Ԥ�鲽��

function [X0,Np] = caculate_initial_x(path,Np,v,T,x_0,y_0,phi_0,len)

k_gain = 0.3;

xw = path(:,1);
yw = path(:,2);
anglew = path(:,3);

x_0 = x_0+len*cos(phi_0);   % ����λ��תΪǰ��λ��
y_0 = y_0+len*sin(phi_0);

%% Ѱ�Ҿ��복�������
C1 = xw(:,1)>(x_0-5)&xw(:,1)<(x_0+5)& yw(:,1)>(y_0-5)&yw(:,1)<(y_0+5); 
D1 = find(C1==1);
S1 = length(D1);
xw_temp = zeros(S1,1);
yw_temp = zeros(S1,1);
anglew_temp = zeros(S1,1);
for i=1:1:S1
    xw_temp(i) = xw(D1(i));
    yw_temp(i) = yw(D1(i));
    anglew_temp(i) = anglew(D1(i));
end
anglew_temp = anglew_temp/180*pi; %rad
C2 = mod(phi_0-anglew_temp+2*pi,2*pi)<pi/3 | mod(phi_0-anglew_temp+2*pi,2*pi)>5*pi/3; %�߼�ֵΪ1������
D2 = find(C2==1); 
S2 = length(D2);  %����D����
xw_temp_2 = zeros(S2,1);
yw_temp_2 = zeros(S2,1);
anglew_temp_2=zeros(S2,1);
for i=1:1:S2
    xw_temp_2(i) = xw_temp(D2(i));
    yw_temp_2(i) = yw_temp(D2(i));
    anglew_temp_2(i) = anglew_temp(D2(i));
end
e = sqrt((xw_temp_2-x_0).^2+(yw_temp_2-y_0).^2); %����������
[~,m] = min(e);
m_1 = D2(m(1));
n = D1(m_1);

%������ǰ��Ϊ��׼�ο���
distance_accumulate = 0;
[path_size,~] = size(xw);  %����ֵ����������������������ʱΪ��ֵ��
j = path_size-n;           %ʣ�����
x_ref = [];
y_ref = [];
angle_ref = [];
if j >= 10000                 % use 100m forward
    E = xw(n:n+10000);        
    F = yw(n:n+10000);
    G = anglew(n:n+10000);
else
    E = xw(n:path_size);
    F = yw(n:path_size);
    G = anglew(n:path_size);
end
[pre_refway_size,~] = size(E);
i = 1;
k = 1;
while ((i<=pre_refway_size-1)&&(k<=Np))
   distance_accumulate = distance_accumulate+sqrt((E(i+1,1)-E(i,1)).^2+(F(i+1,1)-F(i,1)).^2);
   if distance_accumulate >= k*v*T
       k = k+1;
       x_ref = [x_ref;E(i+1,1)];
       y_ref = [y_ref;F(i+1,1)];
       angle_ref = [angle_ref;G(i+1,1)];
   end
   i = i+1;
end
[Np,~] = size(x_ref);
angle_ref = angle_ref/180*pi;

%% stanley�������ֵ��
% calculate the initial_value of X0 using stanley method
% the bicyle model is base on the front axle
x = zeros(1,Np);
y = zeros(1,Np);
phi = zeros(1,Np);
delta = zeros(1,Np);
temp_lateral_error = (x_0-E(1))*sin(G(1))-(y_0-F(1))*cos(G(1));    
temp_angle_error = mod(G(1)-phi_0+3*pi,2*pi)-pi;
temp_delta = temp_angle_error + atan2(k_gain*temp_lateral_error,v);  %rad
delta(1) = temp_delta;
x(1) = x_0+v*T*cos(phi_0+temp_delta);
y(1) = y_0+v*T*sin(phi_0+temp_delta);
phi(1) = phi_0+v*T*sin(temp_delta)/len;
for i = 1:1:Np-1
     temp_lateral_error = (x(i)-x_ref(i))*sin(angle_ref(i))-(y(i)-y_ref(i))*cos(angle_ref(i)); %�������
     temp_angle_error = mod(angle_ref(i)-phi(i)+3*pi,2*pi)-pi;                                 %��ڽ����
     temp_delta = temp_angle_error + atan2(k_gain*temp_lateral_error,v);                       %����ת�� rad
     x(i+1) = x(i)+v*T*cos(phi(i)+temp_delta);                                                 %���³���λ��
     y(i+1) = y(i)+v*T*sin(phi(i)+temp_delta);
     phi(i+1) = phi(i)+v*T*sin(temp_delta)/len;
     delta(i+1) = temp_delta;                                                                  %���³���ת��
end
x_r = zeros(1,Np); %transform front axle into rear axle
y_r = zeros(1,Np);
for i = 1:1:Np
    x_r(i) = x(i)-len*cos(phi(i));   % ǰ��λ��תΪ����λ��
    y_r(i) = y(i)-len*sin(phi(i));
end
X0 = [x_r y_r phi delta]; %������ֵ

end