%����ο���
%���زο��㣬��������Ԥ�鲽��

function [ref_point,lateral_error,Np] = FindReferencePoint(path,Np,v,T,x_0,y_0,phi_0)
%read road points
xw = path(:,1);
yw = path(:,2);
anglew = path(:,3);   %deg

%find nearest point id
C1 = xw(:,1)>(x_0-5) & xw(:,1)<(x_0+5) & yw(:,1)>(y_0-5) & yw(:,1)<(y_0+5); %�߼�ֵΪ1������
D1 = find(C1==1); %�ҵ��߼�ֵΪ1�ĵ� 
S1 = length(D1);
xw_temp = zeros(S1,1);
yw_temp = zeros(S1,1);
anglew_temp = zeros(S1,1);
for i = 1:1:S1
    xw_temp(i) = xw(D1(i));
    yw_temp(i) = yw(D1(i));
    anglew_temp(i) = anglew(D1(i));
end
anglew_temp = anglew_temp/180*pi; %rad
C2 = rem(phi_0-anglew_temp+2*pi,2*pi)<pi/3 | rem(phi_0-anglew_temp+2*pi,2*pi)>5*pi/3; %�߼�ֵΪ1������
D2 = find(C2==1); %�ҵ��߼�ֵΪ1�ĵ�
S2 = length(D2);  %����D����
xw_temp_2 = zeros(S2,1);
yw_temp_2 = zeros(S2,1);
anglew_temp_2 = zeros(S2,1);
for i = 1:1:S2
    xw_temp_2(i) = xw_temp(D2(i));
    yw_temp_2(i) = yw_temp(D2(i));
    anglew_temp_2(i) = anglew_temp(D2(i));
end
e = sqrt((xw_temp_2-x_0).^2+(yw_temp_2-y_0).^2); %����������
[lateral_error,m] = min(e);
m_1 = D2(m(1));
n = D1(m_1); % ���������

phi_0 = phi_0*180/pi;
if phi_0 < 0        %(0,180) to (0,360)
phi_0 = phi_0+360;
end
%�ж��������
cx = xw(n);
cy = yw(n);
theta1 = atan2(cy-y_0,cx-x_0)*180/pi;  %����λ������������߽Ƕ�
theta1 = rem(theta1+360,360);
theta2 = theta1-phi_0;
theta2 = rem(theta2+360,360);
if theta2 < 180
    lateral_error = -lateral_error;    %����ƫ���Ϊ�����Ҳ�Ϊ��
end

%���ɴӲο���
distance_accumulate = 0;
[path_size,~] = size(xw);
j = path_size-n; %ʣ�����
x_ref = [];
y_ref = [];
angle_ref = [];
if j >= 10000          % use 100m forward
    E = xw(n:n+10000); % 10001*1 matrix.
    F = yw(n:n+10000);
    G = anglew(n:n+10000);
else
    E = xw(n:path_size);
    F = yw(n:path_size);
    G = anglew(n:path_size);
end

[pre_refway_size,~] = size(E);  %�ֲ�·����С
i = 1;
k = 1;
while ((i <= pre_refway_size-1) && (k <= Np))
   distance_accumulate = distance_accumulate+sqrt((E(i+1,1)-E(i,1)).^2+(F(i+1,1)-F(i,1)).^2);
   if distance_accumulate >= k*v*T
       k = k+1;
       x_ref = [x_ref;E(i+1,1)];
       y_ref = [y_ref;F(i+1,1)];
       angle_ref = [angle_ref;G(i+1,1)];
   end
   i = i+1;
end
[Np,~] = size(x_ref);                %����Ԥ�����
angle_ref = angle_ref/180*pi;
ref_point = [x_ref, y_ref,angle_ref]; %rad

end