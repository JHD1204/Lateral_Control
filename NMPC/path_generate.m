%生成直线圆弧路径
%返回值为路径（x,y,phi）

function path = path_generate(T)

line = 20;  %直线长度
R = 50;     %弯道半径

xw1 = -3:T:line;
yw1 = 0*xw1;
anglew1 = 0*xw1;
xw3 = -line:0.01:110;
xw3 = -xw3;
yw3 = 0*xw3-2*R;
s3 = size(xw3);
anglew3 = -pi*ones(1,s3(2));

t2 = 0:T:R*pi;
angle = t2/R;
s = size(angle);
xw2 = zeros(1,s(2));
yw2 = zeros(1,s(2));
anglew2 = zeros(1,s(2));
for i = 1:1:s(2)
    if angle(i) <= pi/2
       yw2(i) = -(R-R*cos(angle(i)));
       xw2(i) = R*sin(angle(i))+line;
       anglew2(i) = -angle(i);
    else
       yw2(i) = -(R+R*sin(angle(i)-pi/2));
       xw2(i) = R*sin(angle(i))+line;
       anglew2(i) = -angle(i);
    end
end

 xw = [xw1,xw2,xw3]';
 yw = [yw1,yw2,yw3]';
 anglew = [anglew1,anglew2,anglew3]';
 anglew = anglew/pi*180;
 path = [xw,yw,anglew]; %deg

end