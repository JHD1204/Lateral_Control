% 改进了最近点求法，考虑了路径的航向

function [sys,x0,str,ts,simStateCompliance] = MPC_ipopt(t,x,u,flag) %主函数

switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes; %初始化函数
  case 1
    sys=mdlDerivatives(t,x,u);  %更新连续状态
  case 2
    sys=mdlUpdate(t,x,u);  %更新离散状态
  case 3
    sys=mdlOutputs(t,x,u); %计算模块输出
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sample_t = 0.05;
sizes = simsizes;
sizes.NumContStates  = 0; %连续状态量个数
sizes.NumDiscStates  = 0; %离散状态量个数
sizes.NumOutputs     = 7; %输出量个数
sizes.NumInputs      = 5; %输入量个数
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; % at least one sample time is needed
sys = simsizes(sizes);
x0  = [];                 %状态量初始化
str = [];                 %matlab保留变量，默认为空，用不到
ts  = [sample_t 0];       %采样时间及其偏移
simStateCompliance = 'UnknownSimState';
end

function sys=mdlDerivatives(t,x,u) %连续状态更新
sys = [];
end

function sys=mdlUpdate(t,x,u)      %离散状态更新
sys = x;
end

function sys=mdlOutputs(t,x,u)     %输出函数

global path;   %读入参考路径
Np = 60;
T = 0.1;
q = 10;
len = 2.47;                         %轴距m
steering_speed = 15;              % deg/s   前轮转向速度
steering_speed_rad = steering_speed/180*pi; %rad/s
steering_max_deg = 32;            %最大转向角

%读入carsim返回值
x_0 = u(1);                      %位置
y_0 = u(2);
phi_0 = u(3);                     %横摆角
phi_0 = rem(phi_0,2*pi);  
phi_0 = rem(phi_0+3*pi,2*pi)-pi; %转换为（-pi，pi）
v = u(4);                        %车速，转换为m/s
delta_0 = u(5);                   %车轮转角,弧度制

%生成参考路点
[ref_point,lateral_error,Np] = FindReferencePoint(path,Np,v,T,x_0,y_0,phi_0);
x_ref = ref_point(:,1);         % x
y_ref = ref_point(:,2);         % y
angle_ref = ref_point(:,3);     % 航向角

%迭代初值计算
[X0,Np] = caculate_initial_x(path,Np,v,T,x_0,y_0,phi_0,len);
% X0 = zeros(1, 4*Np);

%求解变量上下界约束
lb = zeros(1,4*Np);   
ub = zeros(1,4*Np);
lb(1) = x_0+v*T*cos(phi_0);
ub(1) = x_0+v*T*cos(phi_0);
lb(Np+1) = y_0+v*T*sin(phi_0);
ub(Np+1) = y_0+v*T*sin(phi_0);
for i1 = 1:1:Np
    lb(i1) = x_0-v*i1*T;
    ub(i1) = x_0+v*i1*T;
    lb(Np+i1) = y_0-v*i1*T;
    ub(Np+i1) = y_0+v*i1*T;
    lb(2*Np+i1) = phi_0-v*i1*T/len;
    ub(2*Np+i1) = phi_0+v*i1*T/len;
    lb(3*Np+i1) = -steering_max_deg/180*pi;
    ub(3*Np+i1) = steering_max_deg/180*pi;
end


%目标函数求解
tic;
opts = optiset('solver','ipopt','display','iter','derivCheck','off');     %求解器设置
fun = @(x)objfunction(x,Np,q,x_ref,y_ref,angle_ref,delta_0);            %目标函数句柄
nlcon = @(x)kinematics(x, Np, T, v, len, x_0, y_0, phi_0, delta_0);       %车辆运动学约束函数句柄
cl_1 = zeros(1,3*Np);                                                
cu_1 = zeros(1,3*Np);                                                 
cl_2 = -steering_speed_rad*T*ones(1,Np);
cu_2 = steering_speed_rad*T*ones(1,Np);             
cl = [cl_1,cl_2];                                                       %非线性约束上下界
cu = [cu_1,cu_2]; 
grad = @(x)grad_generate(x,x_ref,y_ref,angle_ref,Np,q,delta_0);         %目标函数梯度
jac = @(x)jac_generate(x,Np,v,T,len);                                     %约束雅可比矩阵
jacstr = @()jacstr_generate(Np);                                        %约束雅可比矩阵结构
H = @(x,sigma,lambda)hessian_generate(x,sigma,lambda,q,angle_ref,Np,v,T,len);   %海森矩阵
Hstr = @()Hstr_generate(Np);                                                  %海森矩阵结构
Opt = opti('fun',fun,'grad',grad,'jac',jac,'jacstr',jacstr,'hess',H,'hstr',Hstr,'nl',nlcon,cl,cu,'bounds',lb,ub,'x0',X0,'options',opts); 
[y,fval,exitflag,info] = solve(Opt);                                    %求解（y:结果，fval:目标函数值，exitflag：求解状态，info：求解信息）
prog_run_t = toc;                                                       %计时

sys=[y(3*Np+1) lateral_error fval exitflag prog_run_t info.Time info.Iterations];  %输出结果


%目标函数
function y = objfunction(x,Np,q,x_ref,y_ref,angle_ref,delta_0)
%初始化
y=0;
for i = 1:1:Np
    y = y+((x_ref(i)-x(i))*sin(angle_ref(i))+(x(Np+i)-y_ref(i))*cos(angle_ref(i))).^2;
end
y = y+q*(x(3*Np+1)-delta_0).^2;
for i = 1:1:Np-1
    y = y+q*(x(3*Np+i+1)-x(3*Np+i)).^2;
end
end

%目标函数梯度
function grad = grad_generate(x,x_ref,y_ref,angle_ref,Np,q,delta_0)
grad = zeros(1,4*Np);
for i = 1:1:Np
    grad(i) = 2*(-(x(i)-x_ref(i))*sin(angle_ref(i))+(x(i+Np)-y_ref(i))*cos(angle_ref(i)))*(-sin(angle_ref(i)));
    grad(i+Np) = 2*(-(x(i)-x_ref(i))*sin(angle_ref(i))+(x(i+Np)-y_ref(i))*cos(angle_ref(i)))*(cos(angle_ref(i)));
end
grad(3*Np+1) = 2*q*(x(3*Np+1)-delta_0)-2*q*(x(3*Np+2)-x(3*Np+1));
for i = 2:1:Np-1
    grad(3*Np+i) = 2*q*(x(i+3*Np)-x(i-1+3*Np))-2*q*(x(i+3*Np+1)-x(i+3*Np));
end
grad(4*Np) = 2*q*(x(4*Np)-x(4*Np-1));
end

%非线性约束
function ceq = kinematics(x, Np, T, v, len, x_0, y_0, phi_0, delta_0)
ceq = zeros(1,4*Np);
for k=1:1:Np
    if k==1
        ceq(1,k) = x(k) - x_0 - v*T*cos(phi_0);     % x; 
        ceq(1,Np+k) = x(Np+k) - y_0 - v*T*sin(phi_0);   % y; 
        ceq(1,2*Np+k) = x(2*Np+k) - phi_0 - v*T*tan(delta_0)/len;    % phi;
        ceq(1,3*Np+k) = x(3*Np+k) - delta_0;         % delta;
    else
        ceq(1,k) = x(k) - x(k-1) - v*T*cos(x(2*Np+k-1));     % x; (2, Np)
        ceq(1,Np+k) = x(Np+k) - x(Np+k-1) - v*T*sin(x(2*Np+k-1));   % y; (Np+2, 2*Np)
        ceq(1,2*Np+k) = x(2*Np+k) - x(2*Np+k-1) - v*T*tan(x(3*Np+k-1))/len;    % phi; (2*Np+2, 3*Np)
        ceq(1,3*Np+k) = x(3*Np+k) - x(3*Np+k-1);         % delta; (3*Np+2, 4*Np)
    end
end
end

% %约束雅可比矩阵
function jac = jac_generate(x,Np,v,T,len)
jac = zeros(4*Np,4*Np);
for i=1:1:Np
    if i == 1
        jac(i,i) = 1;
        jac(i+Np,i+Np) = 1;
        jac(i+2*Np,i+2*Np) = 1;
        jac(i+3*Np,i+3*Np) = 1;
    else
        jac(i,i-1) = -1;
        jac(i,i) = 1;
        jac(i,i-1+2*Np) = v*T*sin(x(2*Np+i-1));
        
        jac(i+Np,i-1+Np) = -1;
        jac(i+Np,i+Np) = 1;
        jac(i+Np,i-1+2*Np) = -v*T*cos(x(2*Np+i-1));
        
        jac(i+2*Np,i-1+2*Np) = -1;
        jac(i+2*Np,i+2*Np) = 1;
        jac(i+2*Np,i-1+3*Np) = -v*T/len/(cos(x(3*Np+i-1)).^2);
        
        jac(i+3*Np,i-1+3*Np) = -1;
        jac(i+3*Np,i+3*Np) = 1;
    end
end
jac = sparse(jac);
end

%约束雅可比矩阵结构
function jacstr = jacstr_generate(Np)
jacstr = zeros(4*Np,4*Np);
for i=1:1:Np
    if i == 1
        jacstr(i,i) = 1;
        jacstr(i+Np,i+Np) = 1;
        jacstr(i+2*Np,i+2*Np) = 1;
        jacstr(i+3*Np,i+3*Np) = 1;
    else
        jacstr(i,i-1) = 1;
        jacstr(i,i) = 1;
        jacstr(i,i-1+2*Np) = 1;
        
        jacstr(i+Np,i-1+Np) = 1;
        jacstr(i+Np,i+Np) = 1;
        jacstr(i+Np,i-1+2*Np) = 1;
        
        jacstr(i+2*Np,i-1+2*Np) = 1;
        jacstr(i+2*Np,i+2*Np) = 1;
        jacstr(i+2*Np,i-1+3*Np) = 1;
        
        jacstr(i+3*Np,i-1+3*Np) = 1;
        jacstr(i+3*Np,i+3*Np) = 1;
    end
end
jacstr = sparse(jacstr);
end

%海森矩阵
function H = hessian_generate(x,sigma,lambda,q,angle_ref,Np,v,T,len)
H_f=zeros(4*Np,4*Np);
for i = 1:1:Np
    H_f(i,i) = sigma*2*(sin(angle_ref(i))).^2;
    H_f(i+Np,i) = -sigma*2*sin(angle_ref(i))*cos(angle_ref(i));
    H_f(i+Np,i+Np) = sigma*2*(cos(angle_ref(i))).^2;
end
for i = 1:1:Np-1
    H_f(i+2*Np,i+2*Np) = lambda(i+1)*v*T*cos(x(2*Np+i)) + lambda(i+1+Np)*v*T*sin(x(2*Np+i));
    H_f(i+3*Np,i+3*Np) = sigma*4*q - lambda(i+1+2*Np)*2*v*T*sin(x(3*Np+i))/len/((cos(x(3*Np+i))).^3);
    H_f(i+3*Np+1,i+3*Np) = -sigma*2*q;
end
H_f(4*Np,4*Np) = sigma*2*q;
H=sparse(H_f);
end

%海森矩阵结构
function Hstr = Hstr_generate(Np)
Hstr = zeros(4*Np,4*Np);
for i = 1:1:Np
    Hstr(i,i) = 1;
    Hstr(i+Np,i) = 1;
    Hstr(i+Np,i+Np) = 1;
end
for i=1:1:Np-1
    Hstr(i+2*Np,i+2*Np) = 1;
    Hstr(i+3*Np,i+3*Np) = 1;
    Hstr(i+3*Np+1,i+3*Np) = 1;
end
Hstr(4*Np,4*Np) = 1;
Hstr=sparse(Hstr);   
end

end
    
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
end

function sys=mdlTerminate(t,x,u)

sys = [];
end

end




