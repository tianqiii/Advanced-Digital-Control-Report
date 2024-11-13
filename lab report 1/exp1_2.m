T0=0.1;                                        %采样周期
A=[0.81 -0.23 -0.045                              %状态矩阵
   0.09 0.98  -0.0023
   0.005 0.1   1];
B=[0.09 0.0047 0.00016]';                         %输入矩阵
C=[1 3.5 3];                                    %输出矩阵
D=0;   
Gss=ss(A,B,C,D,T0)                              %创建系统的ss模型
p_s=[0.75,0.88,0.9]';                              %期望的系统极点
K=place(A,B,p_s)                                %极点配置的控制器增益
p_o=[0.5 0.6 0.75]';                               %期望的观测器极点
L=place(A',C',p_o)'                               %观测器增益
%build observer-controller
F_oc=A-B*K-L*C                                %观测器-控制器系统矩阵
Goc=ss(F_oc,-L,-K,0,T0)                          %创建观测器-控制器为LTI
Goc_poles=pole(Goc)                             %控制器极点全为实数
Goc_zeros=tzero(Goc)                            %控制器零点
figure
title('Example 2.2:Bode Diagram')                  %标题
bode(Goc),grid                                  %画控制器的bode图
Gpc=Gss*Goc;                                  %控制器和被控对象
Gcl=feedback(Gpc,1,-1)                     %具有非单位的DC增益的闭环系统
cl_loop_poles=pole(Gcl)                           %闭环系统极点

 
lfg=dcgain(Gcl)
N=1/lfg                                         %归一化常数


T_ref=N*Gcl                                     %归一化闭环系统
t=[0:T0:4];                                       %时间向量
r=0*t;                                           %零参考输入
z0=[1 -0.75 0.5 0 0 0]                              %初始状态向量
[y,t,z]=lsim(T_ref,r,t,z0);                           %初始状态仿真


figure
plot(t,z(:,1:3),'o',t,z(:,4),'+',t,z(:,5),'x',t,z(:,6),'^'),grid     %画系统和观测器状态
title('Example 2.2: Observer-Controller Design')        %标题
xlabel('Time(s)')                                 %定义x轴
ylabel('Amplitude')                               %定义y轴
text(0.4,0.85,'x_1')
text(0.33,-0.5,'x_2')
text(0.07,0.5,'x_3')
legend('x_1','x_2','x_3','x_1hat','x_2hat','x_3hat')      %不同曲线标识符

figure
[ys,t,z]=step(T_ref,4);                            %计算前4s的阶跃响应
stem(t,ys,':*');grid
title('Example 2.2: Step Response')                 %标题
xlabel('Time(s)')                                %定义x轴
ylabel('Amplitude')                              %定义y轴

figure
[y_no_observer,t]=step(Gss,4);    % 没有观测器时的阶跃响应
stem(t,y_no_observer,':*');grid
title('Step Response without Observer')  % 标题
xlabel('Time(s)')                         % x轴
ylabel('Amplitude')                       % y轴
