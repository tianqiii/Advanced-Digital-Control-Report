T0=0.1;                                       %采样周期
A=[0.81 -0.23 -0.045                             %状态矩阵
   0.09 0.98 -0.0023
   0.005 0.1   1];
B=[0.09 0.0047 0.00016]';                        %输入矩阵
C=[1 3.5 3];                                   %输出矩阵
D=0;                                         %传递矩阵
p=[0.5 0.6 0.75]';                              %期望的观测极点
L=place(A',C',p)'                                %极点配置的评价增益(注：place指令比其它，如acker等，更适合极点配置分析和设计)
F_ob=A-L*C                                  %观测系统矩阵
eig(F_ob)                                     %计算观测系统矩阵的特征值

x0=[1;-0.75;0.5]                               %初始条件
dtime=[0:T0:6];                                %时间向量
u=zeros(1,length(dtime));                        %输入零向量
G=ss(A,B,C,D,T0)                             %创建ss型(离散状态方程State Space)系统为LTI对象(线性时不变系统,Linear Time Invariant)
[y,dtime,x]=lsim(G,u,dtime,x0);                   %系统仿真获得y(k) (注：lsim模拟状态观测器)
 
disp('Simulate Observer')                        %显示
B_ob=ss(F_ob,L,C,D,T0)                       %创建系统为LTI对象
[y_hat,dtime,x_hat]=lsim(B_ob,y,dtime);           %仿真观测器
%plot system and observer states
figure
plot(dtime,x,'o',dtime,x_hat(:,1),'+',dtime,x_hat(:,2),'x',...
    dtime,x_hat(:,3),'^'),grid                     %画系统和观测器状态
title('Observer design for Example 2.1')
xlabel('Time(s)')                               %定义x轴
ylabel('Amplitude')                             %定义y轴
text(0.6,0.75,'x_1')
text(0.3,-0.5,'x_2')
text(0.07,0.5,'x_3')

legend('x_1','x_2','x_3','x_1hat','x_2hat','x_3hat')   %不同曲线标识符

figure
plot(dtime,y,'o',dtime,y_hat,'*'),grid               %画y和y-hat图
title('Observer design for Example 2.1')            %标题
xlabel('Time(s)')                              %定义x轴
ylabel('Amplitude')                            %定义y轴
legend('y','y_hat')                             %不同曲线标识符
