clear all; close all;

h=0.1; L=100/h;        %数值积分步长和仿真步数（减小h，可以提高积分精度）
num=[2 1]; den=[1 2 1]; n=length(den)-1;    %对象参数（严格正实）
k=1; [Ap,Bp,Cp,Dp]=tf2ss(k*num,den);      %对象参数（传递函数型转换为状态空间型）
km=1; [Am,Bm,Cm,Dm]=tf2ss(km*num,den); %参考模型参数

gamma=0.1;                           %自适应增益

yr0=0; u0=0; e0=0;                     %初值
xp0=zeros(n,1); xm0=zeros(n,1);          %状态向量初值
kc0=0;                               %可调增益初值
r=2; yr=r*[ones(1,L/4) -ones(1,L/4) ones(1,L/4) -ones(1,L/4)]; %输入信号

for ke=1:L
    time(ke)=ke*h;
    xp(:,ke)=xp0+h*(Ap*xp0+Bp*u0);
    yp(ke)=Cp*xp(:,ke);            %计算yp
    
    xm(:,ke)=xm0+h*(Am*xm0+Bm*yr0);
    ym(ke)=Cm*xm(:,ke);                %计算ym
    
    e(ke)=ym(ke)-yp(ke);                %e=ym-yp
    kc=kc0+h*gamma*e0*yr0;            %Lyapunov-MRAC自适应律(7-81)
    u(ke)=kc*yr(ke); %控制量
    
    %更新数据
    yr0=yr(ke); u0=u(ke); e0=e(ke);
    xp0=xp(:,ke); xm0=xm(:,ke);
    kc0=kc;
end
subplot(2,1,1);
plot(time,ym,'r',time,yp,':');
xlabel('t'); ylabel('y_m(t)，y_p(t)');
legend('y_m(t)','y_p(t)');
subplot(2,1,2);
plot(time,u);
xlabel('t'); ylabel('u(t)');
