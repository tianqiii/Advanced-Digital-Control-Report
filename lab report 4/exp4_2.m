a=[1 -2 1.1]; b=[1 2]; c=1; d=4; %对象参数
na=length(a)-1; b=[zeros(1,d-1) b]; nb=length(b)-1; %na、nb 为多项式 A、B 阶次（因 d!=1，对 b 添 0）

naa=na+1; %aa 的阶次

N1=d; N=8; Nu=2; %最小输出长度、预测长度、控制长度
gamma=1*eye(Nu); alpha=0.7; %控制加权矩阵、输出柔化系数

L=400; %控制步数
uk=zeros(d+nb,1); %输入初值：uk(i)表示 u(k-i)
duk=zeros(d+nb,1); %控制增量初值
yk=zeros(na,1); %输出初值
dyk=zeros(na,1); %输出增量初值
w=10*[ones(L/4,1);-ones(L/4,1);ones(L/4,1);-ones(L/4+d,1)]; %设定值
xi=sqrt(0.01)*randn(L,1); %白噪声序列

%RLS 初值
thetae_1=0.001*ones(na+nb-d+2,1); %不辨识 b 中添加的 0
P=10^6*eye(na+nb-d+2);
lambda=1; %遗忘因子[0.9 1]

for k=1:L
 time(k)=k;
 dy(k)=-a(2:na+1)*dyk(1:na)+b*duk(1:nb+1)+xi(k); %式(7-45)
 y(k)=yk(1)+dy(k); %采集输出数据
 Yk=[y(k); yk(1:na)]; %构建向量 Y(k)
 dUk=duk(1:nb); %构建向量 ΔU(k-j)
 
 %参考轨迹
 yr(k)=y(k); %式(7-123)
 for i=1:N
 yr(k+i)=alpha*yr(k+i-1)+(1-alpha)*w(k+d); %式(7-123)
 end
 Yr=[yr(k+N1:k+N)]'; %构建向量 Yr(k)
 
 %递推最小二乘法
 phi=[-dyk(1:na);duk(d:nb+1)];
 K=P*phi/(lambda+phi'*P*phi);
 thetae(:,k)=thetae_1+K*(dy(k)-phi'*thetae_1);
 P=(eye(na+nb-d+2)-K*phi')*P/lambda;%式(7-138)
 
 %提取辨识参数
 ae=[1 thetae(1:na,k)']; be=[zeros(1,d-1) thetae(na+1:na+nb-d+2,k)'];
 aae=conv(ae,[1 -1]);
 
 %求解多步 Diophantine 方程并构建 F1、F2、G
 [E,F,G]=multidiophantine(aae,be,c,N);
 G=G(N1:N,:);
 F1=zeros(N-N1+1,Nu); F2=zeros(N-N1+1,nb);
 for i=1:N-N1+1
 for j=1:min(i,Nu); F1(i,j)=F(i+N1-1,i+N1-1-j+1); end
 for j=1:nb; F2(i,j)=F(i+N1-1,i+N1-1+j); end
 end
 
 %求控制量
 dU=inv(F1'*F1+gamma)*F1'*(Yr-F2*dUk-G*Yk); %ΔU 式(7-135)
 du(k)=dU(1); u(k)=uk(1)+du(k);
 
 %更新数据
 thetae_1=thetae(:,k);
 
 for i=1+nb:-1:2
 uk(i)=uk(i-1);
 duk(i)=duk(i-1);
 end
 uk(1)=u(k);
 duk(1)=du(k);
 
 for i=na:-1:2
 yk(i)=yk(i-1);
 dyk(i)=dyk(i-1);
 end
 yk(1)=y(k);
 dyk(1)=dy(k);
end

figure(1);
subplot(2,1,1);
plot(time,w(1:L),'r:',time,y);
xlabel('k'); ylabel('w(k)、y(k)');
legend('w(k)','y(k)'); axis([0 L -20 20]);
subplot(2,1,2);
plot(time,u);
xlabel('k'); ylabel('u(k)'); axis([0 L -4 4]);
figure(2);
plot(1:L,thetae);
xlabel('k'); ylabel('辨识参数 a、b');
legend('a_1','a_2','b_0','b_1'); axis([0 L -3 3]);