A=[1 1;1 0]; B=[1;0];                      %状态空间表达式
Q1=[1 0;0 1];Q0=[1 0;0 1];Q2=1;           %代价函数 
n=8;x1(1)=1;x2(1)=0;                      %x0=[1;0] 
k1(n)=0;k2(n)=0;
s11(n)=1; s12(n)=0; s21(n)=0; s22(n)=1; Snext=Q0; %最终段条件 
for i=n-1:-1:1
     K=(Q2+B'*Snext*B)\B'*Snext*A;     %对应于式(6-164) 
     k1(i)=K(1);
     k2(i)=K(2);
     S=(A-B*K)'*Snext*(A-B*K)+Q1+K'*Q2*K;    %对应于式(6-163) 
     s11(i)=S(1,1);s12(i)=S(1,2);s21(i)=S(2,1);s22(i)=S(2,2);Snext=S; 
end
for i=1:n-1 
   xnext=(A-B*[k1(i) k2(i)])*[x1(i);x2(i)]; 
   x1(i+1)=xnext(1); x2(i+1)=xnext(2);     %系统状态 
   y(i)=x2(i);                          %C=[0 1] 
end
for i=1:n 
   u(i)=-[k1(i) k2(i)]*[x1(i); x2(i)]; 
end
k=0:7;
%画图 
figure
subplot(221),plot(k,s11,'o',k,s12,'*',k,s21,'r',k,s22,'+'); 
ylabel('S(k)'); 
text(2,3.5,'s11(k)'); 
text(2,2.2,' s22(k) ');
text(1.5,0.7,'s12(k)=s21(k) '); 
subplot(222),plot(k,x2,'o');
ylabel('y(k)=x2(k)'); 
subplot(223),plot(k,k1,'o',k,k2,'*');
ylabel('K(k)'); 
text(1,0.9,'l1(k)');text(3,0.7,'l2(k)'); 
xlabel('k'); 
subplot(224),plot(k,u,'o');
ylabel('u(k)');xlabel('k'); 
%显示值
K=[k1;k2]; S=[s11;s12;s12;s22]; 
