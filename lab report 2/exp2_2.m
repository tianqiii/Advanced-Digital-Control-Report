A = [0.368 0; 0.362 1];    % 状态空间矩阵 A
B = [0.632; 0.368];        % 输入矩阵 B
C = [0 1];                 % 输出矩阵 C
Q0 = [0 0; 0 0];           % 最终代价矩阵 Q0
Q1 = [2 0; 0 2];           % 状态代价矩阵 Q1
Q2 = 1;                    % 控制代价矩阵 Q2

N = 10;                    % 时间步长 N
x1(1) = -1;                % 初始状态 x1(1)
x2(1) = 0;                 % 初始状态 x2(1)

% 初始化 K(k) 和 S(k) 矩阵
k1 = zeros(1, N); 
k2 = zeros(1, N);
s11 = zeros(1, N); 
s12 = zeros(1, N); 
s22 = zeros(1, N);

% (2) 设置最终条件 S(N) = Q0
Snext = Q0;
s11(N) = Snext(1,1); 
s12(N) = Snext(1,2); 
s22(N) = Snext(2,2);

% (3-6) 迭代求解 K(k) 和 S(k)
for k = N-1:-1:1
    % (3) 计算 K(k) 根据式(6-165)
    K = (Q2 + B' * Snext * B) \ B' * Snext * A;
    k1(k) = K(1); 
    k2(k) = K(2);
    
    % (4) 计算 S(k) 根据式(6-164)
    S = (A - B * K)' * Snext * (A - B * K) + Q1 + K' * Q2 * K;
    s11(k) = S(1,1); 
    s12(k) = S(1,2); 
    s22(k) = S(2,2);
    Snext = S;
end

% 状态反馈控制系统
x1 = zeros(1, N); 
x2 = zeros(1, N); 
u = zeros(1, N);
x1(1) = -1;  % 初始状态
x2(1) = 0;

% (7) 计算控制量 u(k) 和输出序列 y(k)
for i = 1:N-1
    xnext = (A - B * [k1(i) k2(i)]) * [x1(i); x2(i)];
    x1(i+1) = xnext(1);
    x2(i+1) = xnext(2);
    u(i) = -[k1(i) k2(i)] * [x1(i); x2(i)];
end

% (8) 绘制结果图像
k = 0:N-1;

figure;

% 图2(a) Riccati 方程的解 S(k)
subplot(2,2,1), plot(k, s11, 'o', k, s12, '*', k, s22, '+');
ylabel('S(k)');
title('Riccati 方程的解 S(k)');
legend('s11(k)', 's12(k)', 's22(k)');

% 图2(b) 状态反馈增益控制规律 K(k)
subplot(2,2,2), plot(k, k1, 'o', k, k2, '*');
ylabel('K(k)');
title('反馈控制规律 K(k)');
legend('k1(k)', 'k2(k)');

% 图2(c) 控制序列 u(k)
subplot(2,2,3), plot(k, u, 'o');
ylabel('u(k)');
xlabel('k');
title('控制序列 u(k)');

% 图2(d) 输出序列 y(k)
subplot(2,2,4), plot(k, x2, 'o');
ylabel('y(k) = x2(k)');
xlabel('k');
title('输出序列 y(k)');
