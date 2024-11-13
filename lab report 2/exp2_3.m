A = 0.3679;     % 状态矩阵 A
B = 0.6321;     % 输入矩阵 B
Q1 = 1;         % 状态权重矩阵 Q1
Q2 = 1;         % 控制权重矩阵 Q2
Q0 = 1;         % 最终条件 Q0

N = 10;         % 总时间步数 N
x(1) = 1;       % 初始状态 x(0)
S = zeros(1, N);  % 初始化S矩阵
S(N) = Q0;      % 终止条件 S(N) = Q0

% 初始化反馈增益矩阵 K(k)
K = zeros(1, N);
S = zeros(1, N);
S(N) = Q0;

% (2) 递归计算 Riccati 方程解 S(k) 和反馈增益 K(k)
for k = N-1:-1:1
    K(k) = (Q2 + B^2 * S(k+1)) \ (B * A * S(k+1));
    S(k) = A^2 * S(k+1) - A * B * K(k) * S(k+1) + Q1 + K(k)^2 * Q2;
end

% 初始化状态 x(k) 和控制量 u(k)
u = zeros(1, N);
x = zeros(1, N);
x(1) = 1;  % 初始状态 x(0)

% (3) 计算状态序列 x(k) 和控制输入序列 u(k)
for k = 1:N-1
    u(k) = -K(k) * x(k);        % 反馈控制规律
    x(k+1) = A * x(k) + B * u(k);  % 状态更新
end

% (4) 绘制结果图像
k = 0:N-1;

figure;

% 图1: Riccati 方程的解 S(k)
subplot(2,2,1);
plot(k, S, 'o-');
ylabel('S(k)');
xlabel('k');
title('Riccati Equation Solution S(k)');

% 图2: 反馈增益 K(k)
subplot(2,2,2);
plot(k, K, 'o-');
ylabel('K(k)');
xlabel('k');
title('State Feedback Gain K(k)');

% 图3: 状态 x(k)
subplot(2,2,3);
plot(k, x, 'o-');
ylabel('x(k)');
xlabel('k');
title('System State x(k)');

% 图4: 控制输入 u(k)
subplot(2,2,4);
plot(k, u, 'o-');
ylabel('u(k)');
xlabel('k');
title('Control Input u(k)');
