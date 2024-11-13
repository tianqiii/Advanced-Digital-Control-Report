clc;
clear;
close all;

% 第一步，设计PID控制器进行信号 𝑟=0.5sin⁡(10𝑡)跟踪

% 数据生成
ld = 400;
x = rand(2, ld); % 生成一个2行ld列的随机矩阵，值在0到1之间  
x = (x - 0.5) * 1.5 * 2; %将数据缩放到-1.5到1.5之间  
x1 = x(1,:); % 提取第一行作为x1  
x2 = x(2,:); % 提取第二行作为x2  
F = 20 + x1.^2 - 10 * cos(2 * pi * x1) + x2.^2 - 10 * cos(2 * pi * x2); % 目标函数

% 创建RBF径向基网络 - 近似解
net_rb = newrb(x, F);  % 近似 RBF 网络

% 创建RBF径向基网络 - 精确解
net_rbe = newrbe(x, F);  % 精确 RBF 网络

% 生成测试数据
interval = 0.1; %步长
[i, j] = meshgrid(-1.5:interval:1.5, -1.5:interval:1.5);     % 生成网格点  
row = size(i); % 获取网格大小 

% 将i，j转换为行向量作为输入数据
tx1 = i(:)';
tx2 = j(:)';
tx = [tx1; tx2]; 

% 使用两种网络进行测试
ty_rb = sim(net_rb, tx);   % 近似 RBF 网络的输出
ty_rbe = sim(net_rbe, tx);  % 精确 RBF 网络的输出

% 使用图像查看两种网络对非线性函数的拟合效果
figure(1)
plot3(x1, x2, F, 'b*');
hold on;
title('原函数')
grid on
xlabel('x1')
ylabel('x2')
zlabel('F')

% 将网络结果重塑为网格形状
v_rb = reshape(ty_rb, row);  % 近似 RBF 网络的拟合结果
v_rbe = reshape(ty_rbe, row); % 精确 RBF 网络的拟合结果

figure(2)
subplot(1,3,1)      
mesh(i, j, v_rb);
zlim([0, 60])       
title('近似RBF神经网络结果')

% 绘制原函数图像
interval = 0.1;
[x1_mesh, x2_mesh] = meshgrid(-1.5:interval:1.5);
F_mesh = 20 + x1_mesh.^2 - 10 * cos(2 * pi * x1_mesh) + x2_mesh.^2 - 10 * cos(2 * pi * x2_mesh);
subplot(1,3,2)
mesh(x1_mesh, x2_mesh, F_mesh);
zlim([0, 60])
title('真正的函数图像')

% 绘制误差图
subplot(1,3,3)
mesh(x1_mesh, x2_mesh, F_mesh - v_rb);  % 误差图
zlim([0, 60])
title('近似RBF误差图像')

% 显示精确RBF结果
figure(3)
subplot(1,2,1)
mesh(i, j, v_rbe);
zlim([0, 60])       
title('精确RBF神经网络结果')

% 精确RBF误差图
subplot(1,2,2)
mesh(x1_mesh, x2_mesh, F_mesh - v_rbe); % 误差图
zlim([0, 60])
title('精确RBF误差图像')

% 系统传递函数
G = tf(400, [1 50 0]);

% PID控制器参数
Kp = 10;  % 你可以通过试验进行调节
Ki = 0.5;  % 积分增益
Kd = 0.1;  % 微分增益

% PID控制器
C = pid(Kp, Ki, Kd);

% 闭环系统
sys_cl = feedback(C*G, 1);

% 输入信号 r = 0.5sin(10t)
t = 0:0.01:10;  % 时间向量
r = 0.5*sin(10*t);  % 输入信号

% 仿真响应
[y, t, x] = lsim(sys_cl, r, t);

% 绘制响应曲线
figure;
plot(t, r, 'r--', t, y, 'b');
xlabel('Time (s)');
ylabel('Amplitude');
legend('Reference signal', 'PID output');
title('PID Control Tracking');
grid on;

 % 第二步，使用RBF和BP神经网络训练跟踪PID控制器

 % 训练数据准备
% 确保t和r的维度一致
t = t(:)';  % 转置成行向量
r = r(:)';  % 转置成行向量

% 训练数据准备
train_in = [t; r];  % 输入：时间和信号
train_out = y;      % 输出：PID控制器的输出
train_out = y(:)';  % 保证输出也是行向量

% 创建RBF网络
spread = 1;  % 影响范围
net_rbf = newrb(train_in, train_out, 0, spread);

% 测试RBF网络
ty_rbf = sim(net_rbf, [t; r]);

% 绘制RBF网络的跟踪效果
figure;
plot(t, y, 'b', t, ty_rbf, 'g--');
xlabel('Time (s)');
ylabel('Amplitude');
legend('PID output', 'RBF output');
title('RBF Neural Network Tracking');
grid on;

% 创建BP神经网络
hidden_layer_size = 10;  % 隐藏层神经元数量
net_bp = feedforwardnet(hidden_layer_size);

% 训练BP神经网络
net_bp = train(net_bp, train_in, train_out);

% 测试BP网络
ty_bp = sim(net_bp, [t; r]);

% 绘制BP网络的跟踪效果
figure;
plot(t, y, 'b', t, ty_bp, 'r--');
xlabel('Time (s)');
ylabel('Amplitude');
legend('PID output', 'BP output');
title('BP Neural Network Tracking');
grid on;

%第三步，模糊神经网络训练并比较效果

% 准备训练数据
trainData = [t' r' y];  % 时间，输入信号，PID输出

% 初始化模糊神经网络
opt = anfisOptions('InitialFIS', 5);  % 使用五个模糊隶属函数
anfis_model = anfis(trainData, opt);

% 测试模糊神经网络
ty_fuzzy = evalfis([t' r'], anfis_model);

% 绘制模糊神经网络的跟踪效果
figure;
plot(t, y, 'b', t, ty_fuzzy, 'm--');
xlabel('Time (s)');
ylabel('Amplitude');
legend('PID output', 'Fuzzy Neural Network output');
title('Fuzzy Neural Network Tracking');
grid on;

figure;
plot(t, y, 'b', t, ty_rbf, 'g--', t, ty_bp, 'r--', t, ty_fuzzy, 'm--');
xlabel('Time (s)');
ylabel('Amplitude');
legend('PID output', 'RBF output', 'BP output', 'Fuzzy output');
title('Comparison of Tracking Performance');
grid on;
