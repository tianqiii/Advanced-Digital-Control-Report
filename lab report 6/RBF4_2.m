clc;
clear;
close all;
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
