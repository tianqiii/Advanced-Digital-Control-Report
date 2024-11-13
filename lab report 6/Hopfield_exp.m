clc;
clear;
close all;
% 参数初始化
% 权值矩阵（6节点）
W = [0 1.2 -1.6 0 0 1;
     1 0 1 -1 0 0.5;
     -1 1 0 1.7 -1 0.7;
     0 -1 1.1 0 1.6 -1;
     0 0.7 -1.1 1 0 1;
     1 0 0 -1.9 1 0.5];  

% 初态
S = [0.3; -0.5; 1.1; 0.6; -0.4; 0.3];  % 随机设置初始状态

theta = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5];  % 阈值向量

iterations = 10;  % 更新迭代次数


% 定义sign函数
sign_fn = @(x) (x >= 0) * 2 - 1;  % 1 if x >= 0, -1 otherwise

% 能量函数计算
calculate_energy = @(S, W, theta) ...
    -0.5 * S' * W * S + sum(theta .* S);

% 输出初始状态和能量
disp('Initial state:'), disp(S');
E = calculate_energy(S, W, theta);
fprintf('Initial Energy: %.4f\n\n', E);

% 逐步更新节点状态
for t = 1:iterations
    fprintf('Iteration %d:\n', t);
    state_changed = false;  % 标记状态是否变化
    for i = 1:length(S)
        % 计算净输入
        net_input = W(i,:) * S - theta(i);
        new_state = sign_fn(net_input);
        
        % 检查节点状态是否发生变化
        if new_state ~= S(i)
            state_changed = true;
        end
        
        % 更新节点状态
        S(i) = new_state;
        fprintf('Node %d updated to %d\n', i, S(i));
    end
    
    % 显示当前状态
    disp('Current state:'), disp(S');
    
    % 计算能量
    E = calculate_energy(S, W, theta);
    fprintf('Energy: %.4f\n\n', E);
    
    % 如果状态不再变化，停止迭代
    if ~state_changed
        fprintf('神经网络已经收敛.\n');
        break;
    end
end

