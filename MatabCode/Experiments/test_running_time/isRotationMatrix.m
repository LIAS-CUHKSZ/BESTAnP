function isRotation = isRotationMatrix(R)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  输入：3*3 的矩阵
    %  输出：
    %       判断是否是旋转矩阵： 1 是；  0 不是
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 检查矩阵是否为方阵
    [rows, cols] = size(R);
    if rows ~= cols
        isRotation = false;
        return;
    end
    
    % 检查是否为正交矩阵 (R' * R = I)
    I = eye(size(R)); % 生成单位矩阵
    isOrthogonal = norm(R' * R - I) < 1e-6; % 允许一定的数值误差
    
    % 检查行列式是否为 1
    detR = det(R);
    isDetOne = abs(detR - 1) < 1e-6; % 允许一定的数值误差
    
    % 组合条件判断
    isRotation = isOrthogonal && isDetOne;
end