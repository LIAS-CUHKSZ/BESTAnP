function R_corrected = ToRotationMatrix(R)
    % 对矩阵 R 进行奇异值分解
    [U, ~, V] = svd(R);
    
    % 矫正矩阵为旋转矩阵
    R_corrected = U * V';
    
    % 检查行列式，如果行列式为 -1，则需要调整
    if det(R_corrected) < 0
        U(:, end) = -U(:, end);
        R_corrected = U * V';
    end
end