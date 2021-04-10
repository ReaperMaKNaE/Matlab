function [epsilon, J, H] = CostFunc(H, src, tar)
    % Find epsillon
    A_LM = zeros(2*size(src,1),9);
    J = zeros(2*size(src,1),4);
    %tar_LM = H*transpose(src);
    %tar_LM = transpose(tar_LM./tar_LM(3,:));
    
    for i=1:size(src,1)
        A_LM(2*i-1, 4:6, :) = -src(i,:);
        A_LM(2*i-1, 7:9, :) = tar(i,2)*src(i,:);
        A_LM(2*i, 1:3, :) = src(i,:);
        A_LM(2*i, 7:9, :) = -tar(i,1)*src(i,:);
        J(2*i-1, 1) = -H(2,1) + tar(i,2)*H(3,1);
        J(2*i-1, 2) = -H(2,2) + tar(i,2)*H(3,2);
        J(2*i-1, 4) = H(3, 1:3)*transpose(tar(i,1:3));
        J(2*i, 1) = -H(1,1) - tar(i,1)*H(3,1);
        J(2*i, 2) = -H(1,2) - tar(i,1)*H(3,2);
        J(2*i, 3) = -H(3, 1:3)*transpose(tar(i,1:3));
    end
    
    [~, ~, V_LM] = svd(A_LM);
    V_LM = transpose(V_LM);
    H_LM= [V_LM(9,1:3) V_LM(9,4:6) V_LM(9,7:9)];
    
    epsilon = A_LM*transpose(H_LM);
    H = [V_LM(9,1:3); V_LM(9,4:6); V_LM(9,7:9)]/H_LM(9);
end

