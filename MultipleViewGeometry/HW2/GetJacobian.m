function [epsilon, J] = GetJacobian(H, src, tar)
%GETJACOBIAN 이 함수의 요약 설명 위치
%   자세한 설명 위치
    J = zeros(2*size(src,1),9);
    tar_s = H*transpose(src);
    tar_s = transpose(tar_s./tar_s(3,:));
    for i=1:size(src,1)
        %f1 = H(1,1)*src(i,1) + H(1,2)*src(i,2) + H(1,3);
        %f2 = H(2,1)*src(i,1) + H(2,2)*src(i,2) + H(2,3);
        %f3 = H(3,1)*src(i,1) + H(3,2)*src(i,2) + H(3,3);
        f1 = tar_s(i,1);
        f2 = tar_s(i,2);
        f3 = tar_s(i,3);
        J(2*i-1, 1:3) = src(i,1:3)/f3;
        J(2*i-1, 7:9) = -src(i,1:3)*f1/f3^2;
        J(2*i, 4:6) = src(i,1:3)/f3;
        J(2*i, 7:9) = -src(i,1:3)*f2/f3^2;
    end
    %tar_s = H*transpose(src);
    %tar_s = tar_s./tar_s(3,:);
    epsilon = transpose(tar-tar_s);
    epsilon = reshape(epsilon(1:2,:), [2*size(src,1),1]);
end

