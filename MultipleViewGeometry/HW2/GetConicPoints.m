function [xp,yp] = GetConicPoints(conicMat, numPoints, noise)
% args
%   conicMat  : The matrix of conic
%   imgSize   : The size of image(which will be plot)
%   numPoints : The number of points to plot conic on image
%   noise     : If there are noise, make noise
syms x y
x = -1:2/numPoints:1;
equation = conicMat(1,1).*x.^2 + conicMat(1,2).*x.*y + conicMat(2,2).*y.^2 + ...
          conicMat(1,3).*x + conicMat(2,3).*y + conicMat(3,3) == 0;
val = solve(equation, y);
y = val;

xp = 0;
yp = 0;
cnt= 0;
for k = 1:size(y,2)
    for m = 1:size(y,1)
        if imag(y(m,k))==0
            cnt = cnt + 1;
            if noise == 1
                xp(cnt) = x(k) + noise*randn;
                yp(cnt) = y(m,k) + noise*randn;
            else
                xp(cnt) = x(k);
                yp(cnt) = y(m,k);
            end
        end
    end
end

end

