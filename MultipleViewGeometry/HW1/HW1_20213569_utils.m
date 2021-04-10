classdef HW1_20213569_utils
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    methods (Static = true)
        function out = getConic(k, green, red, blue)
            out = ones(5, 5);
            out(1,1:5) = getConicRow(green(k,1:3), red(k,1:3));
            out(2,1:5) = getConicRow(green(k,1:3), red(k+1,1:3));
            out(3,1:5) = getConicRow(green(k+1,1:3), red(k,1:3));
            out(4,1:5) = getConicRow(green(k+1,1:3), red(k+1,1:3));
            out(5,1:5) = getConicRow(blue(k,1:3), blue(k+1,1:3));
        end  
    end
end

function obj = getConicRow(mat1, mat2)
    obj = ones(1,5);
    obj(1) = mat1(1) * mat2(1);
    obj(2) = (mat1(1) * mat2(2) + mat1(2) * mat2(1)) / 2;
    obj(3) = mat1(2) * mat2(2);
    obj(4) = (mat1(1) * mat2(3) + mat1(3) * mat2(1)) / 2;
    obj(5) = (mat1(2) * mat2(3) + mat1(3) * mat2(2)) / 2;
end
