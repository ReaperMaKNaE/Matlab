function x=function_4_1_B(n)
for j=1:length(n)
    if n(j)<=60
        x(j)=(1/2)^(n(j)-1);
    elseif n(j)==61
        x(j)=1;
    else
        x(j)=x(j-60);
    end
end
