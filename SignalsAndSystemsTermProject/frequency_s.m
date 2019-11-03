function frequency = frequency_s(x,shift_y)
cnt = 0;
if mod(length(x),2)==1
    k=(length(x)+1)/2;
else
    k=length(x)/2;
end
for i=k:length(x)-500
    for j=1:500
        if shift_y(i+j)<=5
            cnt=cnt+1;
        else
            cnt=0;
        end
    end
    if cnt==500
        frequency=i;
        break;
    else
        cnt=0;
    end
end