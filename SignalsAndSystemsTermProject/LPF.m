function LPFfilter = LPF(x,frequency_s)
i=length(x)-frequency_s;
k=frequency_s-length(x)/2;
filter=[zeros(1,i), ones(1,2*k), zeros(1,i)];
LPFfilter=transpose(filter);