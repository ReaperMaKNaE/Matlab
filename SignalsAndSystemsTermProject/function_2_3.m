a_n=ones(1,4);
b_n=zeros(1,7);
n1=numel(a_n);
n2=numel(b_n);
x_n=zeros(1,n2);
for i=1:n1
if a_n(i)==1
x_n(i)=1;
else
end
end
n=1:n2;
stem(n,x_n)