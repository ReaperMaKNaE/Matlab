n=1:60;
x_2=function_4_1_A(n);
h_2=function_4_1_B(n);
y_2=conv(x_2,h_2);
stem(y_2)
xlim([0 60])