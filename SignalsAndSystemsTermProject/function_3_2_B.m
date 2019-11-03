t=-10:0.01:10;
x_1=function_3_1_A(t);
h_1=function_3_1_B(t);
y_1=conv(x_1,h_1,'same')/100;
plot(t,y_1)