t=-10:0.01:10;
x_1=function_3_1_A(t);
h_1=function_3_1_B(t);
X_1=fft(x_1);
H_1=fft(h_1);
Y_1=X_1.*H_1;
plot(t,fftshift(ifft(Y_1))/100)