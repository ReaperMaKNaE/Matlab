n=1:60;
x_2=function_4_1_A(n);
h_2=function_4_1_B(n);
X_2=fft(x_2);
H_2=fft(h_2);
y_2_ift=ifft(X_2.*H_2);
stem(y_2_ift)