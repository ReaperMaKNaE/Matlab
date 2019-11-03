n=1:100;
x_2=function_4_1_A(n);
x_2=[x_2 zeros(1,length(n)-1)];
X_2=fft(x_2);
h_2=function_4_1_B(n);
h_2=[h_2 zeros(1,length(n)-1)];
H_2=fft(h_2);
y_2=conv(x_2,h_2);
Y_2=X_2.*H_2;
y_2_ift=ifft(Y_2);
subplot(2,2,1)
stem(y_2)
xlim([1 150])
xlabel 'y_2'
subplot(2,2,2)
stem(y_2_ift)
xlim([1 150])
xlabel 'y_2 (ifft)'
subplot(2,2,[3,4])
stem(y_2)
hold on
plot(y_2_ift,'x')
xlim([1 150])