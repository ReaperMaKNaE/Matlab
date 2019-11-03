n=1:60;
x_2=function_4_1_A(n);
h_2=function_4_1_B(n);
X_2=fft(x_2);
H_2=fft(h_2);
Y_2=X_2.*H_2;
subplot(2,1,1)
stem(abs(Y_2))
xlabel 'Frequency'
ylabel '|Y_2(k)|'
subplot(2,1,2)
stem(angle(Y_2))
xlabel 'Frequency'
ylabel 'Phase'