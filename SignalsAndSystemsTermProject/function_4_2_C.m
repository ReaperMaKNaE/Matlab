n=1:60;
x_2=function_4_1_A(n);
h_2=function_4_1_B(n);
X_2=fft(x_2);
H_2=fft(h_2);
subplot(2,2,1)
stem(abs(X_2))
xlabel 'Frequency'
ylabel '|X_2(k)|'
subplot(2,2,2)
stem(abs(H_2))
xlabel 'Frequency'
ylabel '|H_2(k)|'
subplot(2,2,3)
stem(angle(X_2))
xlabel 'Frequency'
ylabel 'Phase'
subplot(2,2,4)
stem(angle(H_2))
xlabel 'Frequency'
ylabel 'Phase'