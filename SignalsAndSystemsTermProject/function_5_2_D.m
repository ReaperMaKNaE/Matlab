timeconstant=0.2;
x_fft=fft(x);
subplot(2,1,1)
sample=length(x);
x_positive=x_fft(1:sample/2);
plot([1 : sample/2], abs(x_positive));
xlabel('frequency')
title('FFT of x[n]')
ylabel('Magnitude')
subplot(2,1,2)
y=function_5_2_A(timeconstant,x);
y_fft=fft(y);
y_positive=y_fft(1:sample/2);
plot([1: sample/2], abs(y_positive));
xlabel('frequency')
ylabel('Magnitude')
title('FFT of y[n]')