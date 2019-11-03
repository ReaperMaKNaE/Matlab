t=-10:0.01:10;
x_1=function_3_1_A(t);
h_1=function_3_1_B(t);
y_1=conv(x_1,h_1,'same')/100;
X_1=fft(x_1);
H_1=fft(h_1);
y_1_ift=ifft(X_1.*H_1)/100;
subplot(2,2,1);
plot(t,y_1);
subplot(2,2,2);
plot(t,fftshift(y_1_ift));
subplot(2,2,[3, 4]);
plot(t,y_1,'-o','MarkerIndices',1:50:length(y_1));
hold on
plot(t,fftshift(y_1_ift));