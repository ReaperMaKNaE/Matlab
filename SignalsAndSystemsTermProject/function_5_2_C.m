timeconstant=0.2;
subplot(3,1,1)
plot(x)
subplot(3,1,2)
num=1;
den=[1 timeconstant];
bodemag(tf(num,den));   %%plotting magnitude of bode plot of LPF
subplot(3,1,3)
y=function_5_2_A(timeconstant,x);
plot(y)