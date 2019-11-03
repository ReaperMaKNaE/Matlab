y=fft(x);
subplot(3,2,1)
plot(x)
title('Original voice')
subplot(3,2,2)
sample=length(x);
shift_y=y([sample/2+1 : sample 1:sample/2]);
plot([-sample/2+1 : sample/2], abs(shift_y));
title('Original voice in frequency domain')
subplot(3,2,3)
t=1:length(x);
t_trans=transpose(t);
frequency=20;
r=x.*cos(frequency*t_trans);
r_fft=fft(r);
shift_r=r_fft([sample/2+1 : sample 1:sample/2]);
plot([-sample/2+1 : sample/2],abs(shift_r))
title('r(t) in frequency domain')
subplot(3,2,4)
g=r.*cos(frequency*t_trans);
g_fft=fft(g);
shift_g=g_fft([sample/2+1 : sample 1:sample/2]);
plot([-sample/2+1 : sample/2],abs(shift_g))
title('g(t) in frequency domain')
subplot(3,2,5)
fre_s=frequency_s(x,shift_y);
h=LPF(x,fre_s);
filtered_g=shift_g.*h;
plot([-sample/2+1 : sample/2],abs(filtered_g))
title('filtered voice in frequency domain')
subplot(3,2,6)
fft_filtered_voice=filtered_g([sample/2+1 : sample 1:sample/2]);
plot(ifft(fft_filtered_voice))
title('filtered voice in time domain')