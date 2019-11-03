recObj = audiorecorder;
disp('Start speaking.')
recordblocking(recObj, 5); % 5 seconds
disp('End of Recording.');
play(recObj);
x = getaudiodata(recObj);
plot(x);