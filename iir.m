Fs = 1953.128;

f = fliplr([329.63 246.94 196.00 146.83 110.00 82.41]);

%% Low pass filter
figure(1);
[b,a] = butter(2,20*2/Fs);
as = round(a*2^11);
bs = round(b*2^11);
fprintf('{0, 0, 0, 0, %d, %d, %d, %d, %d},\n', bs(1), bs(2), bs(3), -as(2), -as(3))

[h,w] = freqz(b,a,512);
[hs,ws] = freqz(bs,as,512);
plot(w*Fs/pi/2,abs(h), ws*Fs/pi/2,abs(hs))
legend('chebby Design', 'scaled')
xlabel 'Radian Frequency (\omega/\pi)', ylabel 'Magnitude'

%% band pass filters
figure(2);
hold on
for n = 1:6
    %[b,a] = cheby1(2,3,f(n)*2/Fs);
    [b, a] = iirpeak(f(n)*2/Fs,50*2/Fs);
    as = round(a*2^11);
    bs = round(b*2^11);
    fprintf('{0, 0, 0, 0, %d, %d, %d, %d, %d},\n', bs(1), bs(2), bs(3), -as(2), -as(3))

    [h,w] = freqz(b,a,512);
    [hs,ws] = freqz(bs,as,512);
    plot(w*Fs/pi/2,abs(h), ws*Fs/pi/2,abs(hs))
    legend('chebby Design', 'scaled')
    xlabel 'Radian Frequency (\omega/\pi)', ylabel 'Magnitude'
end

