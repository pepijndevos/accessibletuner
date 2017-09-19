f = [0.0 0.3 0.4 1.0];
a = [1.0 1.0 0.0 0.0];
b = firpm(6,f,a);

[h,w] = freqz(b,1,512);
plot(f,a,w/pi,abs(h))
legend('Ideal','firpm Design')
xlabel 'Radian Frequency (\omega/\pi)', ylabel 'Magnitude'
