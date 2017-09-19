snd = audioread('plong.wav');
fs = 44100;
mono = snd(:,1);
%sound(mono, fs);

s = size(mono);
t = (1:s)/fs;

order    = 5;
fcut = 20;
[b,a]    = butter(order,fcut/(fs/2), 'low');

for df=1:15
    fm = 280+df;
    m = sin(2*pi*fm*t);
    mod = mono.*m';
    lpf = filter(b,a,mod);

    lpf(abs(lpf) < 0.001) = 0;

    figure(3);
    plot(t , lpf, t, sign(lpf));

    sound(sign(lpf)+mono, fs)
    pause(3)
end