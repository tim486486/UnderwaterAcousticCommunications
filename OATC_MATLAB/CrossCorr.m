T = 80e-6;
f = 1/T;

fs = 10*f;

T_bit = 10*T;

samples_per_cycle = floor(fs/f);
samples_per_bit = 10*samples_per_cycle;

t = 0:1/fs:20*T_bit - 1/fs;

o = cos(2*pi*f*t(1:samples_per_bit));
z = -cos(2*pi*f*t(1:samples_per_bit));
n = zeros(1,samples_per_bit);

arrivals = [randi([1,samples_per_bit]) randi([samples_per_bit,2*samples_per_bit])];

sig_t = [z o z];
r1 = [zeros(1,arrivals(1)) sig_t];
r2 = [zeros(1,arrivals(2)), sig_t];
r1 = [r1 zeros(1,numel(r2) - numel(r1))];
sig_r = r1 + r2;
sig_r = awgn(sig_r,1.0);

[acor,lag] = xcorr(sig_r,sig_t);

subplot(311);
plot(t(1:numel(sig_t)),sig_t);
subplot(312);
plot(t(1:numel(sig_r)),sig_r);
subplot(313);
plot(lag,abs(acor));
title(strcat('Arrival 1: ',num2str(arrivals(1)), 'Arrival 2: ', num2str(arrivals(2))));