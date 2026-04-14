function [ f, gain ] = calc_fft( t, y )
T = mean( diff(t) );
Fs = 1/T;

start_time = t(1);
end_time = t(end);

t_ = start_time:T:end_time;
y_ = interp1( t, y, t_ );
L = length(y_);

NFFT = 2^nextpow2(L);
Y = fft( y, NFFT ) / L;
f = Fs/2*linspace(0,1,NFFT/2);
gain = abs(Y(1:(NFFT/2)));
