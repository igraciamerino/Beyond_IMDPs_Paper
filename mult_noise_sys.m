function [xp] = mult_noise_sys(x, w, A)

xp = w.*A*x;

end