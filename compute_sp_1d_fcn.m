function s = compute_sp_1d_fcn(x, Deltax, Ns, aX, bX)
% Computes sp given xp

intersects_unsafe = ( any( x <= aX' ) || any( x >= bX' ) );

if (~intersects_unsafe)
    s = ceil((x(1)-aX)/Deltax);
else % Post completely unsafe
    s = Ns + 1;
end

end