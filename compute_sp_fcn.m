function sp = compute_sp_fcn(xp, Deltax, Deltay, Nx, Ns, aX, bX)
% Computes sp given xp

intersects_unsafe = ( any( xp <= aX' ) || any( xp >= bX' ) );

if (~intersects_unsafe)
    n_cells_x = ceil(xp(1)/Deltax);
    n_cells_y = ceil(xp(2)/Deltay);
    sp = (n_cells_y - 1)*Nx + n_cells_x;
else % Post completely unsafe
    sp = Ns + 1;
end

% 
% 
% if (Ns < 65535)
%     sp = uint16(sp);
% else
%     Q_intersects = uint32(Q_intersects);
% end
% 
% end

end