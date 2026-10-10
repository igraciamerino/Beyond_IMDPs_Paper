function Qp_s_a = obtain_Qp_s_a_fcn( Sp, Nx, Ns )

% If Sp == {Ns + 1}
if ( ( length( Sp ) == 1 ) && ( Sp == Ns + 1 ) )
    Qp_s_a = cell(1);
    Qp_s_a{1} = Ns + 1;
    return;
end

if ( ~ismember( Ns + 1, Sp ) )
    length_Sp = length( Sp );
else
    length_Sp = length( Sp ) - 1;
end
estimated_length_q = 4*length_Sp + 1;

Qp_s_a = cell(estimated_length_q, 1);
for spi = 1:length_Sp
    sp_minus_1 = uint16( Sp(spi) - 1 );
    % Type 1 (2x2)
    n_cells_x_ub = 2;
    n_cells_y_ub = 2;
    q1 = cell_bounds2regions_fcn(1, n_cells_x_ub, 1, n_cells_y_ub, Nx);
    q1 = uint16(q1) + sp_minus_1;
    if ( any( ~ismember( q1, Sp ) ) )
        q1 = [];
    end
    % Type 2 (3x2)
    n_cells_x_ub = 3;
    n_cells_y_ub = 2;
    q2 = cell_bounds2regions_fcn(1, n_cells_x_ub, 1, n_cells_y_ub, Nx);
    q2 = uint16(q2) + sp_minus_1;
    if ( any( ~ismember( q2, Sp ) ) )
        q2 = [];
    end
    % Type 3 (2x3)
    n_cells_x_ub = 2;
    n_cells_y_ub = 3;
    q3 = cell_bounds2regions_fcn(1, n_cells_x_ub, 1, n_cells_y_ub, Nx);
    q3 = uint16(q3) + sp_minus_1;
    if ( any( ~ismember( q3, Sp ) ) )
        q3 = [];
    end
    % Type 4 (3x3)
    n_cells_x_ub = 3;
    n_cells_y_ub = 3;
    q4 = cell_bounds2regions_fcn(1, n_cells_x_ub, 1, n_cells_y_ub, Nx);
    q4 = uint16(q4) + sp_minus_1;
    if ( any( ~ismember( q4, Sp ) ) )
        q4 = [];
    end
        
    % Consider only qp that don't contain Ns + 1
    if ( ~ismember( Ns + 1, q1 ) )
        Qp_s_a{4*(spi - 1) + 1} = q1;
    end
    if ( ~ismember( Ns + 1, q2 ) )
        Qp_s_a{4*(spi - 1) + 2} = q2;
    end
    if ( ~ismember( Ns + 1, q3 ) )
        Qp_s_a{4*(spi - 1) + 3} = q3;
    end
    if ( ~ismember( Ns + 1, q4 ) )
        Qp_s_a{4*(spi - 1) + 4} = q4;
    end
end
% Account for Ns + 1
Qp_s_a{end} = uint16(Ns + 1);

Qp_s_a = Qp_s_a(~cellfun('isempty', Qp_s_a));

end