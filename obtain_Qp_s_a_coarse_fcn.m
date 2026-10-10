function Qp_s_a = obtain_Qp_s_a_coarse_fcn( Sp_coarse, Nx_coarse, Ns_coarse )

% If Sp == {Ns + 1}
if ( ( length( Sp_coarse ) == 1 ) && ( Sp_coarse == Ns_coarse + 1 ) )
    Qp_s_a = cell(1);
    Qp_s_a{1} = Ns_coarse + 1;
    return;
end

if ( ~ismember( Ns_coarse + 1, Sp_coarse ) )
    length_Sp_coarse = length( Sp_coarse );
else
    length_Sp_coarse = length( Sp_coarse ) - 1;
end
estimated_length_q_coarse = 4*length_Sp_coarse + 1;

Qp_s_a = cell(estimated_length_q_coarse, 1);
for sp_coarsei = 1:length_Sp_coarse
    sp_coarse_minus_1 = uint16( Sp_coarse(sp_coarsei) - 1 );
    % Type 1 (2x2)
    n_cells_coarse_x_ub = 1;
    n_cells_coarse_y_ub = 1;
    q1 = cell_bounds2regions_fcn(1, n_cells_coarse_x_ub, 1, n_cells_coarse_y_ub, Nx_coarse);
    q1 = uint16(q1) + sp_coarse_minus_1;
    if ( any( ~ismember( q1, Sp_coarse ) ) )
        q1 = [];
    end
    % Type 2 (3x2)
    n_cells_coarse_x_ub = 1;
    n_cells_coarse_y_ub = 2;
    q2 = cell_bounds2regions_fcn(1, n_cells_coarse_x_ub, 1, n_cells_coarse_y_ub, Nx_coarse);
    q2 = uint16(q2) + sp_coarse_minus_1;
    if ( any( ~ismember( q2, Sp_coarse ) ) )
        q2 = [];
    end
    % Type 3 (2x3)
    n_cells_coarse_x_ub = 2;
    n_cells_coarse_y_ub = 1;
    q3 = cell_bounds2regions_fcn(1, n_cells_coarse_x_ub, 1, n_cells_coarse_y_ub, Nx_coarse);
    q3 = uint16(q3) + sp_coarse_minus_1;
    if ( any( ~ismember( q3, Sp_coarse ) ) )
        q3 = [];
    end
    % Type 4 (3x3)
    n_cells_coarse_x_ub = 2;
    n_cells_coarse_y_ub = 2;
    q4 = cell_bounds2regions_fcn(1, n_cells_coarse_x_ub, 1, n_cells_coarse_y_ub, Nx_coarse);
    q4 = uint16(q4) + sp_coarse_minus_1;
    if ( any( ~ismember( q4, Sp_coarse ) ) )
        q4 = [];
    end
        
    % Consider only qp that don't contain Ns + 1
    if ( ~ismember( Ns_coarse + 1, q1 ) )
        Qp_s_a{4*(sp_coarsei - 1) + 1} = q1;
    end
    if ( ~ismember( Ns_coarse + 1, q2 ) )
        Qp_s_a{4*(sp_coarsei - 1) + 2} = q2;
    end
    if ( ~ismember( Ns_coarse + 1, q3 ) )
        Qp_s_a{4*(sp_coarsei - 1) + 3} = q3;
    end
    if ( ~ismember( Ns_coarse + 1, q4 ) )
        Qp_s_a{4*(sp_coarsei - 1) + 4} = q4;
    end
end
% Account for Ns + 1
Qp_s_a{end} = uint16(Ns_coarse + 1);

Qp_s_a = Qp_s_a(~cellfun('isempty', Qp_s_a));

end