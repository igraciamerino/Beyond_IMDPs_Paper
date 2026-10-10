function [Sp_subsumed_bool, error_flag] = subsumed_q_a_w_simplified_n_dim_fcn(Sp_intersects)
% Checks containment in the regions of a non-overlapping partition

if ( length(Sp_intersects) > 1)
    Sp_subsumed_bool = false(size(Sp_intersects));
    error_flag = false;
    return;
end

% Sp_subsumed_bool = (Sp_intersects == Sp);
% if ( any(Sp_subsumed_bool) )
%     error_flag = false;
% else
%     error_flag = true;
%     disp('ERROR: Problem with subsumed...')
% end
Sp_subsumed_bool = true;
error_flag = false;

end