function is_correct_yn = check_abstraction_mdpust_2layer_overlapping_fcn(solution, error_flags, solver_opts, user_param)

Qp = solution.Qp;
P_lower_c = solution.P_lower_c;
P_upper_c = solution.P_upper_c;
S_not_terminal = user_param.S_not_terminal;


error_flag_computing_abstraction = false;
error_flag_check_abstraction = false;
if (error_flag_computing_abstraction == false)
    for s = S_not_terminal
        for a = 1:user_param.Na
            if (  ( length(P_lower_c) == user_param.Nc + 1 ) && ( length(P_upper_c) == user_param.Nc + 1 ) )
                if ( any( P_lower_c > P_upper_c + solver_opts.precision ) )
                    disp('Error computing the abstraction')
                    error_flag_check_abstraction = true;
                    break
                end
            else
                disp('Error computing the abstraction')
                error_flag_check_abstraction = true;
                break
            end
            if ( (sum(P_lower_c) > 1 + solver_opts.precision) || (sum(P_upper_c) < 1 - solver_opts.precision) )
                disp('Error computing the abstraction')
                error_flag_check_abstraction = true;
                break
            end
            if (error_flag_check_abstraction)
                break
            end
        end
        if (error_flag_check_abstraction)
            break
        end
    end
    
end

% Check coherence between coarse and fine partitions

for c_coarse = 1:user_param.Nc_coarse + 1
    children = solution.children{c_coarse}';
    if ( ( sum( solution.P_lower_c(children) ) > solution.P_lower_c_coarse(c_coarse) + solver_opts.precision ) || ( sum( solution.P_upper_c(children) ) < solution.P_upper_c_coarse(c_coarse) - solver_opts.precision ) ||( sum( solution.P_lower_c(children) ) > solution.P_upper_c_coarse(c_coarse) + solver_opts.precision ) ||( sum( solution.P_upper_c(children) ) < solution.P_lower_c_coarse(c_coarse) - solver_opts.precision ) )
        disp('Error computing the abstraction')
        error_flag_check_abstraction = true;
    end
end

% More checks

if ( (error_flag_check_abstraction == false) && ( ~error_flags.error_flag_intersections ) && ( ~error_flags.error_flag_post ) )
    disp('Abstraction is correct')
    is_correct_yn = true;
else
    disp('Problem found during abstraction')
    is_correct_yn = false;
end

end