function is_correct_yn = check_abstraction_IMDP_naive_fcn(solution, error_flags, solver_opts, user_param)

Qp = solution.Qp;
Sp = 1:user_param.Ns+1;
children = solution.children;
P_lower = solution.P_lower;
P_upper = solution.P_upper;
P_lower_coarse = solution.P_lower_coarse;
P_upper_coarse = solution.P_upper_coarse;
Ns = user_param.Ns;
S_not_terminal = user_param.S_not_terminal;
error_flag_computing_abstraction = error_flags.error_flag_computing_abstraction;

error_flag_check_abstraction = false;
if (error_flag_computing_abstraction == false)
    for s = S_not_terminal
        for a = 1:user_param.Na
            if ( ~isempty(length(Sp)) || (length(Sp) == length(P_lower{s,a})) || (length(Sp) == length(P_upper{s,a})) )
                for qp_ind = 1:length(Sp)
                    if (P_lower{s,a}(qp_ind) > P_upper{s,a}(qp_ind) + solver_opts.precision)
                        disp('Error computing the abstraction')
                        error_flag_check_abstraction = true;
                        break
                    end
                end
            else
                disp('Error computing the abstraction')
                error_flag_check_abstraction = true;
                break
            end
            if ( (sum(P_lower{s,a}) > 1 + solver_opts.precision) || (sum(P_upper{s,a}) < 1 - solver_opts.precision) )
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

if ( (error_flag_check_abstraction == false) && ( ~error_flags.error_flag_intersections ) && ( ~error_flags.error_flag_post ) && ( ~error_flags.error_flag_subsumed ) )
    disp('Abstraction is correct')
    is_correct_yn = true;
else
    disp('Problem found during abstraction')
    is_correct_yn = false;
end

end