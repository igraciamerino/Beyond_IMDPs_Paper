function is_correct_yn = check_abstraction_mdpst_fcn(solution, error_flags, solver_opts, user_param)

Qp = solution.Qp;
P_c = solution.P_c;
S_not_terminal = user_param.S_not_terminal;

error_flag_computing_abstraction = false;
error_flag_check_abstraction = false;
if (error_flag_computing_abstraction == false)
    for s = S_not_terminal
        for a = 1:user_param.Na
            if ( ( length( Qp{s,a}.children ) == user_param.Nc + 1 ) && ( length(P_c) == user_param.Nc + 1 ) )
                
            else
                disp('Error computing the abstraction')
                error_flag_check_abstraction = true;
                break
            end
            if ( ( abs( sum(P_c) - 1 ) > solver_opts.precision) )
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

% More checks

if ( (error_flag_check_abstraction == false) && ( ~error_flags.error_flag_intersections ) && ( ~error_flags.error_flag_post ) )
    disp('Abstraction is correct')
    is_correct_yn = true;
else
    disp('Problem found during abstraction')
    is_correct_yn = false;
end

end