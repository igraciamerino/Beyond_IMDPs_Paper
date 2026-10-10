function is_correct_yn = check_abstraction_2layer_single_goal_fcn(solution, error_flags, solver_opts, user_param)

Qp = solution.Qp;
Qp_flag_removed = solution.Qp_flag_removed;
Sp = solution.Sp;
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
            if ( ~isempty( Sp{s,a} ) || (length(Sp{s,a}) == length(P_lower{s,a})) || (length(Sp{s,a}) == length(P_upper{s,a})) )
                for qp_ind = 1:length(Sp{s,a})
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

% More checks

if (error_flag_computing_abstraction == false)
    for s = S_not_terminal
        for a = 1:user_param.Na
            if ( ~isempty( Qp{s,a} ) || ( isempty( Qp{s,a} ) && ~any( ~Qp_flag_removed{s,a} ) ) )
                if ( (length(Qp{s,a}) == length(P_lower_coarse{s,a})) || (length(Qp{s,a}) == length(P_upper_coarse{s,a})) )
                    for qp_ind = 1:length(Qp{s,a})
                        if (P_lower_coarse{s,a}(qp_ind) > P_upper_coarse{s,a}(qp_ind) + solver_opts.precision)
                            disp('Error computing the abstraction')
                            error_flag_check_abstraction = true;
                            break
                        end
                    end
                end
            else
                disp('Error computing the abstraction')
                error_flag_check_abstraction = true;
                break
            end
            
            % Check coherence between coarse and fine abstractions
            for qp_ind = 1:length(Qp{s,a})
                qp = Qp{s,a}(qp_ind);
                qp_children = children{qp}';
                mysum_lb = 0;
                mysum_ub = 0;
                for qp_child = qp_children
                    qp_child_index_in_Sp = find(Sp{s,a} == qp_child);
                    if (~isempty(qp_child_index_in_Sp))
                        mysum_lb = mysum_lb + P_lower{s,a}(qp_child_index_in_Sp);
                        mysum_ub = mysum_ub + P_upper{s,a}(qp_child_index_in_Sp);
                    end
                end
                if ( (mysum_lb > P_lower_coarse{s,a}(qp_ind) + solver_opts.precision) || (mysum_ub + solver_opts.precision < P_upper_coarse{s,a}(qp_ind)) )
                    disp('Error computing the abstraction')
                    error_flag_check_abstraction = true;
                    break
                end
            end
            % This one below might not be met for the upper bound (bc not
            % considering qp with children outside Sp). Consider only the
            % case of the lower bound
            if ( ~isempty( Qp{s,a} ) )
                if ( (sum(P_lower_coarse{s,a}) > 1 + solver_opts.precision) )
                    disp('Error computing the abstraction')
                    error_flag_check_abstraction = true;
                    break
                end
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


if ( (error_flag_check_abstraction == false) && ( ~error_flags.error_flag_intersections ) && ( ~error_flags.error_flag_intersections_nd_coarse ) && ( ~error_flags.error_flag_intersections_nd ) && ( ~error_flags.error_flag_intersections_nd_coarse ) && ( ~error_flags.error_flag_post ) && ( ~error_flags.error_flag_subsumed ) )
    disp('Abstraction is correct')
    is_correct_yn = true;
else
    disp('Problem found during abstraction')
    is_correct_yn = false;
end

end