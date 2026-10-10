function [Qp_intersects_bool, Qp_subsumed_bool, error_flag] = compute_Qp_multi_imdp_old_fcn(a_Post, b_Post, vertices_q, children_q, Ns, Nc, precision, a_X, b_X)
% Compute bounds for transitions starting from q, under a

a_Post = a_Post - precision;
b_Post = b_Post + precision;

intersects_X = ( ~any(b_Post < a_X) && ~any(a_Post > b_X) ); % Post_q_a_w (might have) vertices inside bounds in x and y (might intersect X)
intersects_unsafe = ( any(a_Post <= a_X) || any(b_Post >= b_X) ); % In case that Post_q_a_w intersects unsafe set

if (intersects_X)
    Qp_intersects_bool = true(Nc + 1, 1);
    Qp_subsumed_bool = Qp_intersects_bool;
    for ci = 1:Nc
        if ( ~ismember( Ns + 1, children_q{ci} ) )
            a_q = vertices_q{ci}(:, 1);
            b_q = vertices_q{ci}(:, end);
            if ( any( b_Post < a_q) || any( a_Post > b_q) )
                Qp_intersects_bool(ci) = false;
                Qp_subsumed_bool(ci) = false;
            else
                if ( any( a_Post < a_q) || any( b_Post > b_q) )
                    Qp_subsumed_bool(ci) = false;
                end
            end
        else
            % Don't count subsumed, but count intersects
            Qp_subsumed_bool(ci) = false;
        end
        if ( Qp_subsumed_bool(ci) && ~Qp_intersects_bool(ci))
            error_flag = true;
            return;
        end
    end
else
    Qp_intersects_bool = false(Nc + 1, 1);
    Qp_subsumed_bool = Qp_intersects_bool;
end

if ( intersects_unsafe )
    Qp_intersects_bool(end) = true;
else
    Qp_intersects_bool(end) = false;
end

if ( intersects_X )
    Qp_subsumed_bool(end) = false;
else
    Qp_subsumed_bool(end) = true;
end

if ( ~any(Qp_intersects_bool) )
    error_flag = true;
else
    error_flag = false;
end