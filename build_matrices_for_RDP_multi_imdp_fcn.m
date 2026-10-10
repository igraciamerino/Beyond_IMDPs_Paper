function [A, b, Aeq, beq] = build_matrices_for_RDP_multi_imdp_fcn(P_lower_coarse, P_upper_coarse, Sp, children_q)

% New (I use \sum \gamma \ge 1)
Nq = length(children_q);

Nv = length(Sp);
keep_constraint = true(2*Nq+1, 1);
% sum_{i\in q} \gamma_i \geq P_lower_coarse, for all q \in Q
A = zeros(2*Nq+1, Nv);
b = zeros(2*Nq+1, 1);
for ci = 1:Nq
    sps = children_q{ci};
    if ( ~isempty( sps ) )
        for sp = sps'
            spi = find(Sp == sp);
            if ( ~isempty(spi) ) % Assign 0 bc that sp \notin Q_nd, which means it will have to share the eps_support, and this one will be assigned to unsafe
                A(2*ci-1, spi) = 1;
                A(2*ci, spi) = -1;
            else
                keep_constraint(2*ci-1) = false;
                keep_constraint(2*ci) = false;
                break;
            end
        end
    else
        keep_constraint(2*ci-1) = false;
        keep_constraint(2*ci) = false;
    end
    b(2*ci-1) = P_upper_coarse(ci);
    b(2*ci) = -P_lower_coarse(ci);
end

A(2*Nq+1, :) = -ones(size(Sp'));
% A = sparse( A );
b(2*Nq+1) = -1;

% Delete constraints corresponding to clusters with states not in Q_nd
A = A(keep_constraint, :);
b = b(keep_constraint, :);


Aeq = [];
beq = [];

end

