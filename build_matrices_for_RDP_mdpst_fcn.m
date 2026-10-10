function [A, b, Aeq, beq] = build_matrices_for_RDP_mdpst_fcn(P_c, Sp, children_q)

% New (I use \sum \gamma \ge 1)
Nq = length(children_q);

Nv = length(Sp);
keep_constraint = true(Nq+1, 1);
% sum_{i\in q} \gamma_i \geq P_lower_coarse, for all q \in Q
A = zeros(Nq+1, Nv);
b = zeros(Nq+1, 1);
for ci = 1:Nq
    sps = children_q{ci};
    if ( ~isempty( sps ) )
        for sp = sps'
            spi = find(Sp == sp);
            if ( ~isempty(spi) ) % Assign 0 bc that sp \notin Q_nd, which means it will have to share the eps_support, and this one will be assigned to unsafe
                A(ci, spi) = -1;
            else
                keep_constraint(ci) = false;
                break;
            end
        end
    else
        keep_constraint(ci) = false;
    end
    b(ci) = -P_c(ci);
end

A(Nq+1, :) = -ones(size(Sp'));
% A = sparse( A );
b(Nq+1) = -1;

% Delete constraints corresponding to clusters with states not in Q_nd
A = A(keep_constraint, :);
b = b(keep_constraint, :);


Aeq = [];
beq = [];

end

