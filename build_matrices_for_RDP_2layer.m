function [A, b, Aeq, beq] = build_matrices_for_RDP_2layer(P_lower_coarse, P_upper_coarse, Sp, Qp, children, Nq)

% New (I use \sum \gamma \ge 1)
% First remove cluster corresponding to the unsafe state
if ( Qp(end) > Nq + 0.5 ) % Same as saying Q_nd_coarse(end) == Nq_coarse + 1
    Ncoarse = length(Qp) - 1;
else
    Ncoarse = length(Qp);
end

Nv = length(Sp);
keep_constraint = true(2*Ncoarse+1, 1);
% sum_{i\in q} \gamma_i \geq P_lower_coarse, for all q \in Q
A = zeros(2*Ncoarse + 1, Nv);
b = zeros(2*Ncoarse + 1, 1);
for j = 1:Ncoarse
    qp = Qp(j);
    sps = children{qp};
    for sp = sps'
        spi = find(Sp == sp);
        if ( ~isempty(spi) ) % Assign 0 bc that sp \notin Q_nd, which means it will have to share the eps_support, and this one will be assigned to unsafe
            A(2*j-1, spi) = 1;
            A(2*j, spi) = -1;
        else
            keep_constraint(2*j-1) = false;
            keep_constraint(2*j) = false;
            break;
        end
    end
    b(2*j-1) = P_upper_coarse(j);
    b(2*j) = -P_lower_coarse(j);
end

A(end, :) = -ones(size(Sp'));
% A = sparse( A );
b(end) = -1;

% Delete constraints corresponding to clusters with states not in Q_nd
A = A(keep_constraint, :);
b = b(keep_constraint, :);


Aeq = [];
beq = [];

end

