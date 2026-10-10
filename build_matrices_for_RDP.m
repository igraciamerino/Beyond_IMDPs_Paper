function [A, b, Aeq, beq] = build_matrices_for_RDP(P_lower_coarse,P_upper_coarse,Q_nd, Q_nd_coarse, children)

Nv = length(Q_nd);
Ncoarse = length(Q_nd_coarse);
Nconstraints = 2*Ncoarse;

% sum_{i\in q} \gamma_i \leq P_upper_coarse, for all q \in Q
A = zeros(2*Ncoarse, Nv);
b = zeros(2*Ncoarse, 1);
for j = 1:Ncoarse
    qp = Q_nd_coarse(j);
    sps = children{qp};
    for sp = sps'
        spi = find(Q_nd == sp);
        if ( ~isempty(spi) ) % Assign 0 bc that sp \notin Q_nd, which means it will have to share the eps_support, and this one will be assigned to unsafe
            A(2*j-1, spi) = 1;
            A(2*j, spi) = -1;
        end
    end
    b(2*j-1) = P_upper_coarse(j);
    b(2*j) = -P_lower_coarse(j);
end

Aeq = ones(size(Q_nd'));
beq = 1;


end

