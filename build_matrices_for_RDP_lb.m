function [A, b, Aeq, beq] = build_matrices_for_RDP_lb(P_lower_coarse,Q_nd, Q_nd_coarse, children, Nq_coarse)

% Old (I don't use \sum \gamma \ge 1)

% Nv = length(Q_nd);
% Ncoarse = length(Q_nd_coarse);
% Nconstraints = Ncoarse;
% 
% % sum_{i\in q} \gamma_i \geq P_lower_coarse, for all q \in Q
% A = zeros(Ncoarse, Nv);
% b = zeros(Ncoarse, 1);
% for j = 1:Ncoarse
%     qp = Q_nd_coarse(j);
%     sps = children{qp};
%     for sp = sps'
%         spi = find(Q_nd == sp);
%         if ( ~isempty(spi) ) % Assign 0 bc that sp \notin Q_nd, which means it will have to share the eps_support, and this one will be assigned to unsafe
%             A(j, spi) = -1;
%         end
%     end
%     b(j) = -P_lower_coarse(j);
% end
% 
% Aeq = ones(size(Q_nd'));
% beq = 1;

% New (I use \sum \gamma \ge 1)
% First remove cluster corresponding to the unsafe state
if ( Q_nd_coarse(end) > Nq_coarse + 0.5 ) % Same as saying Q_nd_coarse(end) == Nq_coarse + 1
    Ncoarse = length(Q_nd_coarse) - 1;
else
    Ncoarse = length(Q_nd_coarse);
end

Nv = length(Q_nd);
keep_constraint = true(Ncoarse+1, 1);
% sum_{i\in q} \gamma_i \geq P_lower_coarse, for all q \in Q
A = zeros(Ncoarse + 1, Nv);
b = zeros(Ncoarse + 1, 1);
for j = 1:Ncoarse
    qp = Q_nd_coarse(j);
    sps = children{qp};
    for sp = sps'
        spi = find(Q_nd == sp);
        if ( ~isempty(spi) ) % Assign 0 bc that sp \notin Q_nd, which means it will have to share the eps_support, and this one will be assigned to unsafe
            A(j, spi) = -1;
        else
            keep_constraint(j) = false;
            break;
        end
    end
end
b(1:Ncoarse) = -P_lower_coarse(1:Ncoarse);

A(end, :) = -ones(size(Q_nd'));
% A = sparse( A );
b(end) = -1;

% Delete constraints corresponding to clusters with states not in Q_nd
A = A(keep_constraint, :);
b = b(keep_constraint, :);


Aeq = [];
beq = [];

end

