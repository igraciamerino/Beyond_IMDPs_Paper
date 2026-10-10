function [Q, q, rhs, Aeq, beq] = build_matrices_for_RVI_cme_umdp_old_fcn(beta, x_grid, Sp, Ns, sigmaf, sigmal, epsilon)

k = @(diff) sigmaf^2*exp(-(diff(:).*diff(:))/sigmal^2);

Q_1 = zeros(Ns+1);
Q_2 = zeros(Ns+1, length(Sp));
Q_3 = zeros(length(Sp));
for s1 = 1:Ns
    for s2 = 1:Ns
        s_a = x_grid(:, s1);
        s_b = x_grid(:, s1+1);
        s_c = x_grid(:, s2);
        s_d = x_grid(:, s2+1);
        diff_min = max(abs(s_a - s_d), abs(s_b - s_c));
        Q_1(s1,s2) = k(diff_min);
    end
end

for s1 = 1:Ns
    s2i = 0;
    for s2 = Sp
        if (s2 ~= Ns+1)
            s2i = s2i + 1;
            s_a = x_grid(:, s1);
            s_b = x_grid(:, s1+1);
            s_c = x_grid(:, s2);
            s_d = x_grid(:, s2+1);
            if ( beta(s2i) < 0 )
                diff_min = max(abs(s_a - s_d), abs(s_b - s_c));
                Q_2(s1,s2i) = k(diff_min);
            else
                if max(s_a, s_c) <= min(s_b, s_d)
                    diff_max = 0;  % intervals overlap → min |x - y| = 0
                else
                    % intervals disjoint → compute minimum distance between endpoints
                    diff_max = min(abs(s_b - s_c), abs(s_a - s_d));
                end
                Q_2(s1,s2i) = k(diff_max);
            end
        end
    end
end

s1i = 0;
for s1 = Sp
    s1i = s1i + 1;
    if (s1 ~= Ns+1)
        s2i = 0;
        for s2 = Sp
            if (s2 ~= Ns+1)
                s2i = s2i + 1;
                s_a = x_grid(:, s1);
                s_b = x_grid(:, s1+1);
                s_c = x_grid(:, s2);
                s_d = x_grid(:, s2+1);
                diff_min = max(abs(s_a - s_d), abs(s_b - s_c));
                Q_3(s1i,s2i) = k(diff_min);
            else
                break;
            end
        end
    else
        break;
    end
end

if( ~isPD_fcn(Q_1) )
    disp('ERROR: Matrix is not PD...')
end

Q = Q_1;                           % Quad term in quad constraint
q = -2*Q_2*beta;                    % Linear term in quadratic constraint
% q = q';
r = beta'*Q_3*beta;                           % Constant offset in quadratic constraint
rhs = epsilon^2 - r;                         % Right-hand side of constraint

% Linear constraints
% sum(x) = 1
Aeq = ones(1, Ns + 1);
beq = 1;

end

