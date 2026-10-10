function [x_MC, MC_flag] = MC_run_unicycle2D_nl_friction(x0,state_discretization,user_param,sigma_PROD,original_sys,Xi_hat_new, Act, variables_for_matrices)
% Simulate system via Monte Carlo

MC_timeout = user_param.MC_timeout;

x_MC = x0;
x = x0;
s = J_function(x,state_discretization,user_param);
z = variables_for_matrices.z0_PROD(s);
z_MC = z;
if ( z == 2 ) % If state q \in goal
    MC_flag = 'reach';
    return;
elseif ( z == 4 )
    MC_flag = 'no reach'; % If state q \in sink
    return;
end
for t = 1:MC_timeout
    a = sigma_PROD(s, z);
    u = Act(a);
    w = datasample(Xi_hat_new',1)';
    xp = unicycle2D_ltlf(x, u, w, original_sys, user_param);
    sp = J_function(xp, state_discretization, user_param);
    op = variables_for_matrices.labelDFA(sp);
    zp = variables_for_matrices.transDFA(z,op);
    x_MC = [x_MC xp];
    z_MC = [z_MC zp];
    if ( zp == 2 ) % If state q \in goal
        MC_flag = 'reach';
        break;
    elseif ( zp == 4 )
        MC_flag = 'no reach'; % If state q \in sink
        break;
    end
    x = xp;
    z = zp;
    s = sp;
end

if (t == MC_timeout)
    MC_flag = 'timeout';
    disp('MC timeout...')
end

end

