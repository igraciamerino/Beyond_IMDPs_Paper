function [ii, step] = disp_progress_fcn(N, ii, N_steps, step)
% Display progress
precision = 1e-6;
progress = floor(ii/N*100);
Delta_step = 100/N_steps;
if ( progress > Delta_step*step - precision )
    disp(progress)
    toc
    step = step + 1;
end
ii = ii + 1;
end