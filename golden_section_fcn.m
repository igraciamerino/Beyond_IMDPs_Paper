function [xmax, fmax] = golden_section_fcn(a, b, obj, tol, max_iter)
% Golden section line search
    resphi =  (sqrt(5) - 1) / 2;
    c = b - resphi * (b - a);
    d = a + resphi * (b - a);
    fc = obj(c);
    fd = obj(d);
    for iter = 1:max_iter
        if abs(b - a) < tol
            break;
        end
        if fc > fd
            b = d;
            d = c;
            fd = fc;
            c = b - resphi * (b - a);
            fc = obj(c);
        else
            a = c;
            c = d;
            fc = fd;
            d = a + resphi * (b - a);
            fd = obj(d);
        end
    end

    %Return midpoint of final interval as best estimate
    xmax = (a + b) / 2;
    fmax = obj(xmax);

    if ( iter == max_iter )
        disp('WARNING: MAX ITER REACHED...')
    end

end