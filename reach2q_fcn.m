function [qpi_out, error_flag] = reach2q_fcn(Sp_intersects, Qp, Ns)
% Compute bounds for transitions starting from q, under a

error_flag = false;

% If Sp_intersects contains Ns + 1, output the last cluster
if ( ismember( Ns + 1, Sp_intersects ) )
    qpi_out = length( Qp );
    return;
end

qpi_out = [];
for qpi = 1:length( Qp )
    qp = Qp{qpi};
    if ( isempty( qp ) ) % qp still not defined, so exit
        error_flag = true;
        return;
    end
    if ( length( Sp_intersects ) == length( qp ) )
        for sp = Sp_intersects
            sp_in_qp_yn = false;
            for child = qp
                if ( sp == child )
                    sp_in_qp_yn = true;
                    break; % bc sp \in qp
                end
            end
            if ( ~sp_in_qp_yn )
                break;
            end
        end
        if ( sp_in_qp_yn )
            % I've checked all sp \in Sp_intersects and they're all in qp
            qpi_out = qpi;
            return;
        %else % Continue looping on qp
        end
    end
end
error_flag = true;
end