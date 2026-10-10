function [qpi_out, qpi_1st_empty] = reach2q_fcn_old(Sp_intersects, Qp)
% Compute bounds for transitions starting from q, under a

qpi_out = [];
qpi_1st_empty = [];
for qpi = 1:length( Qp )
    qp = Qp{qpi};
    if ( isempty( qp ) ) % qp still not defined, so exit
        qpi_1st_empty = qpi;
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
qpi_1st_empty = qpi + 1;

end