function result = is_point_on_triangle_fcn(v1, v2, v3, l1, l2, p)
% Input: vertices v1, v2, v3 of triangle, l1 = v2-v1, l2 = v3 - v1, point p

pv1 = v1 - p;
pv2 = v2 - p;
pv3 = v3 - p;
S1 = sqrt( (pv2(2)*pv3(3) - pv2(3)*pv3(2))^2 + (pv2(1)*pv3(3) - pv2(3)*pv3(1))^2  + (pv2(1)*pv3(2) - pv2(2)*pv3(1))^2 );
S2 = sqrt( (pv1(2)*pv3(3) - pv1(3)*pv3(2))^2 + (pv1(1)*pv3(3) - pv1(3)*pv3(1))^2  + (pv1(1)*pv3(2) - pv1(2)*pv3(1))^2 );
S3 = sqrt( (pv1(2)*pv2(3) - pv1(3)*pv2(2))^2 + (pv1(1)*pv2(3) - pv1(3)*pv2(1))^2  + (pv1(1)*pv2(2) - pv1(2)*pv2(1))^2 );

S = sqrt( (l1(2)*l2(3) - l1(3)*l2(2))^2 + (l1(1)*l2(3) - l1(3)*l2(1))^2  + (l1(1)*l2(2) - l1(2)*l2(1))^2 );

if ( S1 + S2 + S3 < S + 1e-10) % Precision
    result = true;
else
    result = false;
end

end