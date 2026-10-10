function supgradObj = compute_supgrad(epsilon,P_lower_nom,P_upper_nom,dist_sq_l,J1_ind,J2_ind)

supgradObj = -epsilon^2 + P_lower_nom(J1_ind)*dist_sq_l(J1_ind) + P_upper_nom(J2_ind)*dist_sq_l(J2_ind);

end