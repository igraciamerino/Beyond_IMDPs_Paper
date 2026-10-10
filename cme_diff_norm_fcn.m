function phi_diff_norm = cme_diff_norm_fcn(Xtraini, Xtrainj, Ytraini, Ytrainj, sigmaf, sigmal, lambda)

[~, Ni] = size(Xtraini);
[~, Nj] = size(Xtrainj);

K_XiXi = kernel_gauss_fcn(Xtraini, Xtraini, sigmal, sigmaf);
K_XjXj = kernel_gauss_fcn(Xtrainj, Xtrainj, sigmal, sigmaf);
K_XiXj = kernel_gauss_fcn(Xtraini, Xtrainj, sigmal, sigmaf);

K_YiYi = kernel_gauss_fcn(Ytraini, Ytraini, sigmal, sigmaf);
K_YjYj = kernel_gauss_fcn(Ytrainj, Ytrainj, sigmal, sigmaf);
K_YiYj = kernel_gauss_fcn(Ytraini, Ytrainj, sigmal, sigmaf);

Wi_inv = K_YiYi + Ni*lambda*eye(Ni);
Wj_inv = K_YjYj + Nj*lambda*eye(Nj);

termi = (Wi_inv\K_YiYi)/Wi_inv;
termj = (Wj_inv\K_YjYj)/Wj_inv;
termij = (Wi_inv\K_YiYj)/(Wj_inv)';

phi_diff_norm = sqrt( termi(:)'*K_XiXi(:) -2*termij(:)'*K_XiXj(:) + termj(:)'*K_XjXj(:) );

end