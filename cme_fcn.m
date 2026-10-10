function [Phi, beta,  Phi_z, beta_z, Phi_norm, k_YY, k_YY_reg_inv] = cme_fcn(Xtrain, Ytrain, z, sigmaf, sigmal, lambda, k_YY, k_YY_reg_inv, Na)

% y (or z) is the conditioning variable

[n, N] = size(Xtrain{1});


if (isempty(k_YY) || isempty(k_YY_reg_inv) )
    k_YY = kernel_gauss_fcn(Ytrain, Ytrain, sigmal, sigmaf);
    k_YY_reg = k_YY + N*lambda*eye(N);
    k_YY_reg_inv = inv(k_YY_reg);
end

k_Yz = kernel_gauss_fcn(Ytrain, z, sigmal, sigmaf);
k_Y = @(y) kernel_gauss_fcn(Ytrain, y, sigmal, sigmaf);
beta_z = k_YY_reg_inv*k_Yz;
beta = @(y) k_YY_reg_inv*k_Y(y);

for a = 1:Na

    k_X{a} = @(x) kernel_gauss_fcn(Xtrain{a}, x, sigmal, sigmaf);
    Phi_z{a} = @(x) beta_z{a}'*k_X{a}(x);
    Phi{a} = @(y,x) beta{a}(y)'*k_X{a}(x);

    k_XX{a} = kernel_gauss_fcn(Xtrain{a}, Xtrain{a}, sigmal, sigmaf);
    betai_betaj = (k_YY_reg_inv*k_YY)*k_YY_reg_inv;
    Phi_norm{a} = sqrt(betai_betaj(:)'*k_XX{a}(:));

end

end