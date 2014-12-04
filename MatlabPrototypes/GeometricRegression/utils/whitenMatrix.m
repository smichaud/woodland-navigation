function [X] = whitenMatrix(X, fudgefactor)
X = bsxfun(@minus, X, mean(X));
A = X'*X;
[V,D] = eig(A);

if ~exist('fudgefactor')
    fudgefactor = max(diag(D))*1e-6;
end

X = X*V*diag(1./(diag(D)+fudgefactor).^(1/2))*V';
end

