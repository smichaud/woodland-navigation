% num = randi(10, 50, 1)
% den = randi(100, 50, 1)
num = [repmat(12, 1, 50) repmat(2, 1, 50)];
den = [repmat(10, 1, 50) repmat(3, 1, 50)];

disp('sum^2/sum^2');
disp(sqrt(sum(num.^2)/sum(den.^2)))
disp('mean of fractions^2');
disp(sqrt(mean((num.^2)./(den.^2))))