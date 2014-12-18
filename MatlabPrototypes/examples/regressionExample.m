load imports-85;
Y = X(:,1);
X = X(:,2:end);
isCat = [zeros(15,1);ones(size(X,2)-15,1)]; % Categorical variable flag

rng(1945,'twister')

% Finding the Optimal Leaf Size
leaf = [5 10 20 50 100];
col = 'rbcmy';
figure
for i=1:length(leaf)
    b = TreeBagger(50,X,Y,'Method','R','OOBPred','On',...
			'Cat',find(isCat == 1),'MinLeaf',leaf(i));
    plot(oobError(b),col(i));
    hold on;
end
xlabel('Number of Grown Trees');
ylabel('Mean Squared Error');
legend({'5' '10' '20' '50' '100'},'Location','NorthEast');
hold off;

% Estimating Feature Importance
b = TreeBagger(100,X,Y,'Method','R','OOBVarImp','On',...
    'Cat',find(isCat == 1),'MinLeaf',5);

figure
plot(oobError(b));
xlabel('Number of Grown Trees');
ylabel('Out-of-Bag Mean Squared Error');

figure
bar(b.OOBPermutedVarDeltaError);
xlabel('Feature Number');
ylabel('Out-of-Bag Feature Importance');
idxvar = find(b.OOBPermutedVarDeltaError>0.7)
idxCat = find(isCat(idxvar)==1);

finbag = zeros(1,b.NTrees);
for t=1:b.NTrees
    finbag(t) = sum(all(~b.OOBIndices(:,1:t),2));
end
finbag = finbag / size(X,1);
figure
plot(finbag);
xlabel('Number of Grown Trees');
ylabel('Fraction of In-Bag Observations');

% Growing Trees on a Reduced Set of Features (5 best)
b5v = TreeBagger(100,X(:,idxvar),Y,'Method','R',...
'OOBVarImp','On','Cat',idxCat,'MinLeaf',5);
figure
plot(oobError(b5v));
xlabel('Number of Grown Trees');
ylabel('Out-of-Bag Mean Squared Error');
figure
bar(b5v.OOBPermutedVarDeltaError);
xlabel('Feature Index');
ylabel('Out-of-Bag Feature Importance');

% Finding Outliers
b5v = fillProximities(b5v);

figure
hist(b5v.OutlierMeasure);
xlabel('Outlier Measure');
ylabel('Number of Observations');
