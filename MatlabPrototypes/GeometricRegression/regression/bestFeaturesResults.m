% ========== Check out result using only important features
% nbOfTrees = 100;
% nbOfLeaves = 5;
% regressorBestFeatures = TreeBagger(nbOfTrees,...
%     regressorFeatures(:,idxvar), regressorLabels,...
%     'Method','R',...
%     'OOBVarImp','On',...
%     'MinLeaf',nbOfLeaves);
% figure
% plot(oobError(regressorBestFeatures));
% title('Error Curve Using Only the Best Features');
% xlabel('Number of Grown Trees');
% ylabel('Out-of-Bag Mean Squared Error');
% uiwait;
% 
% figure
% bar(regressorBestFeatures.OOBPermutedVarDeltaError);
% title('Feature Importance Using Only the Best Features');
% xlabel('Feature Index');
% ylabel('Out-of-Bag Feature Importance');
% uiwait;
