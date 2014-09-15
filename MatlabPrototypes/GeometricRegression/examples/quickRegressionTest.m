% Quicktest to see if the regression works well:
% Example function with noise
mu = 0;
sigma = 100;
x1 = (0:0.005:1)';
x2 = (0:pi/200:pi)';
x3 = (0:0.005:1)';
dataset = [x1, x2, x3];
labels = x1.^2 - cos(x2) + 2.*sqrt(x3);% + normrnd(mu,sigma, size(x1,2), 1);

% datasetOutlier = [10 5 55];
% labelOutlier = -43;
% dataset = [dataset ; datasetOutlier];
% labels = [labels ; labelOutlier];

% plot(dataset(:,1),labels, '.');
% title('Data overview')
% uiwait;

nbOfTrees = 500;
nbOfLeaves = 5;
regressor = TreeBagger(...
    nbOfTrees,...
    dataset, labels,...
    'Method', 'R',...
    'OOBPred', 'On',...
    'OOBVarImp','On',...
    'MinLeaf', nbOfLeaves);

plot(oobError(regressor));
xlabel('Number of Grown Trees');
ylabel('Mean Squared Error');

for i=1:10
    v1 = rand + normrnd(0,0.05,1,1);
    v2 = (rand + normrnd(0,0.05,1,1))*pi;
    v3 = rand + normrnd(0,0.05,1,1);
    testData = [v1 v2 v3];
    realLabel = testData(1).^2 - cos(testData(2)) + 2.*sqrt(testData(3));
    testPrediction = regressor.predict(testData);
    diff = testPrediction - realLabel;
    perc = diff/realLabel;
    disp('==========')
    disp(sprintf('label: %f \nprediction: %f \ndiff: %f \nperc: %f',...
        realLabel, testPrediction, diff, perc));
end