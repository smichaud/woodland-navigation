% Quicktest to see if the regression works well:
% Example function with noise
clear all;
clc;

mu = 0;
sigma = 100;

x1 = (0:1/400:1);
x1 = x1(randperm(length(x1)))';
x2 = (0:pi/400:pi);
x2 = x2(randperm(length(x2)))';
x3 = (0:1/400:1);
x3 = x3(randperm(length(x3)))';

dataset = [x1, x2, x3];
labels = x1.^2 - cos(x2) + 2.*sqrt(x3);% + normrnd(mu,sigma, size(x1,2), 1);

% datasetOutlier = [10 5 55];
% labelOutlier = -43;
% dataset = [dataset ; datasetOutlier];
% labels = [labels ; labelOutlier];

% plot(dataset(:,1),labels, '.');
% title('Data overview')
% uiwait;

nbOfTrees = 1000;
nbOfLeaves = 5;
regressor = TreeBagger(...
    nbOfTrees,...
    dataset, labels,...
    'Method', 'R',...
    'OOBPred', 'On',...
    'OOBVarImp','On',...
    'MinLeaf', nbOfLeaves);

% plot(oobError(regressor));
% xlabel('Number of Grown Trees');
% ylabel('Mean Squared Error');

nbOfTests = 50;
for i=1:nbOfTests
    v1 = rand + normrnd(0,0.05,1,1);
    v2 = (rand + normrnd(0,0.05,1,1))*pi;
    v3 = rand + normrnd(0,0.05,1,1);
    testData = [v1 v2 v3];
    realLabel(i) = testData(1).^2 - cos(testData(2)) + 2.*sqrt(testData(3));
    testPrediction(i) = regressor.predict(testData);
%     diff = testPrediction - realLabel;
%     perc = diff/realLabel;
%     disp('==========')
%     disp(sprintf('label: %f \nprediction: %f \ndiff: %f \nperc: %f',...
%         realLabel, testPrediction, diff, perc));
end
hold on;
plot(1:nbOfTests, realLabel, '*r');
plot(1:nbOfTests, testPrediction, '*b');