disp(['Saving data to',dataDirectory,datasetName, '...']);

% Quick way, just save all
save(strcat(dataDirectory,datasetName));


% Selective way... I'll do it properly when I have some time
% save it in the data folder
% save(strcat(dataDirectory,datasetName), ...
%     'robotSpeed',...
%     'areaOfInterest', ...
%     'dataset');
% 
% if exist('regressor', 'var')
%     save(strcat(dataDirectory,regressorName), ...
%         'regressor');
% end