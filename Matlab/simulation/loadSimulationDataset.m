function dataset = loadSimulationDataset(datasetDir)
countPattern = 'count_([0-9]*)';
radiusPattern = '_radius_mm_([0-9]*)';
samplePattern = '_sample_([0-9]*)';
endPattern = '(.*\.csv)';
pattern = [countPattern, radiusPattern, samplePattern, endPattern];

files = dir([datasetDir '*.csv']);
filesCount = length(files);

dataStruct = struct(...
    'pointcloud', [],...
    'cylindersCount', 0,...
    'cylindersRadius', 0,...
    'isNoisy', false);
dataset = repmat(dataStruct, filesCount, 1);

startingRow = 1; % ignore header (zero based)
startingCol = 0; % take all (zero based)
for i = 1:filesCount
    [tokens, matches] = regexp(files(i).name, pattern, 'tokens', 'match');
    dataset(i).cylindersCount = str2num(tokens{1}{1});
    dataset(i).cylindersRadius = str2num(tokens{1}{2});
    dataset(i).isNoisy = (strcmp(tokens{1}{4}, '_noisy.csv'));
    dataset(i).pointcloud = csvread([datasetDir, files(i).name],...
        startingRow, startingCol);
    dataset(i).pointcloud = dataset(i).pointcloud(:,1:3);
    
end

end

