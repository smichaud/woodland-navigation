% Create a distance matrix using the DFT vector (xInertia)
dftVectors = [0 0; 1 0; 0 1]; 
D = squareform(pdist(x, 'euclidean')); 
% http://www.mathworks.com/help/stats/pdist.html
% Check distances

nbOfSamples = length(dataset)
for i = 1:nbOfSamples
end
