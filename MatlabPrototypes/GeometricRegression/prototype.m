dftVectors

linkageResult = linkage(dftVectors, 'single', 'euclidean');

dendrogram(linkageResult);