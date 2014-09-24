nbOfSamples = length(dataset);
for i=1:nbOfSamples
    nbOfPoints = size(dataset(i).areaOfInterest,1);
    
    values = zeros(3);
    if nbOfPoints > 0
        [vectors values] = eig(cov(dataset(i).areaOfInterest));
    end
    dataset(i).features('eigen') = ...
        [vectors(:,1)' vectors(:,2)' vectors(:,3)'];
    %         diag(values)';    
    %         [diag(values)' vectors(:,1)' vectors(:,2)' vectors(:,3)'];
end