for i=1:nbOfSamples
    clf;
    hold on;
    
    imshow(dataset(i).image);
    uiwait;
    
    clf
    plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,2), 'b');
    plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,3), 'r');
    uiwait;
end


