nbOfSamples = length(dataset);

userTraversabilityCost = zeros(nbOfSamples, 1);
for i=1:nbOfSamples    
    userTraversabilityCost(i) = dataset(i).userTraversabilityCost;
end 
[vector orderedIndexes] = sort(userTraversabilityCost);


hLeft = figure('Name', 'Raw data low', 'units','normalized',...
        'outerposition',[0 0 1 1]);
hRight = figure('Name', 'Raw data high', 'units','normalized',...
        'outerposition',[0 1 2 1]);
    
nbOfChanges = 0;
for i=1:nbOfSamples-1 
    set(0, 'CurrentFigure', hLeft);
    showSample(dataset(orderedIndexes(i)),...
        areaOfInterest, false);
    
    set(0, 'CurrentFigure', hRight);
    showSample(dataset(orderedIndexes(i+1)),...
        areaOfInterest, false);

    userChoice = getkey;
    if userChoice == 113 % q (quit)
        break;
    elseif userChoice == 97 % a (left higher)
        swap('dataset(orderedIndexes(i)).userTraversabilityCost',...
            'dataset(orderedIndexes(i+1)).userTraversabilityCost');
        swap('dataset(orderedIndexes(i))','dataset(orderedIndexes(i+1))');
        nbOfChanges = nbOfChanges + 1;
%     elseif userChoice == 100 % d (righ higher)
%         disp('right');
%     else % (s(115): both equal)
%         disp('equal');
    end
end

% If nbOfChanges is zero, then it is properly ordered
nbOfChanges

% nbOfChanges = 0;
% for i=nbOfSamples-1:-1:1 
%     set(0, 'CurrentFigure', hLeft);
%     showSample(dataset(orderedIndexes(i)),...
%         areaOfInterest, false);
%     
%     set(0, 'CurrentFigure', hRight);
%     showSample(dataset(orderedIndexes(i+1)),...
%         areaOfInterest, false);
% 
%     userChoice = getkey;
%     if userChoice == 113 % q (quit)
%         break;
%     elseif userChoice == 97 % a (left higher)
%         swap('dataset(orderedIndexes(i)).userTraversabilityCost',...
%             'dataset(orderedIndexes(i+1)).userTraversabilityCost');
%         swap('dataset(orderedIndexes(i))','dataset(orderedIndexes(i+1))');
%         nbOfChanges = nbOfChanges + 1;
% %     elseif userChoice == 100 % d (righ higher)
% %         disp('right');
% %     else % (s(115): both equal)
% %         disp('equal');
%     end
% end
% 
% % If nbOfChanges is zero, then it is properly ordered
% nbOfChanges