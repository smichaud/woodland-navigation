standbyCurrent = 0.5900; % normally
startPeakCurrent = 6.0; % both should be over that value

robotSpeed = 0.3; % m/s
distanceBeforeObstacles = 2; % meters
delayBeforeTraversability = distanceBeforeObstacles/robotSpeed;
traversabilityStartTime =  

for i=1:nbOfSamples
%     currents = dataset(i).rawCurrents;
%     firstMoveIndex = find(currents(1,:) > startPeakCurrent && currents(1,:) > startPeakCurrent);
%     
%     clf;
%     hold on;
%     plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,2), 'b');
%     plot(dataset(i).rawCurrents(:,1), dataset(i).rawCurrents(:,3), 'r');
%     line([firstMoveIndex,firstMoveIndex],ylim, 'y');
%     uiwait;
end

