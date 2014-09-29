directory = '/home/smichaud/Desktop/slopeCorrection/ExtractedData/';
fileBase = 'lookDown';
imageFile = [directory fileBase '_image.jpg'];
pointCloudFile = [directory fileBase '_point_cloud.csv'];
rollPitchYawfile = [directory fileBase '_roll_pitch_yaw.csv'];

pointCloud = csvread(pointCloudFile,1,0);
pointCloud = pointCloud(:,1:3);
rollPitchYaw = csvread(rollPitchYawfile);

% imshow(imageFile);

hold on;
plot(1:length(rollPitchYaw), rollPitchYaw(:,2), 'r'); % roll
plot(1:length(rollPitchYaw), rollPitchYaw(:,3), 'b'); % pitch
xlabel('time')
ylabel('angle')
legend({'roll','pitch'});
hold off;

meanOverTime = 5; 
timeInterval = rollPitchYaw(2,1) - rollPitchYaw(1,1);
lastMeanIndex = round(meanOverTime/timeInterval);

roll = mean(rollPitchYaw(1:lastMeanIndex,2));
pitch = mean(rollPitchYaw(1:lastMeanIndex,3));

pitchDegree = rad2deg(pitch)
rollDegree = rad2deg(roll)

rotatedPointCloud = (roty(rad2deg(pitch))*rotx(rad2deg(roll))*pointCloud')';

csvwrite('/home/smichaud/Desktop/original.csv', pointCloud);
csvwrite('/home/smichaud/Desktop/corrected.csv', rotatedPointCloud);