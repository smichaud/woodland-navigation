prediction = pitchMotorCurrentsSamples(:,1)*...
    pitchMotorCurrentsRelation(2) + ...
    pitchMotorCurrentsRelation(1);
measured = pitchMotorCurrentsSamples(:,2);
MSE = mean((prediction - measured).^2);

disp(['MSE: ' num2str(MSE)]);