pitchMotorCurrentsRelation = robustfit(...
    pitchMotorCurrentsSamples(:,1),...
    pitchMotorCurrentsSamples(:,2),...
    'bisquare', 4.685);


