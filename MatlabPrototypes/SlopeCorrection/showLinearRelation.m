% Show the line and the prediction vs time
hold on;

plot(pitchMotorCurrentsSamples(:,1), pitchMotorCurrentsSamples(:,2), '.b');
plot(pitchMotorCurrentsSamples(:,1),...
    pitchMotorCurrentsSamples(:,1)*pitchMotorCurrentsRelation(2)...
    + pitchMotorCurrentsRelation(1),...
    'k-');
title('Motor currents by pitch with line fit')
xlabel('Pitch angle of the robot')
ylabel('Sum of the left and right motor currents');
legend({'Measured samples','Line fit'});

hold off;

figureFilename = [figuresDirectory 'linearRelation.pdf'];
export_fig(figureFilename);

uiwait;