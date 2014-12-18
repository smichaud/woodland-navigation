% Show the line and the prediction vs time
hold on;

plot(1:length(pitchMotorCurrentsSamples),...
    pitchMotorCurrentsSamples(:,1)*pitchMotorCurrentsRelation(2) + ...
    pitchMotorCurrentsRelation(1), 'r.');
plot(1:length(pitchMotorCurrentsSamples),...
    pitchMotorCurrentsSamples(:,2), 'b.');

title('Motor currents by pitch with line fit')
xlabel('Pitch angle of the robot')
ylabel('Sum of the left and right motor currents');
legend({'Measured samples','Line fit'});

hold off;

figureFilename = [figuresDirectory 'predictions.pdf'];
export_fig(figureFilename);

uiwait;