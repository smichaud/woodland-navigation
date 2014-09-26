for i=1:length(rawData)
    clf;
    close all;
    figure('Name', rawData(i).name, 'units','normalized',...
        'outerposition',[0 0 1 1])
    hold on;
    
    plot(rawData(i).rawMotorCurrents(:,1),...
        rawData(i).rawMotorCurrents(:,2)+rawData(i).rawMotorCurrents(:,3),...
        'b-');
    plot(rawData(i).rawRollPitchYaw(:,1),...
        repmat(17,length(rawData(i).rawRollPitchYaw),1) - ...
        rawData(i).rawRollPitchYaw(:,3)*80,...
        'r-');
    
    title('Empirical linear relation between pitch angle and motor currents');
    xlabel('Time')
    ylabel('(Motor Currents) and (17 - angle*80)')
    legend({'Motor currents', '17 - pitch*80'},'Location','EastOutside');
    
    figureFilename = [figuresDirectory rawData(i).name '.png'];
    export_fig(figureFilename); 
%     uiwait;
end
hold off;
close all;