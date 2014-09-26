mkdir('figures');

for i=1:length(rawData)
    clf;
    close all;
    figure('Name', rawData(i).name, 'units','normalized',...
        'outerposition',[0 0 1 1])
    hold on;
    
    plot(rawData(i).motorCurrents(:,1),...
        rawData(i).motorCurrents(:,2)+rawData(i).motorCurrents(:,3),...
        'b-');
    plot(rawData(i).rollPitchYaw(:,1),...
        repmat(17,length(rawData(i).rollPitchYaw),1) - ...
        rawData(i).rollPitchYaw(:,3)*80,...
        'r-');
    
    title('Empirical linear relation between pitch angle and motor currents');
    xlabel('Time')
    ylabel('(Motor Currents) and (17 - angle*80)')
    legend({'Motor currents', '17 - pitch*80'},'Location','EastOutside');
    
    figureFilename = [figuresDirectory 'raw_' rawData(i).name '.png'];
    export_fig(figureFilename); 
%     uiwait;
end
hold off;
close all;