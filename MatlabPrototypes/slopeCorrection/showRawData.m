for i=1:length(rawData)
    clf;
    close all;
    figure('Name', rawData(i).name, 'units','normalized',...
        'outerposition',[0 0 1 1])
    hold on;
    
    plot(rawData(i).rollPitchYaw(:,1),...
        -rawData(i).rollPitchYaw(:,3),...
        'r-');
    plot(rawData(i).motorCurrents(:,1),...
        rawData(i).motorCurrents(:,2)+rawData(i).motorCurrents(:,3),...
        'b-');    
    
    title('Negative pitch angle and motor currents');
    xlabel('Time')
    ylabel('(-pitch) and (Motor Currents)')
    legend({'Pitch angle', 'Motor currents'},'Location','EastOutside');
    
    figureFilename = [figuresDirectory 'raw_' rawData(i).name '.pdf'];
    export_fig(figureFilename);
end
hold off;
close all;