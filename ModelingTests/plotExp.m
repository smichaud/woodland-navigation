function [ output_args ] = plotGauss( m, c , color)
x = 0:0.01:10;  %// Plotting range
y = exp(m*x + c); 
plot(x, y, color)

%// Hide ticks
%set(gca, 'XTick', [], 'XTickLabel', [], 'YTick', [], 'YTickLabel', [])


end

