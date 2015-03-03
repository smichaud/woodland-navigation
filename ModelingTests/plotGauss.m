function [ output_args ] = plotGauss( mu, sigma )
x = (-5 * sigma:0.01:5 * sigma) + mu;  %// Plotting range
y = exp(- 0.5 * ((x - mu) / sigma) .^ 2) / (sigma * sqrt(2 * pi));
plot(x, y)

%// Hide ticks
set(gca, 'XTick', [], 'XTickLabel', [], 'YTick', [], 'YTickLabel', [])


end

