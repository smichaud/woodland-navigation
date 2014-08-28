data = csvread('./motor_currents.csv');

hold on;
plot(data(:,1), data(:,2), 'b');
plot(data(:,1), data(:,3), 'r');

