% To plot the 8 figure path

% Initialise
t = linspace(0 , 2 * pi , 100);
A = 0.5 / sqrt(2); % Value chosen such that x is limited to -0.5m and 0.5m

% Equations given
X_Function = (A * sqrt(2) * cos(t) ) ./ ( (sin(t).^2) + 1);
Y_Function = (A * sqrt(2) * (cos(t) .* sin(t) ) ) ./ ( (sin(t).^2) + 1);

% Plotting the graph
figure(1);
hold on
grid on
title('8 figure path')
xlabel('x(t)')
ylabel('t')
plot( X_Function , Y_Function , 'k')