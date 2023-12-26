function doublePendulumAnimation
    % Parameters
    g = 9.81;   
    l1 = 1;     
    l2 = 1;     
    m1 = 1;     
    m2 = 0.1;     

    % Initial conditions
    theta1_0 = pi/4;   
    omega1_0 = 0;       
    theta2_0 = pi/2;    
    omega2_0 = 0; 

    % Time parameters
    tspan = [0 10];     
    dt = 0.001;       
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    [t, y] = ode45(@doublePendulumODE, tspan, [theta1_0; omega1_0; theta2_0; omega2_0], options, g, l1, l2, m1, m2);

    %animation_generation 
    figure;
    for i = 1:length(t)
       
        theta1 = y(i, 1);
        theta2 = y(i, 3);

        
        x1 = l1 * sin(theta1);
        y1 = -l1 * cos(theta1);
        x2 = x1 + l2 * sin(theta2);
        y2 = y1 - l2 * cos(theta2);

       
        plot([0, x1, x2], [0, y1, y2], 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        axis equal;
        axis([-2 2 -2 2]); 

        
        title(['Double Pendulum Animation - Time: ' num2str(t(i), '%.2f') ' s']);
        pause(dt);

        
        clf;
    end
end

function dydt = doublePendulumODE(t, y, g, l1, l2, m1, m2)
    theta1 = y(1);
    omega1 = y(2);
    theta2 = y(3);
    omega2 = y(4);

    dydt = zeros(4, 1);
    dydt(1) = omega1;
    dydt(2) = (-g * (2 * m1 + m2) * sin(theta1) - m2 * g * sin(theta1 - 2 * theta2) - 2 * sin(theta1 - theta2) * m2 * (omega2^2 * l2 + omega1^2 * l1 * cos(theta1 - theta2))) / (l1 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2)));
    dydt(3) = omega2;
    dydt(4) = (2 * sin(theta1 - theta2) * (omega1^2 * l1 * (m1 + m2) + g * (m1 + m2) * cos(theta1) + omega2^2 * l2 * m2 * cos(theta1 - theta2))) / (l2 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2)));
end
