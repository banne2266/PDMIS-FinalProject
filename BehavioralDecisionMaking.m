function S = BehavioralDecisionMaking(mess, velocity_list, position_list, t_duration, n_cars_vel, n_cars_pos, yaw_list)

cur_time = height(position_list);

t = cur_time - t_duration;
% get initial velocity
velocity = velocity_list(t);
% get initial position
position = position_list(t);

% get r_li or
r_li = 3.5/2;
[K, G_i, F_di, F_li] = energy(mess, velocity, r_li);


S = 0;
while t < cur_time
    t = t+1;
    % get next velocity
    velocity = velocity_list(t, :);
    % get next position
    position = position_list(t, :);
    
    F_list = [];
    a_list = [];
    [t1, t2, t3] = size(n_cars_vel);

    for j = 1:t1
        % get positions
        xi = position_list(t, 1);
        yi = position_list(t, 2);
        xj = n_cars_pos(j, 1, t);
        yj = n_cars_pos(j, 2, t);

        speed = sqrt(velocity(1) ^ 2 + velocity(2) ^ 2);
        
        [F_ij, angle] = ElasticForce(mess, speed, xi, yi, xj, yj, 1.5*speed, 3.5, yaw_list(t));
        F_list = [F_list; F_ij];
        a_list = [a_list; angle];
    end
    
    Ux = -G_i + F_di;
    Uy = F_li;
    for j = 1:height(n_cars_vel)
        Ux = Ux + F_list(j) * cos(a_list(j));
        Uy = Uy + F_list(j) * sin(a_list(j));
    end
    U = Ux * position_list(t, 1)-position_list(t-1, 1) + Uy * position_list(t, 2)-position_list(t-1, 2);
    S = S + K - U;
end

