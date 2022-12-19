function S = BehavioralDecisionMaking(mess, velocity_list, position_list,t_end, n_cars)

t = 1
% get initial velocity
velocity = velocity_list[t]
% get initial position
position = position_list[t]

% get r_li or
r_li = 3.5/2
[K, G_i, F_di, F_li, F_ij] = energy(mess, velocity, r_li)

S = 0
while t >= t_end :
    t = t+1
    % get next velocity
    velocity = velocity_list[t]
    % get next position
    position = position_list[t]
    
    F_list = []
    a_list = []
    for j = 1:n_cars.amount
        % get positions
        xi, yi = position[t]
        xj, yj = n_cars.position[j][t]
        
        F_ij, angle = ElasticForce(mess, velocity, xi, yi, xj, yj, a, b, angle_cari);
        F_list = append(F_list,F_ij);
        a_list = append(a_list,angle);
    end
    
    Ux = -G_i + F_di;
    Uy = F_li;
    for j = 1:n_cars.amount
        Ux = Ux + F_list[j] * cos(a_list[j]);
        Uy = Uy + F_list[j] * sin(a_list[j]);
    end
    U = Ux * position[t][x]-position[t-1][x] + Uy * position[t][y]-position[t-1][y];
    S = S + K - U;
end

end

