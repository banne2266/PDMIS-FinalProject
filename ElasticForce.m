function [F_ij, angle] = ElasticForce(mess, velocity, xi, yi, xj, yj, a, b, angle_cari)
    d_ij = sqrt((xj)^2 + (yj)^2);
    delta_x_ij = sqrt(a^2*(xj)^2 + b^2*(yj)^2) / d_ij - d_ij;
    k = (mess * velocity^2) / (2 * d_ij);
    if delta_x_ij > 0
        F_ij = k * delta_x_ij;
        angle = atan2(yj,xj) - angle_cari;
    else
        F_ij = 0;
        angle = 0;
    end
end
