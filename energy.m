function [K, G_i, F_di, F_li, F_ij] = energy(mess, velocity)
K = KineticEnergy(mess, velocity);
G_i = AttractiveForce(mess, velocity);
F_di = ConstraintResistanceForce(mess, velocity);

end

function K = KineticEnergy(mess, velocity)
K = 0.5 * mess * velocity^2;
end

function g_h = PseudoGA(velocity, varargin)
% The acceleration due to the pseudo-gravity:
%        g_h = k * (v/v_limit) * g
% where v = velocity, k is a constent, v_limit denote the speed limit, and
% g is the gravitational acceleration.

p = inputParser;
addParameter(p,'v_limit',50);
addParameter(p,'k',0.2);
addParameter(p,'g',9.8);
parse(p,varargin{:});
v_limit = p.Results.v_limit;
k = p.Results.k;
g = p.Results.g;

g_h = k * (velocity/v_limit) * g;
end

function G = AttractiveForce(mess, velocity)
% The attractive force is expressed as:
%        G = m * g_h
% where m = mess and  g_h denotes the acceleration due to the
% pseudo-gravity.

G = mess * PseudoGA(velocity);
end

function F = ConstraintResistanceForce(mess, velocity, varargin)

p = inputParser;
addParameter(p,'v_limit',50);
addParameter(p,'tau',1);
parse(p,varargin{:});
v_limit = p.Results.v_limit;
tau = p.Results.tau;

F = mess * PseudoGA(velocity) * (velocity/v_limit)^tau;
end

