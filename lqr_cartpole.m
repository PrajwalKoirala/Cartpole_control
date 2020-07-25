syms x dx theta dtheta u;     % define symbolic variables x, dx, theta, dtheta and u
accelerations = calc_acc(x, dx, theta, dtheta, u);     % calculate angular and translational accelerations
f = [dx; accelerations(1); dtheta; accelerations(2)];  % f is our nonlinear function of state variables and u

% calculates the partial derivatives :
A = jacobian(f, [x; dx; theta; dtheta]);         
B = jacobian(f, u);

% linearize about [x, theta, dx, dtheta, u] = [0,0,0,0, 0]
A = double(subs(A, {x, theta, dx, dtheta, u},{0,0,0,0, 0}))
B = double(subs(B, {x, theta, dx, dtheta, u},{0,0,0,0, 0}))

% Q and R values give corresponding weightage to variables and control signal
Q = [1  0   0   0;
     0  10  0   0;
     0  0   1   0;
     0  0   0   100];
R = 0.1;

k = lqr(A, B, Q, R)
