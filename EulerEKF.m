function [phi,theta,psi] = EulerEKF(z,rates,dt)
% input : 
% A = system matrix using quaternion
% z = position meaurement
% output :
% phi = roll, theta = pitch, psi = yaw
persistent H Q R;
persistent x P;
persistent firstRun;

if(isempty(firstRun))
    firstRun = 1;
    x = [0 0 0]';% x(2):roll % x(3):pitch % x(4):yaw
    P = 10*eye(3); %오차의 공분산과 시스템의 잡음공분산(Q)는 항상 행렬의 크기가 같아야 함
    H = [1 0 0;0 1 0];
    Q= 0.0001*eye(3); Q(3,3) = 0.1;
    R= 10*eye(2);
end
A = Ajacob(x,rates,dt);% 여기서 문제
% prediction for estimation value and error
xp = fx(x,rates,dt);
Pp = A*P*A' + Q;

% calculate gain
K = Pp*H'*inv(H*Pp*H'+R);

% calculate estimation value
x = xp + K*(z-H*xp);
% calculate error
P = Pp-(K*H*Pp);

phi = x(1); theta = x(2); psi = x(3);
end

function xp = fx(xhat,rates,dt)
phi = xhat(1); theta = xhat(2);
p = rates(1); q = rates(2); r=rates(3);

xdot = zeros(3,1);
xdot(1) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
xdot(2) = q*cos(phi) - r*sin(phi);
xdot(3) = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);

xp = xhat + xdot*dt;
end


function A = Ajacob(z,rates,dt)
phi = z(1); theta = z(2); A = zeros(3,3);
p = rates(1); q = rates(2); r= rates(3);

x1 = @(phi,theta,row) p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta) + row*0;
x2 = @(phi,theta,row) q*cos(phi) - r*sin(phi) + theta*0 + row*0;
x3 = @(phi,theta,row) q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta) + row*0;

A(1,:) = partial_diff(x1,phi,theta,0,0.01,0.01,0.01);
A(2,:) = partial_diff(x2,phi,theta,0,0.01,0.01,0.01);
A(3,:) = partial_diff(x3,phi,theta,0,0.01,0.01,0.01);

A = eye(3) + A*dt; %반드시 이산시스템으로 바꿔줘야함
end

function H = partial_diff(f,x,y,z,dx,dy,dz)
% x= phi(roll), y = theta(pitch), z = row(yaw)

dfdx1 = (f(x+dx,y,z) - f(x-dx,y,z))/(2*dx);
dfdx2 = (f(x,y+dy,z) - f(x,y-dy,z))/(2*dy);
dfdx3 = (f(x,y,z+dz) - f(x,y,z-dz))/(2*dz);

H = [dfdx1 dfdx2 dfdx3];
end