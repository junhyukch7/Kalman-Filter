function [pos,vel,alt] = RadarEKF(z,dt)
% input : 
% A = system matrix using quaternion
% z = position meaurement
% output :
% phi = roll, theta = pitch, psi = yaw
persistent A Q R;
persistent x P;
persistent firstRun;

if(isempty(firstRun))
    firstRun = 1;
    x = [0 90 1100]'; % 수평거리 속도 고도
    A = eye(3) + [0 1 0;0 0 0;0 0 0]*dt;
    P = 10*eye(3); %오차의 공분산과 시스템의 잡음공분산(Q)는 항상 행렬의 크기가 같아야 함
    Q= 0.001*eye(3); Q(1,1) = 0;
    R= 10;
end
H = Hjacob2(x);
    
% prediction for estimation value and error
xp = A * x;
Pp = A*P*A' + Q;

% calculate gain
K = Pp*H'*inv(H*Pp*H'+R);

% calculate estimation value
x = xp + K*(z-H*xp);
% calculate error
P = Pp-(K*H*Pp);
pos = x(1); vel = x(2); alt = x(3);

end

function H = Hjacob(xp)

x1 = xp(1);
x3 = xp(3);
H = [x1/sqrt(x1^2+x3^2) 0 x3/sqrt(x1^2+x3^2)];
end

function H = Hjacob2(xp)
x1 = xp(1);
x3 = xp(3);
r = @(x1,x3) sqrt(x1.^2+x3.^2);
H = partial_diff(r,x1,x3,0.01,0.01);
end

function H = partial_diff(f,x,y,dx,dy)

dfdx1 = (f(x+dx,y) - f(x-dx,y))/(2*dx);
dfdx3 = (f(x,y+dy) - f(x,y-dy))/(2*dy);

H = [dfdx1 0 dfdx3];
end