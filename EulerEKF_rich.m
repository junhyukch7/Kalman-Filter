function [phi,theta,psi] = EulerEKF_rich(z,rates,dt)
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


% A(1,:) = partial_diff(x1,phi,theta,0,0.01,0.01,0.01);
% A(2,:) = partial_diff(x2,phi,theta,0,0.01,0.01,0.01);
% A(3,:) = partial_diff(x3,phi,theta,0,0.01,0.01,0.01);

r1 = rich(x1,phi,theta,0,0.1);
r2 = rich(x2,phi,theta,0,0.1);
r3 = rich(x3,phi,theta,0,0.1);

A(:,1) = r1(1);
A(:,2) = r2(2);
A(:,3) = r3(3);

A = eye(3) + A*dt; %반드시 이산시스템으로 바꿔줘야함
end


function dfdx = partial_diff_x(f,x,y,z,h)
dfdx = (f(x+h,y,z) - f(x-h,y,z))/(2*h);
end

function dfdy = partial_diff_y(f,x,y,z,h)
dfdy = (f(x,y+h,z) - f(x,y-h,z))/(2*h);
end

function dfdz = partial_diff_z(f,x,y,z,h)
dfdz = (f(x,y,z+h) - f(x,y,z-h))/(2*h);
end

function [q,ea,iter]=rich(f,x,y,z,h,es,maxit,varargin)
if nargin<5,error('at least 4 input arguments required'),end
if nargin<6||isempty(es), es=0.000001;end
if nargin<7||isempty(maxit), maxit=6;end
n = 1;
Dx(1,1) = partial_diff_x(f,x,y,z,h/n);
Dy(1,1) = partial_diff_y(f,x,y,z,h/n);
Dz(1,1) = partial_diff_z(f,x,y,z,h/n);
iter = 0;

while iter<maxit
  iter = iter+1;
  n = 2^iter;
  Dx(iter+1,1) = partial_diff_x(f,x,y,z,h/n);
  Dy(iter+1,1) = partial_diff_y(f,x,y,z,h/n);
  Dz(iter+1,1) = partial_diff_z(f,x,y,z,h/n);
  for k = 2:iter+1
    j = 2+iter-k;
    Dx(j,k) = (4^(k-1)*Dx(j+1,k-1)-Dx(j,k-1))/(4^(k-1)-1);
    Dy(j,k) = (4^(k-1)*Dy(j+1,k-1)-Dy(j,k-1))/(4^(k-1)-1);
    Dz(j,k) = (4^(k-1)*Dz(j+1,k-1)-Dz(j,k-1))/(4^(k-1)-1);
  end
  ea = abs((Dx(1,iter+1)-Dx(2,iter))/Dx(1,iter+1))*100;
  if (ea<=es), break; end
end

q = [Dx(1,iter+1) Dy(1,iter+1) Dz(1,iter+1)];

end