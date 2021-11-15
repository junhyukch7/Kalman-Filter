function [pos,vel,alt] = RadarEKF_rich(z,dt)
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

function H = Hjacob2(xp)
x1 = xp(1);
x3 = xp(3);
r = @(x1,x3) sqrt(x1.^2+x3.^2);

H2 = rich(r,x1,x3,0.5);
H = [H2(1) 0 H2(2)];
end

function dfdy = partial_diff_y(f,x,y,h)
dfdy = (f(x,y+h) - f(x,y-h))/(2*h);
end

function dfdx = partial_diff_x(f,x,y,h)
dfdx = (f(x+h,y) - f(x-h,y))/(2*h);
end


function [q,ea,iter]=rich(f,x,y,h,es,maxit,varargin)
if nargin<4,error('at least 4 input arguments required'),end
if nargin<5||isempty(es), es=0.000001;end
if nargin<6||isempty(maxit), maxit=6;end
n = 1;
Dx(1,1) = partial_diff_x(f,x,y,h/n);
Dy(1,1) = partial_diff_y(f,x,y,h/n);
iter = 0;

while iter<maxit
  iter = iter+1;
  n = 2^iter;
  Dx(iter+1,1) = partial_diff_x(f,x,y,h/n);
  Dy(iter+1,1) = partial_diff_y(f,x,y,h/n);
  for k = 2:iter+1
    j = 2+iter-k;
    Dx(j,k) = (4^(k-1)*Dx(j+1,k-1)-Dx(j,k-1))/(4^(k-1)-1);
    Dy(j,k) = (4^(k-1)*Dy(j+1,k-1)-Dy(j,k-1))/(4^(k-1)-1);
  end
  ea = abs((Dx(1,iter+1)-Dx(2,iter))/Dx(1,iter+1))*100;
  if (ea<=es), break; end
end

q = [Dx(1,iter+1) Dy(1,iter+1)];

end
