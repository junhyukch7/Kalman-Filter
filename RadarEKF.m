function [pos,vel,alt] = RadarEKF(z,dt)
% input : 
% z = Measurement distance
% z = sampling time
% output :
% pos = horizontal distance beteween radar and obj, vel = obj speed, alt = obj height from road 
persistent A Q R;
persistent x P;
persistent firstRun;

if(isempty(firstRun))
    firstRun = 1;
    x = [0 90 1100]'; % 수평거리 속도 고도
    A = eye(3) + [0 1 0;0 0 0;0 0 0]*dt;
    P = 10*eye(3); 
    Q= 0.001*eye(3); Q(1,1) = 0;
    R= 10;
end
H = Hjacob2(x); %Measurement system model is non-linear. must change linear sys by using jacobian matrix
    
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
r = @(x1,x3) sqrt(x1.^2+x3.^2); % variable x2 is not exit, so x2 diff = 0
H = partial_diff(r,x1,x3,0.01,0.01);
end

% To find out first order difference, using central diffrence(numerical method)
function H = partial_diff(f,x,y,dx,dy)

dfdx1 = (f(x+dx,y) - f(x-dx,y))/(2*dx);
dfdx3 = (f(x,y+dy) - f(x,y-dy))/(2*dy);

H = [dfdx1 0 dfdx3];
end
