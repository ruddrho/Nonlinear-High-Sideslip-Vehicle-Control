function [dx,aux] = vehicle_dynamics(x,u,p)
% Nonlinear planar RWD bicycle model with rear friction-circle coupling.
X=x(1); Y=x(2); psi=x(3); vx=x(4); vy=x(5); r=x(6); %#ok<NASGU>
delta = max(-p.maxSteer,min(p.maxSteer,u(1)));
throttle = max(0,min(1,u(2)));
vxs = sign(vx)*max(abs(vx),p.minVx);

af = atan2(vy+p.a*r,abs(vxs)) - delta;
ar = atan2(vy-p.b*r,abs(vxs));

Fyf = fiala_tire(af,p.Caf,p.mu*p.Fzf);
Fxr_req = throttle*p.maxDrive;
% Rear tyre uses the same grip budget for drive and lateral force.
rearBudget = sqrt(max((p.mu*p.Fzr)^2 - Fxr_req^2,0));
Fyr = fiala_tire(ar,p.Car,max(rearBudget,1));
Fxr = min(Fxr_req,p.mu*p.Fzr*0.995);

Fdrag = 0.5*p.rho*p.CdA*vx*abs(vx);
Froll = p.Crr*p.m*p.g*tanh(vx/0.5);

dvx = (Fxr - Fyf*sin(delta) - Fdrag - Froll)/p.m + vy*r;
dvy = (Fyf*cos(delta)+Fyr)/p.m - vx*r;
dr  = (p.a*Fyf*cos(delta)-p.b*Fyr)/p.Iz;

dX = vx*cos(psi)-vy*sin(psi);
dY = vx*sin(psi)+vy*cos(psi);
dpsi = r;
dx = [dX;dY;dpsi;dvx;dvy;dr];

aux.alpha_f=af; aux.alpha_r=ar; aux.Fyf=Fyf; aux.Fyr=Fyr; aux.Fxr=Fxr;
aux.beta=atan2(vy,max(abs(vx),0.1));
aux.speed=hypot(vx,vy);
aux.delta=delta; aux.throttle=throttle;
end
