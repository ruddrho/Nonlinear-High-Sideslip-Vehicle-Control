function [dx,aux] = four_wheel_vehicle_dynamics(x,uCmd,p)
%FOUR_WHEEL_VEHICLE_DYNAMICS Higher-fidelity planar validation plant.
% States: [X Y psi vx vy r deltaAct throttleAct].
% Inputs:  [deltaCmd throttleCmd].  Front axle steers; rear axle drives.
% Each tyre has its own Fiala lateral force and friction-ellipse limit.

X=x(1); Y=x(2); psi=x(3); vx=x(4); vy=x(5); r=x(6); %#ok<NASGU>
delta=x(7); throttle=x(8);
deltaCmd=max(-p.maxSteer,min(p.maxSteer,uCmd(1)));
throttleCmd=max(0,min(1,uCmd(2)));

% First-order actuators + steering-rate constraint.
deltaDot=max(-p.maxSteerRate,min(p.maxSteerRate,(deltaCmd-delta)/p.tauSteer));
throttleDot=(throttleCmd-throttle)/p.tauThrottle;
throttleEff=max(0,min(1,throttle));

% Quasi-static longitudinal and lateral load transfer.
FxReq=throttleEff*p.maxDrive;
dFLong=p.hCG*FxReq/p.L; % longitudinal load transfer (N)
FzfAx=max(0.20*p.m*p.g,p.Fzf-dFLong);
FzrAx=max(0.20*p.m*p.g,p.Fzr+dFLong);
ayGuess=vx*r;
dFLat=p.m*p.hCG*ayGuess/max(p.track,0.5);
dFf=dFLat*(p.b/p.L); dFr=dFLat*(p.a/p.L);
% Positive lateral acceleration transfers load from left to right.
Fz=[FzfAx/2-dFf/2; FzfAx/2+dFf/2; FzrAx/2-dFr/2; FzrAx/2+dFr/2];
Fz=max(Fz,0.08*p.m*p.g/4);

% Wheel locations: FL, FR, RL, RR. Body +y is left.
xw=[p.a;p.a;-p.b;-p.b]; yw=[p.track/2;-p.track/2;p.track/2;-p.track/2];
steer=[delta;delta;0;0];
Ca=[p.Caf/2;p.Caf/2;p.Car/2;p.Car/2];
Fx=zeros(4,1); Fy=zeros(4,1); alpha=zeros(4,1); util=zeros(4,1);

% Rear drive is split by rear normal load.
rearSum=max(Fz(3)+Fz(4),1);
FxRear=[0;0;FxReq*Fz(3)/rearSum;FxReq*Fz(4)/rearSum];

for i=1:4
    vxb=vx-r*yw(i); vyb=vy+r*xw(i);
    c=cos(steer(i)); s=sin(steer(i));
    vlong=c*vxb+s*vyb;
    vlat=-s*vxb+c*vyb;
    alpha(i)=atan2(vlat,max(abs(vlong),p.minVx));
    muFz=max(p.mu*Fz(i),1);
    FyPure=fiala_tire(alpha(i),Ca(i),muFz);
    Fxw=min(max(FxRear(i),-0.995*muFz),0.995*muFz);
    latScale=sqrt(max(0,1-(Fxw/muFz)^2));
    Fyw=FyPure*latScale;
    % Wheel -> body frame.
    Fx(i)=c*Fxw-s*Fyw;
    Fy(i)=s*Fxw+c*Fyw;
    util(i)=sqrt(Fxw^2+Fyw^2)/muFz;
end

Fdrag=0.5*p.rho*p.CdA*vx*abs(vx);
Froll=p.Crr*p.m*p.g*tanh(vx/0.5);
FxSum=sum(Fx)-Fdrag-Froll; FySum=sum(Fy);
Mz=sum(xw.*Fy-yw.*Fx);

dvx=FxSum/p.m+vy*r;
dvy=FySum/p.m-vx*r;
dr=Mz/p.Iz;
dX=vx*cos(psi)-vy*sin(psi);
dY=vx*sin(psi)+vy*cos(psi);
dpsi=r;
dx=[dX;dY;dpsi;dvx;dvy;dr;deltaDot;throttleDot];

aux.Fx=Fx; aux.Fy=Fy; aux.Fz=Fz; aux.alpha=alpha; aux.utilization=util;
aux.beta=atan2(vy,max(abs(vx),0.1)); aux.speed=hypot(vx,vy);
aux.ay=dvy+vx*r; aux.ax=dvx-vy*r; aux.delta=delta; aux.throttle=throttleEff;
end
