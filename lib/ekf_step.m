function [zhat,P,innov] = ekf_step(zhat,P,u,y,dt,p)
%EKF_STEP EKF for [vx vy r deltaAct throttleAct].
% Sensors: wheel-speed, yaw gyro, lateral acceleration, steering position,
% and throttle position.  The estimator uses the nonlinear 4-wheel model.
Qk=diag([0.10 0.20 deg2rad(0.7) deg2rad(0.35) 0.012].^2)*dt;
Rk=diag([p.sigmaVx p.sigmaYaw p.sigmaAy p.sigmaSteer p.sigmaThrottle].^2);
f=@(z) process_rate(z,u,p);h=@(z) measurement_model(z,u,p);
A=numjac(f,zhat);zpred=zhat+dt*f(zhat);F=eye(5)+dt*A;Ppred=F*P*F'+Qk;
H=numjac(h,zpred);ypred=h(zpred);innov=y-ypred;S=H*Ppred*H'+Rk;Kgain=(Ppred*H')/S;
zhat=zpred+Kgain*innov;I=eye(5);P=(I-Kgain*H)*Ppred*(I-Kgain*H)'+Kgain*Rk*Kgain';P=(P+P')/2;
zhat(1)=max(0.5,zhat(1));zhat(4)=max(-p.maxSteer,min(p.maxSteer,zhat(4)));zhat(5)=max(0,min(1,zhat(5)));
end
function dz=process_rate(z,u,p),x=[0;0;0;z(:)];dx=four_wheel_vehicle_dynamics(x,u,p);dz=dx(4:8);end
function y=measurement_model(z,u,p),x=[0;0;0;z(:)];dx=four_wheel_vehicle_dynamics(x,u,p);ay=dx(5)+z(1)*z(3);y=[z(1);z(3);ay;z(4);z(5)];end
function J=numjac(fun,x),y0=fun(x);J=zeros(numel(y0),numel(x));for i=1:numel(x),h=1e-5*max(1,abs(x(i)));xp=x;xm=x;xp(i)=xp(i)+h;xm(i)=xm(i)-h;J(:,i)=(fun(xp)-fun(xm))/(2*h);end,end
