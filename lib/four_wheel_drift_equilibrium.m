function trim = four_wheel_drift_equilibrium(target,p)
%FOUR_WHEEL_DRIFT_EQUILIBRIUM Steady drift trim for the 4-wheel plant.
vx=target.vx;
if abs(target.beta)<deg2rad(0.25)
    Fdrag=0.5*p.rho*p.CdA*vx*abs(vx); Froll=p.Crr*p.m*p.g*tanh(vx/0.5);
    z=[0;0;0;max(0.02,min(0.98,(Fdrag+Froll)/p.maxDrive))];
else
    b=drift_equilibrium(target,p); z=[b.vy;b.r;b.delta;b.throttle];
end
for k=1:60
    F=residual(z,vx,target.beta,p); if norm(F)<2e-5,break;end
    J=zeros(4); hh=[1e-4 1e-5 1e-5 1e-5];
    for j=1:4
        zp=z;zm=z;zp(j)=zp(j)+hh(j);zm(j)=zm(j)-hh(j);
        J(:,j)=(residual(zp,vx,target.beta,p)-residual(zm,vx,target.beta,p))/(2*hh(j));
    end
    dz=-pinv(J)*F; lam=1;base=norm(F);
    for q=1:12
        zn=project(z+lam*dz,p); if norm(residual(zn,vx,target.beta,p))<base,break;end;lam=lam/2;
    end
    z=project(z+lam*dz,p);
end
F=residual(z,vx,target.beta,p);
trim.vx=vx;trim.vy=z(1);trim.r=z(2);trim.delta=z(3);trim.throttle=z(4);trim.beta=atan2(z(1),vx);trim.residual=F;trim.converged=norm(F)<0.15;
end
function F=residual(z,vx,beta,p)
x=[0;0;0;vx;z(1);z(2);z(3);z(4)];u=z(3:4);dx=four_wheel_vehicle_dynamics(x,u,p);F=[dx(4);dx(5);dx(6);3*(atan2(z(1),vx)-beta)];
end
function z=project(z,p),z(1)=max(-15,min(15,z(1)));z(2)=max(-2,min(2,z(2)));z(3)=max(-p.maxSteer,min(p.maxSteer,z(3)));z(4)=max(0.01,min(0.99,z(4)));end
