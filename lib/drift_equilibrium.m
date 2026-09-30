function trim = drift_equilibrium(target,p)
%DRIFT_EQUILIBRIUM Damped Newton solve for a steady circular drift.
% Unknown z=[vy r delta throttle]. Residual=[dvx dvy dr beta-betaTarget].
vx=target.vx;
sgn = sign(target.beta); if sgn==0, sgn=-1; end
r0 = -sgn*vx/target.radius; % chosen to create counter-steer drift geometry
z=[vx*tan(target.beta); r0; -sgn*deg2rad(16); 0.36];

for k=1:45
    F=residual(z,vx,target.beta,p);
    if norm(F)<2e-5, break; end
    J=zeros(4); h=[1e-4 1e-5 1e-5 1e-5];
    for j=1:4
        zp=z; zp(j)=zp(j)+h(j);
        zm=z; zm(j)=zm(j)-h(j);
        J(:,j)=(residual(zp,vx,target.beta,p)-residual(zm,vx,target.beta,p))/(2*h(j));
    end
    dz = -pinv(J)*F;
    lam=1;
    base=norm(F);
    for ls=1:10
        zn=project(z+lam*dz,p);
        if norm(residual(zn,vx,target.beta,p)) < base, break; end
        lam=lam/2;
    end
    z=project(z+lam*dz,p);
end
F=residual(z,vx,target.beta,p);
trim.vx=vx; trim.vy=z(1); trim.r=z(2); trim.delta=z(3); trim.throttle=z(4);
trim.beta=atan2(z(1),vx); trim.radius=hypot(vx,z(1))/max(abs(z(2)),1e-6);
trim.residual=F; trim.converged=norm(F)<0.15;
end

function F=residual(z,vx,betaTarget,p)
x=[0;0;0;vx;z(1);z(2)]; u=[z(3);z(4)];
[dx,~]=vehicle_dynamics(x,u,p);
F=[dx(4);dx(5);dx(6); 3*(atan2(z(1),vx)-betaTarget)];
end
function z=project(z,p)
z(3)=max(-p.maxSteer,min(p.maxSteer,z(3)));
z(4)=max(0.02,min(0.98,z(4)));
z(1)=max(-15,min(15,z(1))); z(2)=max(-2,min(2,z(2)));
end
