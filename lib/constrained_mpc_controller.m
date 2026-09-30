function [u,info] = constrained_mpc_controller(z,refz,vxRef,betaRef,sched,p,dt,uPrev,cfg)
%CONSTRAINED_MPC_CONTROLLER Toolbox-free constrained linear MPC.
% The controller uses a locally scheduled linear model and a condensed
% finite-horizon quadratic cost. A projected-gradient QP iteration enforces
% steering/throttle magnitude and rate constraints without MPC Toolbox.
if nargin<9, cfg=struct; end
if ~isfield(cfg,'mpcHorizon'), cfg.mpcHorizon=10; end
if ~isfield(cfg,'mpcIterations'), cfg.mpcIterations=16; end
if ~isfield(cfg,'mpcSteerRate'), cfg.mpcSteerRate=deg2rad(120); end
if ~isfield(cfg,'mpcThrottleRate'), cfg.mpcThrottleRate=1.8; end
if ~isfield(cfg,'mpcUseRateConstraints'), cfg.mpcUseRateConstraints=true; end
[A,B,ff,eq]=scheduled_linear_model(vxRef,betaRef,sched);
if isfield(cfg,'referenceFeedforward') && numel(cfg.referenceFeedforward)==2
    ff=cfg.referenceFeedforward(:); eq(4:5)=ff;
end
% Exact ZOH is unnecessary here; first-order discretisation is sufficient
% at the research sample time and keeps the project toolbox-free.
Ad=eye(size(A))+dt*A; Bd=dt*B;
e=z-[refz;eq(4);eq(5)];
Np=max(3,round(cfg.mpcHorizon)); nx=size(Ad,1); nu=size(Bd,2);
Q=diag([1.2 5.0 9.0 0.8 0.35]); R=diag([1.5 0.65]); Qf=2.5*Q;
Phi=zeros(nx*Np,nx); Gamma=zeros(nx*Np,nu*Np);
Ap=eye(nx);
for i=1:Np
    Ap=Ad*Ap; Phi((i-1)*nx+1:i*nx,:)=Ap;
    for j=1:i
        Gamma((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu)=Ad^(i-j)*Bd;
    end
end
Qbar=kron(eye(Np),Q); Qbar(end-nx+1:end,end-nx+1:end)=Qf;
Rbar=kron(eye(Np),R);
H=Gamma'*Qbar*Gamma+Rbar; g=Gamma'*Qbar*Phi*e;
H=(H+H')/2+1e-8*eye(size(H));
% Warm start at the equilibrium command deviation.
D=zeros(nu*Np,1);
L=max(sum(abs(H),2)); alpha=0.75/max(L,1e-6);
for it=1:max(1,round(cfg.mpcIterations))
    D=D-alpha*(H*D+g);
    % Project sequentially onto magnitude and slew constraints.
    prev=uPrev(:);
    for j=1:Np
        q=(j-1)*nu+(1:nu); uj=ff+D(q);
        uj(1)=max(-p.maxSteer,min(p.maxSteer,uj(1)));
        uj(2)=max(0,min(1,uj(2)));
        if cfg.mpcUseRateConstraints
            ds=cfg.mpcSteerRate*dt; dtc=cfg.mpcThrottleRate*dt;
            uj(1)=max(prev(1)-ds,min(prev(1)+ds,uj(1)));
            uj(2)=max(prev(2)-dtc,min(prev(2)+dtc,uj(2)));
        end
        D(q)=uj-ff; prev=uj;
    end
end
u=ff+D(1:nu);
u(1)=max(-p.maxSteer,min(p.maxSteer,u(1))); u(2)=max(0,min(1,u(2)));
info.horizon=Np; info.iterations=cfg.mpcIterations; info.cost=0.5*D'*H*D+g'*D; info.ff=ff;
end
