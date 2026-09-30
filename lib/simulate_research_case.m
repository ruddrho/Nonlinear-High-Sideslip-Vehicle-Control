function sim = simulate_research_case(controller,p,sched,cfg)
%SIMULATE_RESEARCH_CASE Closed-loop 4-wheel validation with reproducible scenarios.
% Supports PID, fixed LQR, gain-scheduled LQR and toolbox-free constrained MPC.
if ~isfield(cfg,'robustnessChallenge'), cfg.robustnessChallenge=true; end
if ~isfield(cfg,'enableDisturbance'), cfg.enableDisturbance=true; end
if ~isfield(cfg,'sensorNoise'), cfg.sensorNoise=true; end
if ~isfield(cfg,'mpcHorizon'), cfg.mpcHorizon=10; end
if ~isfield(cfg,'mpcIterations'), cfg.mpcIterations=16; end
if ~isfield(cfg,'mpcSteerRate'), cfg.mpcSteerRate=deg2rad(120); end
if ~isfield(cfg,'mpcThrottleRate'), cfg.mpcThrottleRate=1.8; end
if ~isfield(cfg,'enforceCommonCommandRateLimits'), cfg.enforceCommonCommandRateLimits=false; end
if ~isfield(cfg,'estimatorMode'), cfg.estimatorMode='ekf'; end
if isfield(cfg,'caseSeed'), rng(cfg.caseSeed); end
sc=default_scenario(cfg);
% Separate nominal design/reference parameters from the validation plant.
% This is essential for a defensible model-mismatch/Monte-Carlo study.
pRef=p; if isfield(cfg,'nominalParams') && ~isempty(cfg.nominalParams), pRef=cfg.nominalParams; end
dt=cfg.researchDt; t=(0:dt:cfg.researchT)'; N=numel(t);
ref=research_reference(t,pRef,cfg.trim);
x=zeros(N,8);
% Start every controller from the SAME small perturbation around the exact
% benchmark drift trim.  The previous launch-from-3-m/s initial condition
% was outside the 20-40 km/h gain-schedule envelope and invalidated a fair
% local LQR/MPC comparison.
tr0=ref.trim;
x0=[0;0;0;tr0.vx-0.20;tr0.vy+0.10;tr0.r-0.02;tr0.delta;tr0.throttle];
x(1,:)=x0'; u=zeros(N,2);
zh=zeros(N,5); zh(1,:)=[x0(4) x0(5) x0(6) x0(7) x0(8)]; P=diag([0.25 0.35 deg2rad(2) deg2rad(1) 0.02].^2);
y=zeros(N,5); innov=zeros(N,5); beta=zeros(N,1); util=zeros(N,4); muHist=zeros(N,1);
challenge=zeros(N,1); computeTime=zeros(N,1); mpcCost=nan(N,1);
intV=0; intBeta=0; prevBetaErr=0; gustApplied=false; vyRaw=0;
ref.X=zeros(N,1); ref.Y=zeros(N,1); ref.psi=zeros(N,1);
for k=1:N-1
    ref.psi(k+1)=ref.psi(k)+dt*ref.r(k); chi=ref.psi(k)+ref.beta(k);
    ref.X(k+1)=ref.X(k)+dt*ref.vx(k)*cos(chi); ref.Y(k+1)=ref.Y(k)+dt*ref.vx(k)*sin(chi);
end
% Initial sensor sample.
[~,aux0]=four_wheel_vehicle_dynamics(x(1,:)',[ref.trim.delta;ref.trim.throttle],p); psens=scaled_sensor_params(p,sc.sensorNoiseScale);
y(1,:)=sensor_model(x(1,:)',aux0,psens,sc.sensorNoise && cfg.sensorNoise)';
for k=1:N-1
    pk=p;
    if cfg.enableDisturbance && sc.wetEnabled && t(k)>=sc.wetStart && t(k)<sc.wetEnd
        pk.mu=sc.wetMuScale*p.mu; challenge(k)=2;
    end
    if cfg.enableDisturbance && sc.payloadEnabled && t(k)>=sc.payloadStart && t(k)<sc.payloadEnd
        pk.m=sc.massScale*p.m; pk.Iz=sc.inertiaScale*p.Iz;
        pk.Fzf=pk.m*pk.g*pk.b/pk.L; pk.Fzr=pk.m*pk.g*pk.a/pk.L; challenge(k)=3;
    end
    muHist(k)=pk.mu;
    switch lower(cfg.estimatorMode)
        case 'truth'
            z=[x(k,4);x(k,5);x(k,6);x(k,7);x(k,8)];
        case 'raw'
            if k>1, vyRaw=0.995*vyRaw+dt*(y(k,3)-max(y(k,1),0.5)*y(k,2)); end
            z=[max(y(k,1),0.5);vyRaw;y(k,2);y(k,4);y(k,5)];
        otherwise
            z=zh(k,:)';
    end
    refz=[ref.vx(k);ref.vy(k);ref.r(k)]; ct=tic;
    switch lower(controller)
        case 'scheduled_lqr'
            [K,~,~]=scheduled_lqr_gain(ref.vx(k),ref.beta(k),sched);
            ff=[ref.trim.delta;ref.trim.throttle]; eaug=z-[refz;ref.trim.delta;ref.trim.throttle]; uc=ff-K*eaug;
        case 'fixed_lqr'
            K=sched.Kfixed; if ref.beta(k)>0,S=diag([1 -1 -1 -1 1]);T=diag([-1 1]);K=T*K*S;end
            ff=[ref.trim.delta;ref.trim.throttle]; eaug=z-[refz;ref.trim.delta;ref.trim.throttle]; uc=ff-K*eaug;
        case 'mpc'
            if k==1,uPrev=[ref.trim.delta;ref.trim.throttle];else,uPrev=u(k-1,:)';end
            cfgM=cfg; cfgM.referenceFeedforward=[ref.trim.delta;ref.trim.throttle];
            [uc,mi]=constrained_mpc_controller(z,refz,ref.vx(k),ref.beta(k),sched,pRef,dt,uPrev,cfgM); mpcCost(k)=mi.cost;
        otherwise
            bHat=atan2(z(2),max(z(1),0.5)); eb=ref.beta(k)-bHat; er=ref.r(k)-z(3); ev=ref.vx(k)-z(1);
            intV=max(-3,min(3,intV+ev*dt)); intBeta=max(-0.8,min(0.8,intBeta+eb*dt)); dBeta=(eb-prevBetaErr)/dt; prevBetaErr=eb;
            ff=[ref.trim.delta;ref.trim.throttle];
            % Classical baseline uses equilibrium feedforward plus PID/PI
            % corrections.  Earlier versions omitted feedforward, which made
            % the comparison unfair at a large-sideslip equilibrium.
            uc=[ff(1)+2.0*eb+0.22*er+0.22*intBeta+0.018*dBeta;ff(2)+0.11*ev+0.035*intV];
    end
    computeTime(k)=toc(ct);
    uc(1)=max(-p.maxSteer,min(p.maxSteer,uc(1))); uc(2)=max(0,min(1,uc(2)));
    if cfg.enforceCommonCommandRateLimits
        ds=cfg.mpcSteerRate*dt; dth=cfg.mpcThrottleRate*dt;
        if k>1, prev=u(k-1,:)'; else, prev=[ref.trim.delta;ref.trim.throttle]; end
        uc(1)=max(prev(1)-ds,min(prev(1)+ds,uc(1))); uc(2)=max(prev(2)-dth,min(prev(2)+dth,uc(2)));
    end
    u(k,:)=uc'; xx=x(k,:)';
    k1=four_wheel_vehicle_dynamics(xx,uc,pk); k2=four_wheel_vehicle_dynamics(xx+dt*k1/2,uc,pk);
    k3=four_wheel_vehicle_dynamics(xx+dt*k2/2,uc,pk); k4=four_wheel_vehicle_dynamics(xx+dt*k3,uc,pk);
    xn=xx+dt*(k1+2*k2+2*k3+k4)/6;
    if cfg.enableDisturbance && sc.gustEnabled && ~gustApplied && t(k)>=sc.gustTime
        xn(5)=xn(5)+sc.gustVyImpulse; gustApplied=true; challenge(k)=1;
    end
    xn(4)=max(0.5,xn(4)); xn(7)=max(-p.maxSteer,min(p.maxSteer,xn(7))); xn(8)=max(0,min(1,xn(8))); x(k+1,:)=xn';
    [~,aux]=four_wheel_vehicle_dynamics(xn,uc,pk); psens=scaled_sensor_params(p,sc.sensorNoiseScale);
    y(k+1,:)=sensor_model(xn,aux,psens,sc.sensorNoise && cfg.sensorNoise)';
    [znew,P,iv]=ekf_step(zh(k,:)',P,uc,y(k+1,:)',dt,psens); zh(k+1,:)=znew'; innov(k+1,:)=iv';
    beta(k)=atan2(xx(5),max(xx(4),0.1)); util(k,:)=aux.utilization';
end
muHist(N)=muHist(max(1,N-1)); u(N,:)=u(max(1,N-1),:); beta(N)=atan2(x(N,5),max(x(N,4),0.1)); util(N,:)=util(max(1,N-1),:);
computeTime(N)=computeTime(max(1,N-1)); challenge(N)=challenge(max(1,N-1));
sim.controller=controller;sim.t=t;sim.x=x;sim.u=u;sim.zhat=zh;sim.y=y;sim.innovation=innov;sim.beta=beta;sim.utilization=util;sim.mu=muHist;sim.ref=ref;
sim.challenge=challenge;sim.computeTime=computeTime;sim.mpcCost=mpcCost;sim.scenario=sc;sim.estimatorMode=cfg.estimatorMode;
sim.metrics=research_metrics(sim,dt,cfg);
end

function sc=default_scenario(cfg)
if isfield(cfg,'scenario') && ~isempty(cfg.scenario),sc=cfg.scenario;return;end
sc=research_scenarios(); sc=sc(6);
if ~cfg.robustnessChallenge,sc=sc(1);end
end
function ps=scaled_sensor_params(p,s)
ps=p; f={'sigmaVx','sigmaYaw','sigmaAy','sigmaSteer','sigmaThrottle'}; for i=1:numel(f),ps.(f{i})=p.(f{i})*s;end
end
function m=research_metrics(sim,dt,cfg)
mask=sim.t>=1.0 & sim.t<=min(23.5,sim.t(end)); eb=wrap_pi(sim.beta-sim.ref.beta); er=sim.x(:,6)-sim.ref.r; ev=sim.x(:,4)-sim.ref.vx; ep=hypot(sim.x(:,1)-sim.ref.X,sim.x(:,2)-sim.ref.Y);
m.betaRMSEdeg=sqrt(mean(eb(mask).^2))*180/pi; m.yawRMSEdeg=sqrt(mean(er(mask).^2))*180/pi; m.speedRMSEkmh=sqrt(mean(ev(mask).^2))*3.6; m.pathRMSE=sqrt(mean(ep(mask).^2));
m.maxBetaErrorDeg=max(abs(eb(mask)))*180/pi; m.p95BetaErrorDeg=percentile(abs(eb(mask))*180/pi,95); m.maxSteerDeg=max(abs(sim.u(:,1)))*180/pi;
m.controlEffort=trapz(sim.t,sim.u(:,1).^2+0.20*sim.u(:,2).^2); E=[ev sim.x(:,5)-sim.ref.vy er]; stage=0.9*E(:,1).^2+4.5*E(:,2).^2+8.0*E(:,3).^2+2.4*sim.u(:,1).^2+0.8*sim.u(:,2).^2; m.performanceIndexJ=trapz(sim.t,stage);
m.maxTireUtil=max(sim.utilization(:)); m.meanComputeMs=1000*mean(sim.computeTime(mask)); m.maxComputeMs=1000*max(sim.computeTime(mask));
rawV=sim.y(:,1); rawR=sim.y(:,2); estV=sim.zhat(:,1); estR=sim.zhat(:,3); m.rawSpeedRMSE=sqrt(mean((rawV(mask)-sim.x(mask,4)).^2)); m.ekfSpeedRMSE=sqrt(mean((estV(mask)-sim.x(mask,4)).^2)); m.rawYawRMSEdeg=sqrt(mean((rawR(mask)-sim.x(mask,6)).^2))*180/pi; m.ekfYawRMSEdeg=sqrt(mean((estR(mask)-sim.x(mask,6)).^2))*180/pi;
% Recovery is measured after the last deterministic plant disturbance.
recStart=16.0; idx=find(sim.t>=recStart); rec=NaN; win=max(2,round(0.5/dt));
if ~isempty(idx),for q=idx(1):max(idx(1),numel(sim.t)-win),qq=q:min(numel(sim.t),q+win-1);if all(abs(eb(qq))<deg2rad(5))&&all(abs(er(qq))<deg2rad(10)),rec=sim.t(q)-recStart;break;end,end,end
m.recoveryTime=rec;
if isfield(cfg,'mpcSteerRate'),sr=cfg.mpcSteerRate;else,sr=inf;end;if isfield(cfg,'mpcThrottleRate'),tr=cfg.mpcThrottleRate;else,tr=inf;end
if size(sim.u,1)>1,du=diff(sim.u)/dt;m.constraintViolations=sum(abs(du(:,1))>sr+1e-9)+sum(abs(du(:,2))>tr+1e-9);else,m.constraintViolations=0;end
% Publication success criterion: finite response plus genuinely useful
% drift-state tracking.  Path RMSE is NOT part of success because this
% benchmark has no outer-loop global-position controller.
m.success=all(isfinite(sim.x(:))) && m.betaRMSEdeg<10 && m.p95BetaErrorDeg<15 && ...
    m.yawRMSEdeg<25 && m.speedRMSEkmh<5 && max(abs(sim.x(:,6)))<3.5;
% Command-slew violations remain a reported metric.  They are not a generic
% success criterion because all controllers share the plant's physical
% actuator dynamics/rate limit, while explicit command-rate constraints are
% an MPC design feature being evaluated separately.
end
function q=percentile(x,p),x=sort(x(:));if isempty(x),q=NaN;return;end;k=1+(numel(x)-1)*p/100;i=floor(k);j=ceil(k);if i==j,q=x(i);else,q=x(i)+(k-i)*(x(j)-x(i));end,end
function y=wrap_pi(x),y=atan2(sin(x),cos(x));end
