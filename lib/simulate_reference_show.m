function sim = simulate_reference_show(p,trim,K,cfg) %#ok<INUSD>
%SIMULATE_REFERENCE_SHOW Reference-matched visual stunt plus tyre-model evaluation.
t=(0:cfg.dt:cfg.T)'; N=numel(t); ref=reference_trajectory(t,p,trim);
x=[ref.X ref.Y ref.psi ref.vx ref.vy ref.r];
u=[ref.steer ref.throttle];
beta=zeros(N,1); speed=zeros(N,1); af=zeros(N,1); ar=zeros(N,1);
Fyf=zeros(N,1); Fyr=zeros(N,1); Fxr=zeros(N,1);
for k=1:N
    [~,a]=vehicle_dynamics(x(k,:)',u(k,:)',p);
    beta(k)=ref.beta(k); speed(k)=hypot(ref.vx(k),ref.vy(k));
    af(k)=a.alpha_f; ar(k)=a.alpha_r; Fyf(k)=a.Fyf; Fyr(k)=a.Fyr; Fxr(k)=a.Fxr;
end
sim.t=t; sim.x=x; sim.u=u; sim.beta=beta; sim.speed=speed;
sim.alpha_f=af; sim.alpha_r=ar; sim.Fyf=Fyf; sim.Fyr=Fyr; sim.Fxr=Fxr;
sim.mode=ref.mode; sim.trim=trim; sim.reference=ref; sim.referenceMatch=true;
end
