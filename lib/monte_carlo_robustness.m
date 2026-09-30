function mc = monte_carlo_robustness(p,sched,cfg)
%MONTE_CARLO_ROBUSTNESS Parameter-uncertainty study for scheduled LQR.
rng(cfg.randomSeed+100); n=cfg.monteCarloRuns;
B=zeros(n,1);Y=zeros(n,1);V=zeros(n,1);U=zeros(n,1);S=false(n,1);
% Use a coarser step for the batch study to keep the project practical.
cm=cfg; cm.researchDt=max(0.03,cfg.researchDt); cm.sensorNoise=false;
for i=1:n
    pi=p;
    pi.m=max(0.70*p.m,p.m*(1+0.10*randn)); pi.Iz=max(0.65*p.Iz,p.Iz*(1+0.12*randn));
    pi.Caf=max(0.55*p.Caf,p.Caf*(1+0.15*randn)); pi.Car=max(0.55*p.Car,p.Car*(1+0.15*randn));
    pi.mu=max(0.72,min(1.30,p.mu*(1+0.10*randn)));
    pi.Fzf=pi.m*pi.g*pi.b/pi.L; pi.Fzr=pi.m*pi.g*pi.a/pi.L;
    s=simulate_research_case('scheduled_lqr',pi,sched,cm);
    B(i)=s.metrics.betaRMSEdeg;Y(i)=s.metrics.yawRMSEdeg;V(i)=s.metrics.speedRMSEkmh;U(i)=s.metrics.controlEffort;S(i)=s.metrics.success;
end
mc.betaRMSEdeg=B;mc.yawRMSEdeg=Y;mc.speedRMSEkmh=V;mc.controlEffort=U;mc.success=S;
mc.successRate=100*mean(S);mc.meanBetaRMSE=mean(B);mc.p95BetaRMSE=percentile(B,95);mc.worstBetaRMSE=max(B);
end
function q=percentile(x,p), x=sort(x(:)); if isempty(x),q=NaN;return;end; k=1+(numel(x)-1)*p/100;i=floor(k);j=ceil(k);if i==j,q=x(i);else,q=x(i)+(k-i)*(x(j)-x(i));end,end
