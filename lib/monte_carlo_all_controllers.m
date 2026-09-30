function mc = monte_carlo_all_controllers(p,sched,cfg)
%MONTE_CARLO_ALL_CONTROLLERS Same randomized parameter set for all controllers.
controllers=cfg.paperControllers;labels=cfg.paperControllerLabels;n=cfg.paperMonteCarloRuns;nc=numel(controllers);
B=nan(n,nc);Y=B;V=B;Pth=B;Eff=B;Comp=B;Succ=false(n,nc);
% Generate perturbations once; every controller sees exactly the same cases.
rng(cfg.randomSeed+9000); dm=0.10*randn(n,1);dIz=0.12*randn(n,1);dCaf=0.15*randn(n,1);dCar=0.15*randn(n,1);dMu=0.10*randn(n,1);
sc=research_scenarios();sc=sc(1); cm=cfg;cm.researchDt=max(cfg.monteCarloDt,cfg.researchDt);cm.scenario=sc;cm.sensorNoise=false;cm.estimatorMode='truth';cm.enableDisturbance=false;cm.nominalParams=p;
% Monte Carlo isolates parametric model mismatch.  Deterministic wet-road,
% gust and payload events are evaluated separately in the scenario matrix.
for i=1:n
    pi=p;pi.m=max(0.70*p.m,p.m*(1+dm(i)));pi.Iz=max(0.65*p.Iz,p.Iz*(1+dIz(i)));pi.Caf=max(0.55*p.Caf,p.Caf*(1+dCaf(i)));pi.Car=max(0.55*p.Car,p.Car*(1+dCar(i)));pi.mu=max(0.72,min(1.30,p.mu*(1+dMu(i))));pi.Fzf=pi.m*pi.g*pi.b/pi.L;pi.Fzr=pi.m*pi.g*pi.a/pi.L;
    for c=1:nc
        cm.caseSeed=cfg.randomSeed+100000+i; s=simulate_research_case(controllers{c},pi,sched,cm);m=s.metrics;
        B(i,c)=m.betaRMSEdeg;Y(i,c)=m.yawRMSEdeg;V(i,c)=m.speedRMSEkmh;Pth(i,c)=m.pathRMSE;Eff(i,c)=m.controlEffort;Comp(i,c)=m.meanComputeMs;Succ(i,c)=m.success;
    end
    if mod(i,max(1,round(n/10)))==0,fprintf('  Monte Carlo %d/%d complete\n',i,n);end
end
mc.controllers=controllers;mc.labels=labels;mc.betaRMSEdeg=B;mc.yawRMSEdeg=Y;mc.speedRMSEkmh=V;mc.pathRMSE=Pth;mc.controlEffort=Eff;mc.computeMs=Comp;mc.success=Succ;mc.successRate=100*mean(Succ,1);
mc.meanBeta=mean(B,1);mc.stdBeta=std(B,0,1);mc.medianBeta=median(B,1);mc.p95Beta=zeros(1,nc);mc.worstBeta=max(B,[],1);for c=1:nc,mc.p95Beta(c)=percentile(B(:,c),95);end
write_mc_csv(mc,cfg.outputDir);
end
function write_mc_csv(mc,out)
fid=fopen(fullfile(out,'Table_V_monte_carlo_statistics.csv'),'w');fprintf(fid,'Controller,Runs,Mean_Beta_RMSE_deg,Std_Beta_RMSE_deg,Median_Beta_RMSE_deg,P95_Beta_RMSE_deg,Worst_Beta_RMSE_deg,Success_Rate_pct,Mean_Integrated_Position_RMSE_m,Mean_Control_Effort,Mean_Compute_ms\n');
for c=1:numel(mc.controllers),fprintf(fid,'%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.6f,%.6f,%.6f\n',mc.labels{c},size(mc.betaRMSEdeg,1),mc.meanBeta(c),mc.stdBeta(c),mc.medianBeta(c),mc.p95Beta(c),mc.worstBeta(c),mc.successRate(c),mean(mc.pathRMSE(:,c)),mean(mc.controlEffort(:,c)),mean(mc.computeMs(:,c)));end;fclose(fid);
end
function q=percentile(x,p),x=sort(x(:));k=1+(numel(x)-1)*p/100;i=floor(k);j=ceil(k);if i==j,q=x(i);else,q=x(i)+(k-i)*(x(j)-x(i));end,end
