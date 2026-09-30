function ab = run_ablation_study(p,sched,cfg)
%RUN_ABLATION_STUDY Estimator and MPC constraint ablations.
sc=research_scenarios();
% Estimator ablation: high-noise case without simultaneous plant disturbances.
base=cfg; base.scenario=sc(5); base.caseSeed=cfg.randomSeed+7000; base.nominalParams=p; base.enableDisturbance=false;
a=base;a.estimatorMode='truth'; mpcTruth=simulate_research_case('mpc',p,sched,a);
a=base;a.estimatorMode='raw';   mpcRaw=simulate_research_case('mpc',p,sched,a);
a=base;a.estimatorMode='ekf';   mpcEkf=simulate_research_case('mpc',p,sched,a);
% Constraint ablation: same combined challenge and EKF feedback.
base=cfg; base.scenario=sc(6); base.caseSeed=cfg.randomSeed+7100; base.nominalParams=p; base.enableDisturbance=true; base.estimatorMode='ekf';
a=base;a.enforceCommonCommandRateLimits=false;a.mpcUseRateConstraints=false;mpcNoRate=simulate_research_case('mpc',p,sched,a);
a=base;a.enforceCommonCommandRateLimits=false;a.mpcUseRateConstraints=true;mpcRate=simulate_research_case('mpc',p,sched,a);
ab.labels={'MPC truth states','MPC raw sensors','MPC + EKF','MPC no slew constraint','Constrained MPC'};ab.runs={mpcTruth,mpcRaw,mpcEkf,mpcNoRate,mpcRate};
fid=fopen(fullfile(cfg.outputDir,'Table_VII_ablation_study.csv'),'w');fprintf(fid,'Variant,Beta_RMSE_deg,Integrated_Position_RMSE_m,Yaw_RMSE_deg_s,Speed_RMSE_km_h,Control_Effort,Command_Slew_Violations,Success\n');
for i=1:numel(ab.runs),m=ab.runs{i}.metrics;fprintf(fid,'%s,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n',ab.labels{i},m.betaRMSEdeg,m.pathRMSE,m.yawRMSEdeg,m.speedRMSEkmh,m.controlEffort,m.constraintViolations,m.success);end;fclose(fid);
end
