function bench = run_paper_scenarios(p,sched,cfg)
%RUN_PAPER_SCENARIOS Fair controller benchmark plus output-feedback cases.
% Scenarios 1-4 use true plant states to isolate controller-law behaviour.
% Scenarios 5-6 use the same EKF for every controller to test output feedback.
scenarios=research_scenarios(); controllers=cfg.paperControllers; labels=cfg.paperControllerLabels;
ns=numel(scenarios); nc=numel(controllers); runs=cell(ns,nc); estimator=cell(ns,1);
for s=1:ns
    if s<=4, estimator{s}=cfg.benchmarkEstimatorMode; else, estimator{s}=cfg.outputFeedbackEstimatorMode; end
    fprintf('\nScenario %d/%d: %s [%s feedback]\n',s,ns,scenarios(s).label,estimator{s});
    for c=1:nc
        cc=cfg; cc.scenario=scenarios(s); cc.caseSeed=cfg.randomSeed+1000*s; cc.estimatorMode=estimator{s}; cc.enableDisturbance=true; cc.nominalParams=p;
        fprintf('  %-22s ... ',labels{c}); runs{s,c}=simulate_research_case(controllers{c},p,sched,cc); fprintf('done\n');
    end
end
bench.scenarios=scenarios;bench.controllers=controllers;bench.labels=labels;bench.runs=runs;bench.estimatorMode=estimator;
write_scenario_csv(bench,cfg.outputDir);
end
function write_scenario_csv(b,out)
fid=fopen(fullfile(out,'Table_III_IV_scenario_benchmark.csv'),'w');
fprintf(fid,'Scenario,Estimator,Controller,Beta_RMSE_deg,Yaw_RMSE_deg_s,Speed_RMSE_km_h,Integrated_Position_RMSE_m,P95_Beta_Error_deg,Max_Beta_Error_deg,Recovery_s,Control_Effort,Mean_Compute_ms,Max_Tire_Util,Command_Slew_Violations,Success\n');
for s=1:numel(b.scenarios),for c=1:numel(b.controllers),m=b.runs{s,c}.metrics;fprintf(fid,'%s,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n',b.scenarios(s).name,b.estimatorMode{s},b.labels{c},m.betaRMSEdeg,m.yawRMSEdeg,m.speedRMSEkmh,m.pathRMSE,m.p95BetaErrorDeg,m.maxBetaErrorDeg,m.recoveryTime,m.controlEffort,m.meanComputeMs,m.maxTireUtil,m.constraintViolations,m.success);end,end
fclose(fid);
end
