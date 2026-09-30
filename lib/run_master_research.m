function research = run_master_research(p,trim,cfg)
%RUN_MASTER_RESEARCH Graduate-level control engineering validation suite.
fprintf('\n=== MASTER''S CONTROL ENGINEERING VALIDATION ===\n');
% Defensive defaults for every research entry point.
if ~isfield(cfg,'saveFigures'),       cfg.saveFigures=true; end
if ~isfield(cfg,'researchDashboard'), cfg.researchDashboard=false; end
if ~isfield(cfg,'sensorNoise'),       cfg.sensorNoise=true; end
if ~isfield(cfg,'enableDisturbance'), cfg.enableDisturbance=true; end
if ~isfield(cfg,'robustnessChallenge'),cfg.robustnessChallenge=true; end
if ~isfield(cfg,'randomSeed'),        cfg.randomSeed=42; end
if ~isfield(cfg,'monteCarloRuns'),    cfg.monteCarloRuns=100; end
if ~isfield(cfg,'researchDt'),        cfg.researchDt=0.02; end
if ~isfield(cfg,'researchT'),         cfg.researchT=24.0; end
if ~isfield(cfg,'mpcHorizon'),        cfg.mpcHorizon=10; end
if ~isfield(cfg,'mpcIterations'),     cfg.mpcIterations=16; end
if ~isfield(cfg,'mpcSteerRate'),      cfg.mpcSteerRate=deg2rad(120); end
if ~isfield(cfg,'mpcThrottleRate'),   cfg.mpcThrottleRate=1.8; end
if ~isfield(cfg,'enforceCommonCommandRateLimits'), cfg.enforceCommonCommandRateLimits=true; end
if ~isfield(cfg,'mpcUseRateConstraints'), cfg.mpcUseRateConstraints=true; end
if ~isfield(cfg,'outputDir') || isempty(cfg.outputDir), cfg.outputDir=fullfile(pwd,'outputs'); end
if ~isfolder(cfg.outputDir), [ok,msg]=mkdir(cfg.outputDir); if ~ok, error('CarDrift:OutputFolder','%s',msg); end, end
cfg.trim=trim; rng(cfg.randomSeed);
sched=gain_schedule_design(p);
analysis=control_analysis(p,sched,cfg.outputDir);
controllers={'pid','fixed_lqr','scheduled_lqr','mpc'}; cases=cell(1,numel(controllers));
for i=1:numel(controllers)
    fprintf('Running %s benchmark...\n',controllers{i});
    cases{i}=simulate_research_case(controllers{i},p,sched,cfg);
end
fprintf('Running %d-case Monte Carlo robustness study...\n',cfg.monteCarloRuns);
mc=monte_carlo_robustness(p,sched,cfg);
plot_research_results(cases,mc,analysis,cfg);
write_summary(cases,mc,analysis,cfg.outputDir);
final_research_dashboard(cases,mc,analysis,cfg);
research.schedule=sched;research.analysis=analysis;research.cases=cases;research.monteCarlo=mc;
save(fullfile(cfg.outputDir,'research_validation.mat'),'research');
if cfg.researchDashboard, animate_control_dashboard(cases{4},cfg); end
fprintf('Research validation complete. Monte Carlo success rate: %.1f%%\n',mc.successRate);
end

function write_summary(cases,mc,a,out)
labels={'PID','Fixed LQR','Gain-scheduled LQR','Constrained MPC'};
fid=fopen(fullfile(out,'controller_comparison.csv'),'w');
fprintf(fid,'Controller,Beta_RMSE_deg,Yaw_RMSE_deg_s,Speed_RMSE_km_h,Path_RMSE_m,Max_Steer_deg,Control_Effort,Performance_Index_J,Recovery_s,Mean_Compute_ms,Max_Compute_ms,Success\n');
for i=1:numel(cases)
    m=cases{i}.metrics; fprintf(fid,'%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.6f,%.5f,%.5f,%.6f,%.6f,%d\n',labels{i},m.betaRMSEdeg,m.yawRMSEdeg,m.speedRMSEkmh,m.pathRMSE,m.maxSteerDeg,m.controlEffort,m.performanceIndexJ,m.recoveryTime,m.meanComputeMs,m.maxComputeMs,m.success);
end
fclose(fid);
fid=fopen(fullfile(out,'RESEARCH_SUMMARY.txt'),'w');
fprintf(fid,'GRADUATE CONTROL ENGINEERING PROJECT VALIDATION\n');
fprintf(fid,'================================================\n\n');
fprintf(fid,'Plant: nonlinear 4-wheel planar RWD model with individual Fiala tyres, friction ellipse, load transfer and actuator dynamics.\n');
fprintf(fid,'Estimator: EKF using wheel-speed, yaw-rate gyro, lateral acceleration, steering and throttle sensors.\n');
fprintf(fid,'Controllers: PID baseline, fixed LQR, gain-scheduled LQR, toolbox-free constrained linear MPC.\n');
fprintf(fid,'MPC: finite-horizon local linear prediction with steering/throttle magnitude and slew-rate constraints.\n');
fprintf(fid,'Robustness challenge: crosswind-equivalent lateral impulse, temporary wet-road friction drop, +15%% payload/model mismatch and sensor noise.\n\n');
fprintf(fid,'Controllability rank: %d/5\nObservability rank: %d/5\n',a.rankControllability,a.rankObservability);
fprintf(fid,'Monte Carlo runs: %d\nSuccess rate: %.2f%%\nMean beta RMSE: %.3f deg\n95th percentile beta RMSE: %.3f deg\nWorst beta RMSE: %.3f deg\n\n',numel(mc.success),mc.successRate,mc.meanBetaRMSE,mc.p95BetaRMSE,mc.worstBetaRMSE);
for i=1:numel(cases)
    m=cases{i}.metrics; fprintf(fid,'%s: beta RMSE %.3f deg | yaw RMSE %.3f deg/s | speed RMSE %.3f km/h | path RMSE %.3f m | effort %.3f | J %.3f | mean compute %.3f ms\n',labels{i},m.betaRMSEdeg,m.yawRMSEdeg,m.speedRMSEkmh,m.pathRMSE,m.controlEffort,m.performanceIndexJ,m.meanComputeMs);
end
m=cases{4}.metrics;
fprintf(fid,'\nEKF validation on MPC case: raw yaw RMSE %.4f deg/s -> EKF %.4f deg/s; raw speed RMSE %.4f m/s -> EKF %.4f m/s.\n',m.rawYawRMSEdeg,m.ekfYawRMSEdeg,m.rawSpeedRMSE,m.ekfSpeedRMSE);
fclose(fid);
end
