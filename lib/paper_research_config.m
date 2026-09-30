function cfg = paper_research_config(projectRoot)
%PAPER_RESEARCH_CONFIG Reproducible defaults for publication experiments.
if nargin<1 || isempty(projectRoot), projectRoot=pwd; end
cfg.projectRoot=projectRoot;
cfg.outputDir=fullfile(projectRoot,'outputs','paper_results_v3');
cfg.researchDt=0.02; cfg.researchT=24.0;
cfg.sensorNoise=true; cfg.enableDisturbance=true; cfg.robustnessChallenge=true;
cfg.saveFigures=true; cfg.researchDashboard=false; cfg.randomSeed=20260930;
cfg.mpcHorizon=10; cfg.mpcIterations=16; cfg.mpcSteerRate=deg2rad(120); cfg.mpcThrottleRate=1.8;
% The plant already enforces physical steering-rate and actuator dynamics.
% Do not add a second command-slew limiter to PID/LQR; MPC handles its own
% command-rate constraints explicitly.
cfg.enforceCommonCommandRateLimits=false;
cfg.estimatorMode='ekf';
% 100 is publication-oriented. Set lower while debugging, then restore 100.
cfg.paperMonteCarloRuns=100; cfg.monteCarloDt=0.04;
cfg.benchmarkEstimatorMode='truth';
cfg.outputFeedbackEstimatorMode='ekf';
cfg.paperControllers={'pid','fixed_lqr','scheduled_lqr','mpc'};
cfg.paperControllerLabels={'PID','Fixed LQR','Gain-Scheduled LQR','Constrained MPC'};
if ~isfolder(cfg.outputDir), [ok,msg]=mkdir(cfg.outputDir); if ~ok,error('CarDrift:OutputFolder','%s',msg);end,end
end
