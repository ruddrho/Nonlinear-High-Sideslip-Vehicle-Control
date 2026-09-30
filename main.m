clear; clc; close all;

% Resolve paths from this script, not MATLAB's Current Folder.
projectRoot=fileparts(mfilename('fullpath')); if isempty(projectRoot), projectRoot=pwd; end
addpath(fullfile(projectRoot,'lib'));

cfg.dt=0.01;
cfg.T=26.0;
cfg.referenceMatch=true;     % true = screenshot-matched figure-8 choreography
cfg.makeVideo=false;         % video is generated separately by record_github_video.m
cfg.videoFPS=30;
cfg.videoQuality=95;
cfg.videoWidth=1280;       % GitHub/portfolio friendly 16:9 canvas
cfg.videoHeight=720;
cfg.videoFileName='drift_simulation_16x9.mp4';
cfg.animate=true;
cfg.animationStride=4;
cfg.tireSmoke=true;         % rear-tyre smoke during high-slip drift
cfg.exhaustSmoke=true;      % rear-center exhaust plume
cfg.smokeAttached=true;      % live cloud regenerated at current rear tyres/exhaust
cfg.smokeTrail=false;         % FALSE = smoke follows car; no smoke ring around old route
cfg.smokeDensity=1.15;        % 1.0 normal; increase for denser local smoke
cfg.saveFigures=true;
% Graduate-level control engineering validation suite.
cfg.runResearch=false;       % final research suite is generated separately by generate_paper_results.m
cfg.researchDt=0.02;
cfg.researchT=24.0;
cfg.monteCarloRuns=100;
cfg.sensorNoise=true;
cfg.enableDisturbance=true;
cfg.robustnessChallenge=true; % crosswind + wet road + payload/model mismatch
cfg.mpcHorizon=10;
cfg.mpcIterations=16;
cfg.mpcSteerRate=deg2rad(120);
cfg.mpcThrottleRate=1.8;
cfg.enforceCommonCommandRateLimits=true; % same command slew limits for all benchmark controllers
cfg.mpcUseRateConstraints=true;
cfg.researchDashboard=false; % set true for live EKF/control dashboard
cfg.randomSeed=42;
cfg.projectRoot=projectRoot;
cfg.outputDir=fullfile(projectRoot,'outputs');
if ~isfolder(cfg.outputDir), [ok,msg]=mkdir(cfg.outputDir); if ~ok, error('CarDrift:OutputFolder','%s',msg); end, end

p=vehicle_parameters();
fprintf('\n=== DRIFT EQUILIBRIUM SEARCH ===\n');
target.vx=29.6/3.6; target.beta=deg2rad(-30); target.radius=10;
trim=drift_equilibrium(target,p); print_trim(trim,target);

xtrim=[0;0;0;trim.vx;trim.vy;trim.r]; utrim=[trim.delta;trim.throttle];
[A6,B6]=numerical_linearize(xtrim,utrim,p); idx=[4 5 6]; A=A6(idx,idx); B=B6(idx,:);
Q=diag([0.7 5.0 8.0]); R=diag([2.2 0.9]); K=care_lqr(A,B,Q,R);
fprintf('\nLQR gain K:\n'); disp(K);

research=[];
if cfg.runResearch
    research=run_master_research(p,trim,cfg);
end

if cfg.referenceMatch
    sim=simulate_reference_show(p,trim,K,cfg);
else
    sim=simulate_stunts(p,trim,K,cfg);
end
plot_results(sim,p,cfg);
if cfg.animate, animate_simulation(sim,p,cfg); end
save(fullfile(cfg.outputDir,'simulation_data.mat'),'sim','p','trim','K','cfg','research');
fprintf('\nFinished. Results are in:\n%s\n',cfg.outputDir);
