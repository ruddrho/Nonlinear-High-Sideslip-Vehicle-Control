clear; clc; close all;
% RECORD_SIMULATION_VIDEO
% Records only the visual Figure-8/F1 simulation. Research benchmarks are
% not re-run. Output: outputs/drift_simulation_16x9.mp4
projectRoot=fileparts(mfilename('fullpath'));
if isempty(projectRoot), projectRoot=pwd; end
addpath(fullfile(projectRoot,'lib'));

cfg.dt=0.01;
cfg.T=26.0;
cfg.referenceMatch=true;
cfg.makeVideo=true;
cfg.videoFPS=30;
cfg.videoQuality=95;
cfg.videoWidth=1280;       % GitHub/portfolio friendly 16:9 canvas
cfg.videoHeight=720;
cfg.videoFileName='drift_simulation_16x9.mp4';
cfg.animate=true;
cfg.animationStride=4;
cfg.tireSmoke=true;
cfg.exhaustSmoke=true;
cfg.smokeAttached=true;
cfg.smokeTrail=false;
cfg.smokeDensity=1.15;
cfg.saveFigures=true;
cfg.projectRoot=projectRoot;
cfg.outputDir=fullfile(projectRoot,'outputs');
if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end

p=vehicle_parameters();
target.vx=29.6/3.6; target.beta=deg2rad(-30); target.radius=10;
trim=drift_equilibrium(target,p);
xtrim=[0;0;0;trim.vx;trim.vy;trim.r];
utrim=[trim.delta;trim.throttle];
[A6,B6]=numerical_linearize(xtrim,utrim,p);
K=care_lqr(A6(4:6,4:6),B6(4:6,:),diag([0.7 5 8]),diag([2.2 0.9]));

sim=simulate_reference_show(p,trim,K,cfg);
plot_results(sim,p,cfg);
animate_simulation(sim,p,cfg);

videoFile=fullfile(cfg.outputDir,cfg.videoFileName);
fprintf('\nVideo recording complete:\n%s\n',videoFile);
