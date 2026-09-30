clear;clc;close all;
projectRoot=fileparts(mfilename('fullpath'));if isempty(projectRoot),projectRoot=pwd;end;addpath(fullfile(projectRoot,'lib'));
cfg=paper_research_config(projectRoot);p=vehicle_parameters();target.vx=29.6/3.6;target.beta=deg2rad(-30);target.radius=10;trim=drift_equilibrium(target,p);
paper=run_research_ready_suite(p,trim,cfg);save(fullfile(cfg.outputDir,'paper_validation_workspace.mat'),'paper','p','trim','cfg','-v7.3');
fprintf('\nUse the CSV tables and Fig_*.png files in outputs/paper_results for the final research report.\n');
