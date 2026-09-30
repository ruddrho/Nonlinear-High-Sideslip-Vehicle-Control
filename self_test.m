clear; clc;
projectRoot=fileparts(mfilename('fullpath')); if isempty(projectRoot), projectRoot=pwd; end
addpath(fullfile(projectRoot,'lib'));
out=fullfile(projectRoot,'outputs'); if ~isfolder(out), mkdir(out); end
p=vehicle_parameters(); target.vx=29.6/3.6; target.beta=deg2rad(-30); target.radius=10;
trim=drift_equilibrium(target,p); assert(all(isfinite([trim.vx trim.vy trim.r trim.delta trim.throttle])));
t=(0:0.01:26)'; ref=reference_trajectory(t,p,trim); assert(numel(ref.X)==numel(t)); assert(all(isfinite([ref.X;ref.Y;ref.psi;ref.vx;ref.vy;ref.r])));
% Four-wheel plant smoke test.
x=[0;0;0;8;0;0;0;0.35]; [dx,a]=four_wheel_vehicle_dynamics(x,[0;0.35],p); assert(all(isfinite(dx))); assert(numel(a.utilization)==4);
% Gain schedule / controllability / EKF smoke test.
sched=gain_schedule_design(p); [Kg,ff]=scheduled_lqr_gain(8,deg2rad(-30),sched); assert(all(isfinite(Kg(:))) && all(isfinite(ff(:))));

% Scheduled linear model + constrained MPC smoke test.
[Am,Bm,ffm,eqm]=scheduled_linear_model(8,deg2rad(-30),sched); assert(all(isfinite(Am(:))) && all(isfinite(Bm(:))) && all(isfinite(ffm(:))) && all(isfinite(eqm(:))));
cfgm.mpcHorizon=6;cfgm.mpcIterations=4;cfgm.mpcSteerRate=deg2rad(120);cfgm.mpcThrottleRate=1.8;
zh=[8;0;0;0;0.35];
[um,im]=constrained_mpc_controller(zh,[8;eqm(2);eqm(3)],8,deg2rad(-30),sched,p,0.02,[0;0.35],cfgm); assert(all(isfinite(um)) && isfinite(im.cost));
P=eye(5); y=[8;0;0;0;0.35]; [zn,Pn]=ekf_step(zh,P,[0;0.35],y,0.02,p); assert(all(isfinite(zn)) && all(isfinite(Pn(:))));
% Research-ready paper-suite plumbing smoke test.
sc=research_scenarios();assert(numel(sc)==6 && strcmp(sc(6).name,'combined'));
pcfg=paper_research_config(projectRoot);pcfg.researchT=3.0;pcfg.researchDt=0.04;pcfg.trim=trim;pcfg.scenario=sc(1);pcfg.caseSeed=123;pcfg.sensorNoise=false;pcfg.estimatorMode='ekf';pcfg.enforceCommonCommandRateLimits=false;
sp=simulate_research_case('scheduled_lqr',p,sched,pcfg);assert(all(isfinite(sp.x(:))) && isfield(sp.metrics,'constraintViolations'));
fprintf('RESEARCH-READY SUITE SMOKE TEST PASSED\n');

fid=fopen(fullfile(out,'SELF_TEST_OK.txt'),'w'); fprintf(fid,'Reference visuals, 4-wheel plant, gain schedule, constrained MPC, EKF and research-ready suite smoke tests passed.\n'); fclose(fid);
fprintf('SELF TEST PASSED - INCLUDING RESEARCH-READY SUITE\n');
