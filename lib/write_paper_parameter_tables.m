function write_paper_parameter_tables(p,sched,cfg,bench)
%WRITE_PAPER_PARAMETER_TABLES Reproducible research tables.
out=cfg.outputDir;
fid=fopen(fullfile(out,'Table_I_vehicle_parameters.csv'),'w');fprintf(fid,'Parameter,Symbol,Value,Unit\n');
rows={ 'Mass','m',p.m,'kg';'Yaw inertia','Iz',p.Iz,'kg m^2';'CG to front axle','a',p.a,'m';'CG to rear axle','b',p.b,'m';'Track width','track',p.track,'m';'CG height','hCG',p.hCG,'m';'Nominal friction','mu',p.mu,'-';'Front cornering stiffness','Caf',p.Caf,'N/rad';'Rear cornering stiffness','Car',p.Car,'N/rad';'Max drive force','Fdrive,max',p.maxDrive,'N';'Max steering','delta,max',rad2deg(p.maxSteer),'deg';'Steering actuator time constant','tau_delta',p.tauSteer,'s';'Throttle actuator time constant','tau_T',p.tauThrottle,'s'};
for i=1:size(rows,1),fprintf(fid,'%s,"%s",%.8g,%s\n',rows{i,1},rows{i,2},rows{i,3},rows{i,4});end;fclose(fid);
fid=fopen(fullfile(out,'Table_II_controller_parameters.csv'),'w');fprintf(fid,'Item,Value\n');fprintf(fid,'LQR Q,"[%s]"\n',num2str(diag(sched.Q)'));fprintf(fid,'LQR R,"[%s]"\n',num2str(diag(sched.R)'));fprintf(fid,'Gain schedule speeds km/h,"[%s]"\n',num2str(sched.speeds*3.6));fprintf(fid,'Gain schedule abs beta deg,"[%s]"\n',num2str(rad2deg(sched.betas)));fprintf(fid,'MPC horizon,%d\n',cfg.mpcHorizon);fprintf(fid,'MPC projected-gradient iterations,%d\n',cfg.mpcIterations);fprintf(fid,'Steering command rate limit deg/s,%.6f\n',rad2deg(cfg.mpcSteerRate));fprintf(fid,'Throttle command rate limit 1/s,%.6f\n',cfg.mpcThrottleRate);fprintf(fid,'Research sample time s,%.6f\n',cfg.researchDt);fprintf(fid,'Random seed,%d\n',cfg.randomSeed);fclose(fid);
% Baseline computation table.
fid=fopen(fullfile(out,'Table_VI_computation_time.csv'),'w');fprintf(fid,'Controller,Mean_Compute_ms,Max_Compute_ms\n');for c=1:numel(bench.controllers),m=bench.runs{1,c}.metrics;fprintf(fid,'%s,%.8f,%.8f\n',bench.labels{c},m.meanComputeMs,m.maxComputeMs);end;fclose(fid);
end
