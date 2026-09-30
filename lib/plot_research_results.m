function plot_research_results(cases,mc,analysis,cfg)
%PLOT_RESEARCH_RESULTS Publication-style controller/estimator figures.
out=cfg.outputDir; if ~isfolder(out), mkdir(out); end
if ~isfield(cfg,'saveFigures'), cfg.saveFigures=true; end
names={'PID','Fixed LQR','Gain-scheduled LQR','Constrained MPC'};
% Tracking comparison.
f=figure('Color','w','Name','Controller comparison','Position',[100 80 950 790]);
subplot(3,1,1);hold on;plot(cases{4}.t,rad2deg(cases{4}.ref.beta),'k--','LineWidth',1.4);for i=1:4,plot(cases{i}.t,rad2deg(cases{i}.beta),'LineWidth',1.0);end;grid on;ylabel('\beta (deg)');legend([{'Reference'},names],'Location','best');title('Controller comparison on nonlinear 4-wheel validation plant');
subplot(3,1,2);hold on;plot(cases{4}.t,rad2deg(cases{4}.ref.r),'k--','LineWidth',1.4);for i=1:4,plot(cases{i}.t,rad2deg(cases{i}.x(:,6)),'LineWidth',1.0);end;grid on;ylabel('Yaw rate (deg/s)');
subplot(3,1,3);hold on;for i=1:4,plot(cases{i}.t,cases{i}.x(:,4)*3.6,'LineWidth',1.0);end;plot(cases{4}.t,cases{4}.ref.vx*3.6,'k--','LineWidth',1.4);grid on;ylabel('V_x (km/h)');xlabel('Time (s)');
if cfg.saveFigures, exportgraphics(f,fullfile(out,'research_controller_comparison.png'),'Resolution',180); end

% EKF sensor-noise validation on the constrained MPC case.
s=cases{4}; f2=figure('Color','w','Name','EKF validation','Position',[120 80 930 820]);
subplot(3,1,1);plot(s.t,rad2deg(s.beta),'k','LineWidth',1.1);hold on;plot(s.t,rad2deg(atan2(s.zhat(:,2),s.zhat(:,1))),'--','LineWidth',1.1);plot(s.t,rad2deg(s.ref.beta),':','LineWidth',1.2);grid on;ylabel('\beta (deg)');legend('True','EKF','Reference');title('EKF state estimation and sensor-noise rejection');
subplot(3,1,2);plot(s.t,rad2deg(s.x(:,6)),'k','LineWidth',1.1);hold on;plot(s.t,rad2deg(s.y(:,2)),':','LineWidth',0.8);plot(s.t,rad2deg(s.zhat(:,3)),'--','LineWidth',1.1);grid on;ylabel('Yaw rate (deg/s)');legend('True','Noisy gyro','EKF');
subplot(3,1,3);plot(s.t,s.x(:,4),'k','LineWidth',1.1);hold on;plot(s.t,s.y(:,1),':','LineWidth',0.8);plot(s.t,s.zhat(:,1),'--','LineWidth',1.1);grid on;ylabel('V_x (m/s)');xlabel('Time (s)');legend('True','Noisy wheel speed','EKF');
if cfg.saveFigures, exportgraphics(f2,fullfile(out,'research_ekf_noise_validation.png'),'Resolution',180); end

% Robustness challenge / constraints.
f5=figure('Color','w','Name','Robustness challenge','Position',[140 70 940 820]);
subplot(4,1,1);plot(s.t,rad2deg(s.beta),'LineWidth',1.1);hold on;plot(s.t,rad2deg(s.ref.beta),'k--','LineWidth',1.1);grid on;ylabel('\beta (deg)');title('MPC robustness challenge: crosswind, wet road, payload mismatch and sensor noise');
subplot(4,1,2);plot(s.t,s.mu,'LineWidth',1.1);grid on;ylabel('\mu');
subplot(4,1,3);plot(s.t,rad2deg(s.u(:,1)),'LineWidth',1.1);grid on;ylabel('\delta cmd (deg)');
subplot(4,1,4);plot(s.t,100*s.u(:,2),'LineWidth',1.1);hold on;stairs(s.t,25*s.challenge,'--','LineWidth',1.0);grid on;ylabel('Throttle (%)');xlabel('Time (s)');legend('Throttle','Challenge code x25','Location','best');
if cfg.saveFigures, exportgraphics(f5,fullfile(out,'research_robustness_challenge.png'),'Resolution',180); end

% Monte-Carlo histogram.
f3=figure('Color','w','Name','Monte Carlo robustness','Position',[160 100 760 500]);histogram(mc.betaRMSEdeg,max(10,round(sqrt(numel(mc.betaRMSEdeg)))));grid on;xlabel('\beta RMSE (deg)');ylabel('Runs');title(sprintf('Gain-scheduled LQR Monte Carlo: %d runs, success %.1f%%',numel(mc.success),mc.successRate));
if cfg.saveFigures, exportgraphics(f3,fullfile(out,'research_monte_carlo.png'),'Resolution',180); end

% Closed-loop pole map.
f4=figure('Color','w','Name','Pole map','Position',[180 120 650 520]);plot(real(analysis.openLoopPoles),imag(analysis.openLoopPoles),'x','MarkerSize',10,'LineWidth',1.6);hold on;plot(real(analysis.closedLoopPoles),imag(analysis.closedLoopPoles),'o','MarkerSize',8,'LineWidth',1.6);xline(0,'k--');grid on;xlabel('Real');ylabel('Imaginary');legend('Open loop','Closed loop','Location','best');title('Linearised drift-equilibrium pole map');
if cfg.saveFigures, exportgraphics(f4,fullfile(out,'research_pole_map.png'),'Resolution',180); end

% Quantitative controller metrics and computation burden.
f6=figure('Color','w','Name','Quantitative metrics','Position',[200 100 920 700]);
B=zeros(4,4); C=zeros(4,1);for i=1:4,m=cases{i}.metrics;B(i,:)=[m.betaRMSEdeg m.yawRMSEdeg m.speedRMSEkmh m.pathRMSE];C(i)=m.meanComputeMs;end
subplot(2,1,1);bar(B);grid on;set(gca,'XTick',1:4,'XTickLabel',names);ylabel('Metric value');legend('\beta RMSE deg','Yaw RMSE deg/s','Speed RMSE km/h','Path RMSE m','Location','best');title('Tracking-performance metrics');
subplot(2,1,2);bar(C);grid on;set(gca,'XTick',1:4,'XTickLabel',names);ylabel('Mean controller computation (ms)');title('Real-time computational burden');
if cfg.saveFigures, exportgraphics(f6,fullfile(out,'research_quantitative_metrics.png'),'Resolution',180); end
end
