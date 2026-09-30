function plot_paper_results(bench,ab,mc,analysis,cfg)
%PLOT_PAPER_RESULTS Research-report figures generated from actual runs.
out=cfg.outputDir;names=bench.labels;sc=bench.scenarios;
% Fig 1: baseline local drift-state stabilisation.
f=figure('Color','w','Name','Paper Fig 1','Position',[80 80 980 760]);
subplot(2,1,1);hold on;s0=bench.runs{1,4};plot(s0.t,rad2deg(s0.ref.beta),'k--','LineWidth',1.5);for c=1:4,plot(bench.runs{1,c}.t,rad2deg(bench.runs{1,c}.beta),'LineWidth',1.05);end;grid on;xlabel('Time (s)');ylabel('\beta (deg)');legend([{'Reference'},names],'Location','best');title('Baseline local drift-state stabilisation');
subplot(2,1,2);hold on;plot(s0.ref.X,s0.ref.Y,'k--','LineWidth',1.5);for c=1:4,plot(bench.runs{1,c}.x(:,1),bench.runs{1,c}.x(:,2),'LineWidth',1.05);end;axis equal;grid on;xlabel('X (m)');ylabel('Y (m)');title('Integrated position deviation (diagnostic; no outer-loop path controller)');
if cfg.saveFigures,exportgraphics(f,fullfile(out,'Fig_01_baseline_tracking.png'),'Resolution',220);end
% Fig 2: worst-case controller comparison.
f=figure('Color','w','Name','Paper Fig 2','Position',[100 80 980 760]);sidx=numel(sc);subplot(2,1,1);hold on;s=bench.runs{sidx,4};plot(s.t,rad2deg(s.ref.beta),'k--','LineWidth',1.5);for c=1:4,plot(bench.runs{sidx,c}.t,rad2deg(bench.runs{sidx,c}.beta),'LineWidth',1.05);end;grid on;ylabel('\beta (deg)');legend([{'Reference'},names],'Location','best');title('Combined disturbance scenario');
subplot(2,1,2);hold on;for c=1:4,ep=hypot(bench.runs{sidx,c}.x(:,1)-bench.runs{sidx,c}.ref.X,bench.runs{sidx,c}.x(:,2)-bench.runs{sidx,c}.ref.Y);plot(bench.runs{sidx,c}.t,ep,'LineWidth',1.05);end;grid on;xlabel('Time (s)');ylabel('Integrated position deviation (m)');
if cfg.saveFigures,exportgraphics(f,fullfile(out,'Fig_02_combined_robustness.png'),'Resolution',220);end
% Fig 3: scenario x controller beta RMSE heat map.
M=zeros(numel(sc),4);for i=1:numel(sc),for c=1:4,M(i,c)=bench.runs{i,c}.metrics.betaRMSEdeg;end,end
f=figure('Color','w','Name','Paper Fig 3','Position',[120 100 850 520]);imagesc(M);colorbar;set(gca,'XTick',1:4,'XTickLabel',names,'YTick',1:numel(sc),'YTickLabel',{sc.label});xlabel('Controller');ylabel('Scenario');title('\beta RMSE (deg): standardized scenario matrix');for i=1:size(M,1),for j=1:size(M,2),text(j,i,sprintf('%.2f',M(i,j)),'HorizontalAlignment','center','FontWeight','bold');end,end
if cfg.saveFigures,exportgraphics(f,fullfile(out,'Fig_03_scenario_matrix.png'),'Resolution',220);end
% Fig 4: estimator ablation using MPC under high sensor noise.
f=figure('Color','w','Name','Paper Fig 4','Position',[140 100 900 650]);r1=ab.runs{1};r2=ab.runs{2};r3=ab.runs{3};subplot(2,1,1);plot(r1.t,rad2deg(r1.beta-r1.ref.beta),'LineWidth',1.0);hold on;plot(r2.t,rad2deg(r2.beta-r2.ref.beta),'LineWidth',1.0);plot(r3.t,rad2deg(r3.beta-r3.ref.beta),'LineWidth',1.0);grid on;ylabel('\beta error (deg)');legend(ab.labels{1},ab.labels{2},ab.labels{3},'Location','best');title('Estimator ablation under high sensor noise');subplot(2,1,2);vals=[r1.metrics.betaRMSEdeg r1.metrics.yawRMSEdeg;r2.metrics.betaRMSEdeg r2.metrics.yawRMSEdeg;r3.metrics.betaRMSEdeg r3.metrics.yawRMSEdeg];bar(vals);grid on;set(gca,'XTick',1:3,'XTickLabel',{'Truth','Raw sensors','EKF'});legend('\beta RMSE deg','Yaw RMSE deg/s','Location','best');
if cfg.saveFigures,exportgraphics(f,fullfile(out,'Fig_04_ekf_ablation.png'),'Resolution',220);end
% Fig 5: Monte Carlo mean/std and 95th percentile.
f=figure('Color','w','Name','Paper Fig 5','Position',[160 110 900 620]);subplot(2,1,1);errorbar(1:4,mc.meanBeta,mc.stdBeta,'o','LineWidth',1.3);hold on;plot(1:4,mc.p95Beta,'s--','LineWidth',1.2);grid on;set(gca,'XTick',1:4,'XTickLabel',names);ylabel('\beta RMSE (deg)');legend('Mean +/- 1 SD','95th percentile','Location','best');title(sprintf('Matched-uncertainty Monte Carlo (%d cases/controller)',size(mc.betaRMSEdeg,1)));subplot(2,1,2);bar(mc.successRate);grid on;ylim([0 105]);set(gca,'XTick',1:4,'XTickLabel',names);ylabel('Success rate (%)');
if cfg.saveFigures,exportgraphics(f,fullfile(out,'Fig_05_monte_carlo_statistics.png'),'Resolution',220);end
% Fig 6: pole map and computation burden.
f=figure('Color','w','Name','Paper Fig 6','Position',[180 110 940 520]);subplot(1,2,1);plot(real(analysis.openLoopPoles),imag(analysis.openLoopPoles),'x','MarkerSize',10,'LineWidth',1.5);hold on;plot(real(analysis.closedLoopPoles),imag(analysis.closedLoopPoles),'o','MarkerSize',8,'LineWidth',1.5);xline(0,'k--');grid on;xlabel('Real');ylabel('Imaginary');legend('Open loop','Closed loop','Location','best');title('Linearized pole map');subplot(1,2,2);C=zeros(4,1);for c=1:4,C(c)=bench.runs{1,c}.metrics.meanComputeMs;end;bar(C);grid on;set(gca,'XTick',1:4,'XTickLabel',names);ylabel('Mean computation (ms)');title('Controller computation burden');
if cfg.saveFigures,exportgraphics(f,fullfile(out,'Fig_06_stability_computation.png'),'Resolution',220);end
end
