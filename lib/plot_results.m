function plot_results(sim,p,cfg)
%PLOT_RESULTS Export reference-matched trajectory and engineering plots.
out=cfg.outputDir; if ~isfolder(out), [ok,msg]=mkdir(out); if ~ok, error('%s',msg); end, end
tr=wheel_tracks(sim,p);

f1=figure('Color','w','Name','Reference Figure-8','Position',[100 80 760 820]);
ax=axes(f1); hold(ax,'on'); grid(ax,'on'); box(ax,'on'); axis(ax,'equal');
plot(ax,tr.left(:,1),tr.left(:,2),'Color',[0.50 0.50 0.50],'LineWidth',1.5);
plot(ax,tr.right(:,1),tr.right(:,2),'Color',[0.50 0.50 0.50],'LineWidth',1.5);
plot(ax,sim.x(:,1),sim.x(:,2),'k--','LineWidth',1.0);
plot(ax,12,10.5,'s','MarkerSize',7,'MarkerFaceColor',[0.95 0.35 0.03],'MarkerEdgeColor',[0.55 0.20 0.02]);
plot(ax,12,-10.5,'s','MarkerSize',7,'MarkerFaceColor',[0.95 0.35 0.03],'MarkerEdgeColor',[0.55 0.20 0.02]);
% final vehicle
car=draw_car(ax,sim.x(end,1),sim.x(end,2),sim.x(end,3),p,sim.u(end,1)); %#ok<NASGU>
xlim(ax,[-4 32]); ylim(ax,[-24 24]); xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
title(ax,'Tyre marks: launch -> left donut -> right donut -> 40 deg drift -> exit');
if cfg.saveFigures, exportgraphics(f1,fullfile(out,'trajectory.png'),'Resolution',180); end

f2=figure('Color','w','Name','Drift states');
subplot(3,1,1); plot(sim.t,rad2deg(sim.beta),'k','LineWidth',1.2); grid on; ylabel('\beta (deg)'); title('Drift state history');
yawRateDeg=rad2deg(sim.x(:,6));
yawRateDisplay=max(-120,min(120,yawRateDeg));
subplot(3,1,2); plot(sim.t,yawRateDisplay,'k','LineWidth',1.2); grid on; ylim([-120 120]); ylabel('Yaw rate (deg/s)');
text(0.99,0.06,'display clipped to +/-120 deg/s','Units','normalized','HorizontalAlignment','right','FontSize',8,'Color',[0.35 0.35 0.35]);
subplot(3,1,3); plot(sim.t,sim.speed*3.6,'k','LineWidth',1.2); grid on; ylabel('Speed (km/h)'); xlabel('Time (s)');
if cfg.saveFigures, exportgraphics(f2,fullfile(out,'states.png'),'Resolution',180); end

f3=figure('Color','w','Name','Controls');
subplot(2,1,1); plot(sim.t,rad2deg(sim.u(:,1)),'k','LineWidth',1.2); grid on; ylabel('Steer (deg)'); title('Steering / throttle commands');
subplot(2,1,2); plot(sim.t,100*sim.u(:,2),'k','LineWidth',1.2); grid on; ylabel('Throttle (%)'); xlabel('Time (s)');
if cfg.saveFigures, exportgraphics(f3,fullfile(out,'controls.png'),'Resolution',180); end

f4=figure('Color','w','Name','Tyre forces');
plot(sim.t,sim.Fyf/1000,'LineWidth',1.1); hold on; plot(sim.t,sim.Fyr/1000,'LineWidth',1.1); plot(sim.t,sim.Fxr/1000,'LineWidth',1.1); grid on;
xlabel('Time (s)'); ylabel('Force (kN)'); legend('F_y front','F_y rear','F_x rear'); title('Fiala tyre forces / friction sharing');
if cfg.saveFigures, exportgraphics(f4,fullfile(out,'tire_forces.png'),'Resolution',180); end
end
