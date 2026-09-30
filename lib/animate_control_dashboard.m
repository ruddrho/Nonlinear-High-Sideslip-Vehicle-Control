function animate_control_dashboard(sim,cfg)
%ANIMATE_CONTROL_DASHBOARD Optional live control/estimation dashboard.
fig=figure('Color','w','Name','Control Engineering Dashboard','Position',[60 50 1100 760]);
ax1=subplot(2,2,1);hold(ax1,'on');grid(ax1,'on');axis(ax1,'equal');xlabel(ax1,'X (m)');ylabel(ax1,'Y (m)');title(ax1,'4-wheel plant trajectory');plot(ax1,sim.ref.X,sim.ref.Y,'k--');hPath=plot(ax1,nan,nan,'LineWidth',1.4);
ax2=subplot(2,2,2);hold(ax2,'on');grid(ax2,'on');title(ax2,'Sideslip: reference / true / EKF');xlabel(ax2,'Time (s)');ylabel(ax2,'\beta (deg)');plot(ax2,sim.t,rad2deg(sim.ref.beta),'k--');hB=plot(ax2,nan,nan,'LineWidth',1.2);hE=plot(ax2,nan,nan,':','LineWidth',1.2);
ax3=subplot(2,2,3);hold(ax3,'on');grid(ax3,'on');title(ax3,'Actuator commands');xlabel(ax3,'Time (s)');ylabel(ax3,'Steer (deg) / throttle (%)');hS=plot(ax3,nan,nan,'LineWidth',1.1);hT=plot(ax3,nan,nan,'LineWidth',1.1);
ax4=subplot(2,2,4);hold(ax4,'on');grid(ax4,'on');title(ax4,'Tyre utilisation and friction');xlabel(ax4,'Time (s)');ylabel(ax4,'Utilisation / \mu');hU=plot(ax4,nan,nan,'LineWidth',1.1);hM=plot(ax4,nan,nan,'--','LineWidth',1.1);yline(ax4,1,'k:');
stride=max(1,round(0.08/mean(diff(sim.t))));
for k=1:stride:numel(sim.t)
    set(hPath,'XData',sim.x(1:k,1),'YData',sim.x(1:k,2));
    set(hB,'XData',sim.t(1:k),'YData',rad2deg(sim.beta(1:k)));set(hE,'XData',sim.t(1:k),'YData',rad2deg(atan2(sim.zhat(1:k,2),sim.zhat(1:k,1))));
    set(hS,'XData',sim.t(1:k),'YData',rad2deg(sim.u(1:k,1)));set(hT,'XData',sim.t(1:k),'YData',100*sim.u(1:k,2));
    set(hU,'XData',sim.t(1:k),'YData',max(sim.utilization(1:k,:),[],2));set(hM,'XData',sim.t(1:k),'YData',sim.mu(1:k));drawnow limitrate;
end
if ~isfield(cfg,'saveFigures') || cfg.saveFigures
    if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end
    exportgraphics(fig,fullfile(cfg.outputDir,'research_dashboard_final.png'),'Resolution',180);
end
end
