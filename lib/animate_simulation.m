function animate_simulation(sim,p,cfg)
%ANIMATE_SIMULATION Reference-style animation with F1 model and visible smoke.
% Smoke follows the car by default.
% The live cloud is regenerated every frame at the current rear tyres/exhaust,
% so old smoke does NOT remain around the full drift route. An optional very
% short world-space trail can be enabled with cfg.smokeTrail=true.
if ~isfield(cfg,'videoWidth'), cfg.videoWidth=1280; end
if ~isfield(cfg,'videoHeight'), cfg.videoHeight=720; end
fig=figure('Color','w','Name','F1 Drift Control & Figure-8 - MATLAB',...
    'Position',[60 60 cfg.videoWidth cfg.videoHeight],'Resize','off',...
    'ToolBar','none','MenuBar','none');

% Defensive defaults keep older launch scripts compatible.
if ~isfield(cfg,'tireSmoke'), cfg.tireSmoke=true; end
if ~isfield(cfg,'exhaustSmoke'), cfg.exhaustSmoke=true; end
if ~isfield(cfg,'smokeAttached'), cfg.smokeAttached=true; end
if ~isfield(cfg,'smokeTrail'), cfg.smokeTrail=false; end
if ~isfield(cfg,'smokeDensity'), cfg.smokeDensity=1.0; end
if ~isfield(cfg,'saveFigures'), cfg.saveFigures=true; end
if ~isfield(cfg,'makeVideo'), cfg.makeVideo=false; end
if ~isfield(cfg,'videoFPS'), cfg.videoFPS=30; end
if ~isfield(cfg,'videoQuality'), cfg.videoQuality=95; end
if ~isfield(cfg,'videoFileName'), cfg.videoFileName='drift_simulation.mp4'; end
if ~isfield(cfg,'animationStride'), cfg.animationStride=4; end

ax=axes(fig,'Position',[0.055 0.085 0.585 0.845]);
% Prevent MATLAB's axes interaction toolbar from appearing in captured frames.
try, ax.Toolbar.Visible='off'; catch, end
try, disableDefaultInteractivity(ax); catch, end
hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
if isfield(sim,'referenceMatch') && sim.referenceMatch
    title(ax,'The tyre marks draw a FIGURE-8','FontSize',15,'FontWeight','bold','Units','normalized','Position',[0.5 1.012 0]);
else
    title(ax,'F1 DRIFT CONTROL in MATLAB','FontSize',15,'FontWeight','bold','Units','normalized','Position',[0.5 1.012 0]);
end
xlim(ax,[-4 32]); ylim(ax,[-24 24]);

tr=wheel_tracks(sim,p);
plot(ax,sim.x(:,1),sim.x(:,2),'--','LineWidth',0.9,'Color',[0.25 0.25 0.25]);
leftLine=plot(ax,nan,nan,'-','LineWidth',1.55,'Color',[0.34 0.34 0.34]);
rightLine=plot(ax,nan,nan,'-','LineWidth',1.55,'Color',[0.34 0.34 0.34]);
plot(ax,12,10.5,'s','MarkerSize',7,'MarkerFaceColor',[0.95 0.35 0.03],'MarkerEdgeColor',[0.55 0.20 0.02]);
plot(ax,12,-10.5,'s','MarkerSize',7,'MarkerFaceColor',[0.95 0.35 0.03],'MarkerEdgeColor',[0.55 0.20 0.02]);

H=draw_reference_hud(fig,sim.u(1,1),sim.u(1,2),sim.speed(1),sim.beta(1));
modeText=text(ax,0.012,0.988,'','Units','normalized','FontSize',8.5,...
    'FontWeight','normal','VerticalAlignment','top','HorizontalAlignment','left',...
    'BackgroundColor',[1 1 1 0.82],'Margin',2);

% Optional SHORT smoke trails. They are disabled by default so smoke moves
% with the car instead of painting the entire figure-8 route.
tireSmokeTrail=scatter(ax,nan,nan,30,[0.67 0.67 0.67],'filled',...
    'MarkerFaceAlpha',0.16,'MarkerEdgeAlpha',0.025);
exhaustSmokeTrail=scatter(ax,nan,nan,22,[0.58 0.58 0.58],'filled',...
    'MarkerFaceAlpha',0.13,'MarkerEdgeAlpha',0.020);

% Dense current smoke cloud. These particles are re-positioned every frame
% relative to the car, therefore the smoke is always visibly connected to it.
liveSmokeOuter=scatter(ax,nan,nan,60,[0.78 0.78 0.78],'filled',...
    'MarkerFaceAlpha',0.22,'MarkerEdgeAlpha',0.015);
liveSmokeCore=scatter(ax,nan,nan,36,[0.54 0.54 0.54],'filled',...
    'MarkerFaceAlpha',0.19,'MarkerEdgeAlpha',0.012);
liveExhaust=scatter(ax,nan,nan,30,[0.48 0.48 0.48],'filled',...
    'MarkerFaceAlpha',0.18,'MarkerEdgeAlpha',0.012);

smx=[]; smy=[]; sms=[];
exx=[]; exy=[]; exs=[];
car=[];

video=[];
if cfg.makeVideo
    out=cfg.outputDir; if ~isfolder(out), mkdir(out); end
    video=VideoWriter(fullfile(out,cfg.videoFileName),'MPEG-4');
    video.FrameRate=cfg.videoFPS;
    if isprop(video,'Quality'), video.Quality=cfg.videoQuality; end
    open(video);
end

% Stable visual noise across repeated runs.
rng(11,'twister');
for k=1:cfg.animationStride:numel(sim.t)
    if ~isempty(car), delete_car(car); end

    psi=sim.x(k,3); throttle=max(0,min(1,sim.u(k,2)));
    betaAbs=abs(rad2deg(sim.beta(k))); speedK=sim.speed(k);
    fwd=[cos(psi) sin(psi)]; lat=[-sin(psi) cos(psi)];

    % Rear wheel locations used for local smoke. wheel_tracks() gives the
    % two rear contact traces used by the skid marks.
    rearL=tr.left(k,:); rearR=tr.right(k,:);

    % Drift intensity controls density/size. A small minimum keeps a visible
    % connection to the car during drift entry/exit rather than abruptly
    % switching the cloud off.
    driftIntensity=min(1,max(0,(betaAbs-4)/24));
    powerIntensity=min(1,max(0,(throttle-0.08)/0.62));
    smokeIntensity=max(driftIntensity,0.55*powerIntensity);

    % --- Persistent rear-tyre smoke trail ---------------------------------
    if cfg.smokeTrail && cfg.tireSmoke && speedK>3 && (betaAbs>6 || throttle>0.16)
        nEmit=max(2,round(cfg.smokeDensity*(2+4*smokeIntensity)));
        for q=1:nEmit
            base=rearL; if mod(q,2)==0, base=rearR; end
            trail=(0.12+0.80*rand)*fwd;
            jitter=(0.04+0.22*randn)*lat + 0.08*randn*fwd;
            pt=base-trail+jitter;
            smx=[smx pt(1)]; smy=[smy pt(2)]; %#ok<AGROW>
            sms=[sms 28+78*smokeIntensity+58*rand]; %#ok<AGROW>
        end
        if numel(smx)>90
            smx=smx(end-89:end); smy=smy(end-89:end); sms=sms(end-89:end);
        end
        set(tireSmokeTrail,'XData',smx,'YData',smy,'SizeData',sms);
    end

    % --- Smoke cloud physically attached to rear tyres ---------------------
    if cfg.smokeAttached && cfg.tireSmoke && speedK>3 && (betaAbs>5 || throttle>0.12)
        nLocal=max(20,round(cfg.smokeDensity*(24+28*smokeIntensity)));
        lx=zeros(1,nLocal); ly=zeros(1,nLocal); ls=zeros(1,nLocal);
        cx=zeros(1,nLocal); cy=zeros(1,nLocal); cs=zeros(1,nLocal);
        for q=1:nLocal
            base=rearL; if mod(q,2)==0, base=rearR; end
            % Outer cloud expands rearward and sideways from the tyre.
            d=(0.03+1.20*rand)*fwd;
            side=(0.04+0.26*randn)*lat;
            pt=base-d+side+0.035*randn*fwd;
            lx(q)=pt(1); ly(q)=pt(2); ls(q)=48+115*smokeIntensity+80*rand;

            % Darker compact core very close to the tyre contact patch.
            d2=(0.01+0.34*rand)*fwd;
            pt2=base-d2+(0.025+0.075*randn)*lat;
            cx(q)=pt2(1); cy(q)=pt2(2); cs(q)=20+55*smokeIntensity+30*rand;
        end
        set(liveSmokeOuter,'XData',lx,'YData',ly,'SizeData',ls);
        set(liveSmokeCore,'XData',cx,'YData',cy,'SizeData',cs);
    else
        set(liveSmokeOuter,'XData',nan,'YData',nan,'SizeData',60);
        set(liveSmokeCore,'XData',nan,'YData',nan,'SizeData',36);
    end

    % --- Rear-center exhaust plume -----------------------------------------
    % A faint plume remains attached to the car whenever it is moving.  At
    % higher throttle it becomes larger and leaves a persistent trail.
    rear=[sim.x(k,1) sim.x(k,2)]-2.22*fwd;
    if cfg.exhaustSmoke && speedK>2
        exhaustIntensity=max(0.16,powerIntensity);
        nEx=max(7,round(cfg.smokeDensity*(7+12*exhaustIntensity)));
        qx=zeros(1,nEx); qy=zeros(1,nEx); qs=zeros(1,nEx);
        for q=1:nEx
            d=(0.04+1.05*rand)*fwd;
            side=(0.015+0.095*randn)*lat;
            pt=rear-d+side+0.025*randn*fwd;
            qx(q)=pt(1); qy(q)=pt(2); qs(q)=18+48*exhaustIntensity+38*rand;
        end
        set(liveExhaust,'XData',qx,'YData',qy,'SizeData',qs);

        if cfg.smokeTrail && throttle>0.18
            nPart=1+round(2*powerIntensity);
            for q=1:nPart
                trail=(0.12+0.85*rand)*fwd;
                jitter=(0.02+0.07*randn)*lat+0.045*randn*fwd;
                pt=rear-trail+jitter;
                exx=[exx pt(1)]; exy=[exy pt(2)]; %#ok<AGROW>
                exs=[exs 14+38*powerIntensity+30*rand]; %#ok<AGROW>
            end
            if numel(exx)>45
                exx=exx(end-44:end); exy=exy(end-44:end); exs=exs(end-44:end);
            end
            set(exhaustSmokeTrail,'XData',exx,'YData',exy,'SizeData',exs);
        end
    else
        set(liveExhaust,'XData',nan,'YData',nan,'SizeData',30);
    end

    % Draw the car after smoke so the vehicle remains crisp in the foreground.
    car=draw_car(ax,sim.x(k,1),sim.x(k,2),psi,p,sim.u(k,1));
    set(leftLine,'XData',tr.left(1:k,1),'YData',tr.left(1:k,2));
    set(rightLine,'XData',tr.right(1:k,1),'YData',tr.right(1:k,2));

    update_reference_hud(H,sim.u(k,1),sim.u(k,2),speedK,sim.beta(k));
    set(modeText,'String',sprintf('t = %5.2f s   |   %s',sim.t(k),sim.mode{k}));
    drawnow limitrate;
    if ~isempty(video), writeVideo(video,getframe(fig)); end
end
if ~isempty(video), close(video); end
if cfg.saveFigures
    if ~isfolder(cfg.outputDir), mkdir(cfg.outputDir); end
    exportgraphics(fig,fullfile(cfg.outputDir,'reference_final_frame.png'),'Resolution',180);
end
end

function delete_car(h)
c=struct2cell(h);
for i=1:numel(c)
    q=c{i};
    try
        q=q(isgraphics(q));
        if ~isempty(q), delete(q); end
    catch
    end
end
end
