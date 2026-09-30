function h = draw_car(ax,x,y,psi,p,delta)
%DRAW_CAR Detailed top-view Formula-style open-wheel car for animation.
% Local +X is the front of the car.  The geometry is intentionally drawn
% with MATLAB primitives so the project has no external image dependency.
if nargin<6, delta=0; end

R=[cos(psi) -sin(psi); sin(psi) cos(psi)];
tr=@(P) R*P+[x;y];

% Visual dimensions (m). Dynamics still use the parameters in p.
L=5.15; W=2.00;
xf=1.48; xr=-1.36; yw=0.86;

% Palette.
carbon=[0.025 0.030 0.035];
carbon2=[0.055 0.060 0.068];
red=[0.82 0.025 0.035];
red2=[0.98 0.08 0.05];
metal=[0.28 0.30 0.32];
tyre=[0.018 0.020 0.022];

% --- Open wheels -------------------------------------------------------------
h.wheels=gobjects(4,1); h.rims=gobjects(4,1); h.hubs=gobjects(4,1);
axx=[xf xf xr xr]; ayy=[yw -yw yw -yw];
for i=1:4
    wa=psi; if i<=2, wa=psi+delta; end
    WR=[cos(wa) -sin(wa); sin(wa) cos(wa)];
    c=R*[axx(i);ayy(i)]+[x;y];
    % F1-like wide tyres.
    tyrePoly=[0.37 0.155;0.37 -0.155;-0.37 -0.155;-0.37 0.155]';
    tq=WR*tyrePoly+c;
    h.wheels(i)=patch(ax,tq(1,:),tq(2,:),tyre,'EdgeColor',[0 0 0],...
        'LineWidth',0.8);
    rimPoly=[0.21 0.105;0.21 -0.105;-0.21 -0.105;-0.21 0.105]';
    rq=WR*rimPoly+c;
    h.rims(i)=patch(ax,rq(1,:),rq(2,:),metal,'EdgeColor',[0.10 0.10 0.11],...
        'LineWidth',0.45);
    hc=R*[axx(i);ayy(i)]+[x;y];
    h.hubs(i)=plot(ax,hc(1),hc(2),'o','MarkerSize',3.2,...
        'MarkerFaceColor',red2,'MarkerEdgeColor',[0.35 0 0],'LineWidth',0.4);
end

% --- Suspension arms (drawn before body) ------------------------------------
h.susp=gobjects(8,1); s=1;
for sy=[-1 1]
    % Front upper/lower wishbone appearance.
    P=tr([0.82 xf; sy*0.26 sy*yw]);
    h.susp(s)=plot(ax,P(1,:),P(2,:),'Color',[0.12 0.13 0.14],'LineWidth',1.2); s=s+1;
    P=tr([1.12 xf; sy*0.12 sy*yw]);
    h.susp(s)=plot(ax,P(1,:),P(2,:),'Color',[0.12 0.13 0.14],'LineWidth',1.2); s=s+1;
    % Rear links.
    P=tr([-0.72 xr; sy*0.29 sy*yw]);
    h.susp(s)=plot(ax,P(1,:),P(2,:),'Color',[0.12 0.13 0.14],'LineWidth',1.2); s=s+1;
    P=tr([-1.05 xr; sy*0.14 sy*yw]);
    h.susp(s)=plot(ax,P(1,:),P(2,:),'Color',[0.12 0.13 0.14],'LineWidth',1.2); s=s+1;
end

% --- Front wing --------------------------------------------------------------
P=[2.55 0.98;2.55 -0.98;2.39 -0.98;2.32 -0.48;2.32 0.48;2.39 0.98]';
Q=tr(P); h.frontWing=patch(ax,Q(1,:),Q(2,:),carbon,'EdgeColor',[0.01 0.01 0.01],'LineWidth',0.9);
% Red wing tips.
for sy=[-1 1]
    P=[2.56 sy*0.98;2.56 sy*0.78;2.37 sy*0.78;2.38 sy*0.98]'; Q=tr(P);
    if sy<0, P=[2.56 sy*0.98;2.56 sy*0.78;2.37 sy*0.78;2.38 sy*0.98]'; Q=tr(P); end
    h.(['frontTip' num2str((sy+3)/2)])=patch(ax,Q(1,:),Q(2,:),red,'EdgeColor','none');
end

% --- Narrow nose / monocoque -------------------------------------------------
P=[2.42 0.13;2.42 -0.13;1.66 -0.20;1.02 -0.30;0.52 -0.34;0.20 -0.30;...
   0.20 0.30;0.52 0.34;1.02 0.30;1.66 0.20]';
Q=tr(P); h.nose=patch(ax,Q(1,:),Q(2,:),carbon2,'EdgeColor',[0.01 0.01 0.01],'LineWidth',0.8);
% Nose stripe.
P=[2.35 0.045;2.35 -0.045;0.45 -0.075;0.45 0.075]'; Q=tr(P);
h.noseStripe=patch(ax,Q(1,:),Q(2,:),red,'EdgeColor','none','FaceAlpha',0.95);

% --- Sidepods / floor --------------------------------------------------------
P=[0.60 0.33;0.38 0.62;-0.50 0.69;-1.18 0.50;-1.42 0.31;-1.18 0.23;...
   -0.35 0.30;0.28 0.27]'; Q=tr(P);
h.sidepodL=patch(ax,Q(1,:),Q(2,:),carbon,'EdgeColor',[0.01 0.01 0.01],'LineWidth',0.75);
P(2,:)=-P(2,:); Q=tr(P);
h.sidepodR=patch(ax,Q(1,:),Q(2,:),carbon,'EdgeColor',[0.01 0.01 0.01],'LineWidth',0.75);
% Red aerodynamic accent lines.
for sy=[-1 1]
    P=[0.36 -0.45 -1.16; sy*0.52 sy*0.58 sy*0.43]; Q=tr(P);
    h.(['accent' num2str((sy+3)/2)])=plot(ax,Q(1,:),Q(2,:),'Color',red2,'LineWidth',1.5);
end

% --- Cockpit, driver helmet and halo ----------------------------------------
P=[0.40 0.27;0.40 -0.27;-0.58 -0.31;-0.84 -0.19;-0.84 0.19;-0.58 0.31]';
Q=tr(P); h.cockpit=patch(ax,Q(1,:),Q(2,:),[0.010 0.012 0.016],...
    'EdgeColor',[0.16 0.17 0.18],'LineWidth',0.65);
helmet=tr([-0.12;0]);
h.helmet=plot(ax,helmet(1),helmet(2),'o','MarkerSize',7.2,'MarkerFaceColor',[0.92 0.12 0.05],...
    'MarkerEdgeColor',[0.92 0.92 0.92],'LineWidth',0.65);
% Halo: center pillar and two arms.
P=tr([0.24 -0.04;0 0]); h.halo1=plot(ax,P(1,:),P(2,:),'Color',[0.30 0.31 0.32],'LineWidth',1.7);
P=tr([-0.02 -0.46;0 0.24]); h.halo2=plot(ax,P(1,:),P(2,:),'Color',[0.30 0.31 0.32],'LineWidth',1.4);
P=tr([-0.02 -0.46;0 -0.24]); h.halo3=plot(ax,P(1,:),P(2,:),'Color',[0.30 0.31 0.32],'LineWidth',1.4);

% --- Engine cover / shark-fin silhouette ------------------------------------
P=[-0.62 0.22;-0.62 -0.22;-1.68 -0.19;-2.05 -0.12;-2.05 0.12;-1.68 0.19]';
Q=tr(P); h.engine=patch(ax,Q(1,:),Q(2,:),carbon2,'EdgeColor',[0.01 0.01 0.01],'LineWidth',0.75);
P=[-0.72 0.035;-0.72 -0.035;-1.98 -0.045;-1.98 0.045]'; Q=tr(P);
h.engineStripe=patch(ax,Q(1,:),Q(2,:),red,'EdgeColor','none');

% --- Rear diffuser and rear wing --------------------------------------------
P=[-2.05 0.38;-2.05 -0.38;-2.28 -0.52;-2.34 0.52]'; Q=tr(P);
h.diffuser=patch(ax,Q(1,:),Q(2,:),[0.015 0.017 0.020],'EdgeColor',[0 0 0],'LineWidth',0.7);
P=[-2.31 0.94;-2.31 -0.94;-2.52 -0.94;-2.52 0.94]'; Q=tr(P);
h.rearWing=patch(ax,Q(1,:),Q(2,:),carbon,'EdgeColor',[0 0 0],'LineWidth',0.95);
P=[-2.34 0.90;-2.34 -0.90;-2.40 -0.90;-2.40 0.90]'; Q=tr(P);
h.rearWingAccent=patch(ax,Q(1,:),Q(2,:),red,'EdgeColor','none');

% Exhaust outlet at the rear center; smoke is animated by animate_simulation.
ex=tr([-2.18;0]);
h.exhaust=plot(ax,ex(1),ex(2),'o','MarkerSize',3.8,'MarkerFaceColor',[0.08 0.08 0.08],...
    'MarkerEdgeColor',[0.45 0.45 0.45],'LineWidth',0.6);

% Small white CG marker retained for technical visualization.
h.cg=plot(ax,x,y,'o','MarkerSize',3.2,'MarkerFaceColor','w',...
    'MarkerEdgeColor',[0.72 0.72 0.72],'LineWidth',0.45);
end
