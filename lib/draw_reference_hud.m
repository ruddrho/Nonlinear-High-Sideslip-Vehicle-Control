function H = draw_reference_hud(fig,steer,throttle,speed,beta)
%DRAW_REFERENCE_HUD Clean reference-style HUD with non-overlapping readouts.
% Separate axes are used for the pedal, steering wheel and text so the
% steering wheel stays circular instead of stretching with the figure.

% --- Layout ------------------------------------------------------------------
% For GitHub/portfolio video the recorder uses a 16:9 canvas.  In that mode
% the HUD occupies the right column, leaving the full vertical Figure-8 clear.
fp=get(fig,'Position'); isWide=(fp(3)/max(fp(4),1))>1.45;
if isWide
    pedalPos=[0.665 0.655 0.315 0.285];
    wheelPos=[0.685 0.345 0.270 0.285];
    textPos =[0.690 0.075 0.270 0.245];
else
    pedalPos=[0.075 0.805 0.245 0.165];
    wheelPos=[0.335 0.805 0.255 0.165];
    textPos =[0.625 0.805 0.30 0.165];
end

% --- Pedal / throttle section ------------------------------------------------
H.axPedal=axes(fig,'Position',pedalPos,'Color','w',...
    'XLim',[0 1],'YLim',[0 1],'XTick',[],'YTick',[],'Box','off');
hold(H.axPedal,'on'); axis(H.axPedal,'manual');

% Pedal and floor line.
plot(H.axPedal,[0.34 0.50],[0.17 0.79],'Color',[0.04 0.20 0.78],'LineWidth',4.5);
plot(H.axPedal,[0.47 0.67],[0.79 0.73],'Color',[0.04 0.20 0.78],'LineWidth',7.5);
plot(H.axPedal,[0.23 0.62],[0.15 0.15],'k','LineWidth',2.2);

% Throttle readout. Double slash is intentional: sprintf must preserve TeX.
H.thText=text(H.axPedal,0.02,0.54,sprintf('Throttle = %d %%',round(100*throttle)),...
    'Interpreter','none','FontSize',12,'FontWeight','bold','VerticalAlignment','middle');

% Slim throttle bar.
rectangle(H.axPedal,'Position',[0.79 0.16 0.045 0.67],...
    'EdgeColor',[0.55 0.55 0.55],'LineWidth',1.2,'FaceColor',[0.98 0.98 0.98]);
H.bar=patch(H.axPedal,[0.795 0.825 0.825 0.795],...
    [0.17 0.17 0.17+0.64*throttle 0.17+0.64*throttle],...
    [0.82 0.08 0.12],'EdgeColor','none');

% --- Steering-wheel section --------------------------------------------------
H.axWheel=axes(fig,'Position',wheelPos,'Color','w',...
    'XLim',[-1.12 1.12],'YLim',[-1.12 1.12],'XTick',[],'YTick',[],'Box','off');
hold(H.axWheel,'on'); axis(H.axWheel,'equal'); axis(H.axWheel,'manual');
th=linspace(0,2*pi,220);
plot(H.axWheel,cos(th),sin(th),'k','LineWidth',2.6);
plot(H.axWheel,0.84*cos(th),0.84*sin(th),'Color',[0.58 0.58 0.58],'LineWidth',1.2);
H.spokes=gobjects(3,1); aa=[0 2*pi/3 4*pi/3];
for i=1:3
    a=aa(i)+steer;
    H.spokes(i)=plot(H.axWheel,[0 0.78*cos(a)],[0 0.78*sin(a)],'k','LineWidth',2.0);
end
plot(H.axWheel,0,0,'o','MarkerSize',10,'MarkerFaceColor','w',...
    'MarkerEdgeColor','k','LineWidth',1.5);
H.steerDot=plot(H.axWheel,cos(pi/2+steer),sin(pi/2+steer),'s',...
    'MarkerSize',6,'MarkerFaceColor',[0.90 0.04 0.06],'MarkerEdgeColor',[0.90 0.04 0.06]);

% --- Readout section ---------------------------------------------------------
H.axText=axes(fig,'Position',textPos,'Color','w',...
    'XLim',[0 1],'YLim',[0 1],'XTick',[],'YTick',[],'Box','off');
hold(H.axText,'on'); axis(H.axText,'manual');
H.steerText=text(H.axText,0.05,0.77,angle_string(steer,2),...
    'Interpreter','none','FontSize',13,'FontWeight','bold');
H.speedText=text(H.axText,0.05,0.49,sprintf('Speed = %.1f km/h',speed*3.6),...
    'Interpreter','none','FontSize',13,'FontWeight','bold');
H.betaText=text(H.axText,0.05,0.21,beta_string(beta),...
    'Interpreter','none','FontSize',13,'FontWeight','bold');
end

function s=angle_string(a,n)
% Use a real degree glyph so TeX escape sequences cannot leak into the HUD.
if n==2
    s=sprintf('%+.2f%s',rad2deg(a),char(176));
else
    s=sprintf('%+.0f%s',rad2deg(a),char(176));
end
end

function s=beta_string(beta)
b=rad2deg(beta);
if abs(b)<0.5
    s=sprintf('beta = 0%s',char(176));
else
    s=sprintf('beta = %+.0f%s',b,char(176));
end
end
