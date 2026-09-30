function update_reference_hud(H,steer,throttle,speed,beta)
%UPDATE_REFERENCE_HUD Update the clean three-part reference HUD.
set(H.thText,'String',sprintf('Throttle = %d %%',round(100*throttle)));
set(H.bar,'YData',[0.17 0.17 0.17+0.64*throttle 0.17+0.64*throttle]);
aa=[0 2*pi/3 4*pi/3];
for i=1:3
    a=aa(i)+steer;
    set(H.spokes(i),'XData',[0 0.78*cos(a)],'YData',[0 0.78*sin(a)]);
end
set(H.steerDot,'XData',cos(pi/2+steer),'YData',sin(pi/2+steer));
set(H.steerText,'String',sprintf('%+.2f%s',rad2deg(steer),char(176)));
set(H.speedText,'String',sprintf('Speed = %.1f km/h',speed*3.6));
b=rad2deg(beta);
if abs(b)<0.5
    betaLabel=sprintf('beta = 0%s',char(176));
else
    betaLabel=sprintf('beta = %+.0f%s',b,char(176));
end
set(H.betaText,'String',betaLabel);
end
