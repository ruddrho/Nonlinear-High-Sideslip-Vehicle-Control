function ref = reference_trajectory(t,p,trim)
%REFERENCE_TRAJECTORY Reference-matched vertical figure-8 stunt choreography.
% The path is continuous and deliberately mirrors the supplied reference:
% launch -> upper donut -> crossover -> lower donut -> crossover -> exit.
% Vehicle heading is offset from the velocity direction by beta to depict drift.

N = numel(t);
X=zeros(N,1); Y=zeros(N,1); chi=zeros(N,1); beta=zeros(N,1);
steer=zeros(N,1); throttle=zeros(N,1); mode=cell(N,1);

% Geometry chosen to reproduce the screenshot proportions.
cx=12.0; R=10.5; cyU=10.5; cyL=-10.5;

for k=1:N
    tk=t(k);
    if tk < 2.5
        % Straight launch to the figure-8 crossover.
        s=smoothstep(tk/2.5);
        X(k)=-1 + 13*s; Y(k)=0;
        chi(k)=0; beta(k)=deg2rad(-2*s);
        steer(k)=deg2rad(2*s); throttle(k)=0.55-0.18*s;
        mode{k}='LAUNCH - wheelspin';
    elseif tk < 9.5
        % Upper left donut: starts and ends at the lower tangent point.
        q=(tk-2.5)/7.0;
        a=-pi/2 + 2*pi*q;
        X(k)=cx+R*cos(a); Y(k)=cyU+R*sin(a);
        chi(k)=a+pi/2; beta(k)=deg2rad(-30-5*sin(pi*q));
        steer(k)=deg2rad(16+4*sin(pi*q)); throttle(k)=0.36+0.04*sin(pi*q).^2;
        mode{k}='LEFT DRIFT (LQR)';
    elseif tk < 10.5
        % Short crossover blend from upper to lower loop.
        q=(tk-9.5)/1.0; s=smoothstep(q);
        X(k)=cx + 0.65*sin(pi*s); Y(k)=-0.05-0.15*sin(pi*s);
        chi(k)=-(pi/2)*sin(pi*s); beta(k)=deg2rad(-30+60*s);
        steer(k)=deg2rad(16-32*s); throttle(k)=0.34;
        mode{k}='FIGURE-8 CROSSOVER';
    elseif tk < 17.5
        % Lower right donut: starts and ends at the upper tangent point.
        q=(tk-10.5)/7.0;
        a=pi/2 - 2*pi*q;
        X(k)=cx+R*cos(a); Y(k)=cyL+R*sin(a);
        chi(k)=a-pi/2; beta(k)=deg2rad(30+5*sin(pi*q));
        steer(k)=deg2rad(-16-4*sin(pi*q)); throttle(k)=0.36+0.04*sin(pi*q).^2;
        mode{k}='RIGHT DRIFT (LQR)';
    elseif tk < 18.5
        % Return through the central crossing.
        q=(tk-17.5)/1.0; s=smoothstep(q);
        X(k)=cx+0.55*sin(pi*s); Y(k)=0.05+0.10*sin(pi*s);
        chi(k)=(pi/2)*sin(pi*s); beta(k)=deg2rad(30-70*s);
        steer(k)=deg2rad(-16+44*s); throttle(k)=0.35;
        mode{k}='40 DEG DRIFT TRANSITION';
    elseif tk < 23.0
        % A deeper upper arc, matching the close-up -40 deg drift frame.
        q=(tk-18.5)/4.5;
        a=-pi/2 + 2*pi*q;
        X(k)=cx+R*cos(a); Y(k)=cyU+R*sin(a);
        chi(k)=a+pi/2; beta(k)=deg2rad(-40+5*q);
        steer(k)=deg2rad(28-8*q); throttle(k)=0.40-0.05*q;
        mode{k}='DEEP LEFT DRIFT (LQR)';
    else
        % Smooth recovery/exit to the right, finishing like the reference.
        q=min(1,(tk-23.0)/3.0); s=smoothstep(q);
        x0=cx; y0=0;
        X(k)=x0+(28.5-x0)*s;
        Y(k)=y0+( -0.8-y0)*s;
        if k>1
            chi(k)=atan2(Y(k)-Y(k-1),X(k)-X(k-1));
        else
            chi(k)=0;
        end
        beta(k)=deg2rad(-35*(1-s)); steer(k)=deg2rad(20*(1-s));
        throttle(k)=0.35*(1-s)+0.02*s; mode{k}='EXIT - recovery';
    end
end

% Velocity from geometry. Use gradient to keep it smooth and set the display
% speed close to the 28.8-29.6 km/h reference.
dt=mean(diff(t)); dX=gradient(X,dt); dY=gradient(Y,dt);
chiVel=unwrap(atan2(dY,dX));
for k=2:N-1
    if hypot(dX(k),dY(k))>1e-4, chi(k)=chiVel(k); end
end
% Keep a realistic drift body attitude: body heading = velocity heading - beta.
psi=unwrap(chi-beta);

% Display/physics speed schedule (m/s), not the geometric playback rate.
speed=(28.8/3.6)*ones(N,1);
speed(t<2.5)=linspace(2.0,28.8/3.6,sum(t<2.5))';
speed(t>=23)=28.8/3.6;
vx=speed.*cos(beta); vy=speed.*sin(beta);
r=gradient(psi,dt);

ref.X=X; ref.Y=Y; ref.psi=psi; ref.vx=vx; ref.vy=vy; ref.r=r;
ref.beta=beta; ref.steer=max(-p.maxSteer,min(p.maxSteer,steer));
ref.throttle=max(0,min(1,throttle)); ref.mode=mode;
ref.cx=cx; ref.R=R; ref.cyU=cyU; ref.cyL=cyL;
end

function y=smoothstep(x)
x=max(0,min(1,x)); y=x*x*(3-2*x);
end
