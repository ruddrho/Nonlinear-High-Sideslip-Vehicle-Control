function cmd = stunt_command(t,trim,p)
% Time-coded stunt show with smooth transitions between drift equilibria.
cmd.name=''; cmd.vx=trim.vx; cmd.beta=0; cmd.r=0; cmd.ff=[0;0.40];
if t < 2
    cmd.name='LAUNCH - wheelspin';
elseif t < 5
    q=smoothstep((t-2)/3);
    cmd.name='DRIFT ENTRY'; cmd.beta=trim.beta*q; cmd.r=abs(trim.r)*q;
    cmd.ff=[trim.delta*q; (1-q)*0.40+q*trim.throttle];
elseif t < 8
    cmd.name='LEFT DONUT (LQR)'; cmd.beta=trim.beta; cmd.r=abs(trim.r); cmd.ff=[trim.delta;trim.throttle];
elseif t < 12
    s=smoothstep((t-8)/4); q=1-2*s;
    cmd.name='FIGURE-8 TRANSITION'; cmd.beta=trim.beta*q; cmd.r=abs(trim.r)*q; cmd.ff=[trim.delta*q;trim.throttle];
elseif t < 15
    cmd.name='RIGHT DONUT (LQR)'; cmd.beta=-trim.beta; cmd.r=-abs(trim.r); cmd.ff=[-trim.delta;trim.throttle];
elseif t < 19
    s=smoothstep((t-15)/4); q=-1+2*s;
    cmd.name='FIGURE-8 RETURN'; cmd.beta=trim.beta*q; cmd.r=abs(trim.r)*q; cmd.ff=[trim.delta*q;trim.throttle];
elseif t < 21
    cmd.name='LEFT DRIFT HOLD'; cmd.beta=trim.beta; cmd.r=abs(trim.r); cmd.ff=[trim.delta;trim.throttle];
elseif t < 23
    q=1-smoothstep((t-21)/2);
    cmd.name='DRIFT SLALOM / EXIT'; cmd.beta=trim.beta*q; cmd.r=abs(trim.r)*q; cmd.ff=[trim.delta*q;0.30+q*(trim.throttle-0.30)];
else
    cmd.name='EXIT - recovery'; cmd.vx=0.85*trim.vx; cmd.beta=0; cmd.r=0; cmd.ff=[0;0.10];
end
cmd.ff(1)=max(-p.maxSteer,min(p.maxSteer,cmd.ff(1)));
end
function y=smoothstep(x)
x=max(0,min(1,x)); y=x*x*(3-2*x);
end
