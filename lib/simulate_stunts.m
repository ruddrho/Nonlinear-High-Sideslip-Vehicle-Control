function sim = simulate_stunts(p,trim,K,cfg)
t=(0:cfg.dt:cfg.T)'; N=numel(t); x=zeros(N,6); u=zeros(N,2);
beta=zeros(N,1); speed=zeros(N,1); af=zeros(N,1); ar=zeros(N,1);
Fyf=zeros(N,1); Fyr=zeros(N,1); Fxr=zeros(N,1); names=cell(N,1);
x(1,:)=[0 0 0 trim.vx 0 0]; uprev=[0;0.40];
for k=1:N-1
    cmd=stunt_command(t(k),trim,p); names{k}=cmd.name;
    vx=x(k,4); vy=x(k,5); r=x(k,6);
    vyRef=cmd.vx*tan(cmd.beta);
    % Straight launch/recovery uses a simple speed loop so the drift-trim
    % linearisation cannot inject an artificial steering command.
    if abs(cmd.beta)<deg2rad(0.5) && abs(cmd.r)<0.02
        uc=[0; cmd.ff(2)-0.08*(vx-cmd.vx)];
    else
        e=[vx-cmd.vx; vy-vyRef; r-cmd.r];
        % Mirror the left-drift gain for positive-beta (right-drift) operation.
        if cmd.beta > deg2rad(1)
            S=diag([1 -1 -1]); T=diag([-1 1]); Kuse=T*K*S;
        else
            Kuse=K;
        end
        uc=cmd.ff-Kuse*e;
    end
    uc(1)=max(-p.maxSteer,min(p.maxSteer,uc(1)));
    uc(2)=max(0,min(1,uc(2)));
    % Steering rate and throttle lag.
    du=max(-p.maxSteerRate*cfg.dt,min(p.maxSteerRate*cfg.dt,uc(1)-uprev(1)));
    un=[uprev(1)+du; uprev(2)+(cfg.dt/p.tauThrottle)*(uc(2)-uprev(2))];
    u(k,:)=un'; uprev=un;
    xx=x(k,:)'; h=cfg.dt;
    k1=vehicle_dynamics(xx,un,p); k2=vehicle_dynamics(xx+h*k1/2,un,p);
    k3=vehicle_dynamics(xx+h*k2/2,un,p); k4=vehicle_dynamics(xx+h*k3,un,p);
    xn=xx+h*(k1+2*k2+2*k3+k4)/6;
    if xn(4)<0.5, xn(4)=0.5; end
    x(k+1,:)=xn';
    [~,a]=vehicle_dynamics(xx,un,p); beta(k)=a.beta; speed(k)=a.speed; af(k)=a.alpha_f; ar(k)=a.alpha_r;
    Fyf(k)=a.Fyf; Fyr(k)=a.Fyr; Fxr(k)=a.Fxr;
end
names{N}=stunt_command(t(N),trim,p).name; u(N,:)=u(N-1,:);
[~,a]=vehicle_dynamics(x(N,:)',u(N,:)',p); beta(N)=a.beta; speed(N)=a.speed;
sim.t=t; sim.x=x; sim.u=u; sim.beta=beta; sim.speed=speed; sim.alpha_f=af; sim.alpha_r=ar;
sim.Fyf=Fyf; sim.Fyr=Fyr; sim.Fxr=Fxr; sim.mode=names; sim.trim=trim;
end
