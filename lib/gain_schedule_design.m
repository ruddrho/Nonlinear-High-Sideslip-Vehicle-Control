function sched = gain_schedule_design(p)
%GAIN_SCHEDULE_DESIGN LQR bank over the validated high-sideslip drift envelope.
% The research benchmark operates at 32 km/h and |beta|=35 deg.  The
% schedule therefore uses only converged drift trims surrounding that point;
% low-beta nodes from earlier versions were on a different equilibrium branch.
speeds=[30 35 40]/3.6; betas=deg2rad([30 35 40]);
Q=diag([1.0 4.0 8.0 0.8 0.4]); R=diag([1.2 0.5]);
ns=numel(speeds);nb=numel(betas);K=zeros(2,5,ns,nb);Agrid=zeros(5,5,ns,nb);Bgrid=zeros(5,2,ns,nb);ff=zeros(2,ns,nb);eq=zeros(5,ns,nb);conv=false(ns,nb);
for i=1:ns
    for j=1:nb
        target.vx=speeds(i);target.beta=-betas(j);target.radius=10;tr=four_wheel_drift_equilibrium(target,p);
        if ~tr.converged
            error('CarDrift:GainScheduleTrim','Gain-schedule trim failed at %.1f km/h, beta=%.1f deg.',speeds(i)*3.6,-rad2deg(betas(j)));
        end
        x0=[0;0;0;tr.vx;tr.vy;tr.r;tr.delta;tr.throttle];u0=[tr.delta;tr.throttle];
        [A8,B8]=numerical_linearize_fourwheel(x0,u0,p);idx=4:8;Agrid(:,:,i,j)=A8(idx,idx);Bgrid(:,:,i,j)=B8(idx,:);K(:,:,i,j)=care_lqr(Agrid(:,:,i,j),Bgrid(:,:,i,j),Q,R);
        ff(:,i,j)=u0;eq(:,i,j)=[tr.vx;tr.vy;tr.r;tr.delta;tr.throttle];conv(i,j)=tr.converged;
    end
end
sched.speeds=speeds;sched.betas=betas;sched.K=K;sched.A=Agrid;sched.B=Bgrid;sched.ff=ff;sched.eq=eq;sched.Q=Q;sched.R=R;sched.converged=conv;
[~,is]=min(abs(speeds-30/3.6));[~,ib]=min(abs(betas-deg2rad(30)));sched.Kfixed=K(:,:,is,ib);sched.fffixed=ff(:,is,ib);sched.eqfixed=eq(:,is,ib);
sched.centralTrim.vx=eq(1,is,ib);sched.centralTrim.vy=eq(2,is,ib);sched.centralTrim.r=eq(3,is,ib);sched.centralTrim.delta=eq(4,is,ib);sched.centralTrim.throttle=eq(5,is,ib);sched.centralTrim.beta=-betas(ib);
end
