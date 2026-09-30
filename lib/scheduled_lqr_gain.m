function [K,ff,eq] = scheduled_lqr_gain(vx,beta,sched)
%SCHEDULED_LQR_GAIN Bilinear interpolation in speed and |sideslip|.
[s0,s1,ws]=bracket(sched.speeds,max(min(vx,max(sched.speeds)),min(sched.speeds)));
[b0,b1,wb]=bracket(sched.betas,max(min(abs(beta),max(sched.betas)),min(sched.betas)));
K=blend(sched.K(:,:,s0,b0),sched.K(:,:,s1,b0),sched.K(:,:,s0,b1),sched.K(:,:,s1,b1),ws,wb);
ff=blend(sched.ff(:,s0,b0),sched.ff(:,s1,b0),sched.ff(:,s0,b1),sched.ff(:,s1,b1),ws,wb);
eq=blend(sched.eq(:,s0,b0),sched.eq(:,s1,b0),sched.eq(:,s0,b1),sched.eq(:,s1,b1),ws,wb);
if beta>0
    S=diag([1 -1 -1 -1 1]);T=diag([-1 1]);K=T*K*S;ff(1)=-ff(1);eq([2 3 4])=-eq([2 3 4]);
end
end
function y=blend(y00,y10,y01,y11,ws,wb),y=(1-ws)*(1-wb)*y00+ws*(1-wb)*y10+(1-ws)*wb*y01+ws*wb*y11;end
function [i0,i1,w]=bracket(v,x),if x<=v(1),i0=1;i1=1;w=0;return;end;if x>=v(end),i0=numel(v);i1=i0;w=0;return;end;i1=find(v>=x,1,'first');i0=i1-1;w=(x-v(i0))/(v(i1)-v(i0));end
