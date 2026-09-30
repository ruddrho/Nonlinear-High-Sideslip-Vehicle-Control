function analysis = control_analysis(p,sched,outdir)
%CONTROL_ANALYSIS Controllability, observability and closed-loop pole report.
target.vx=30/3.6;target.beta=deg2rad(-30);target.radius=10;tr=four_wheel_drift_equilibrium(target,p);
x0=[0;0;0;tr.vx;tr.vy;tr.r;tr.delta;tr.throttle];u0=[tr.delta;tr.throttle];[A8,B8]=numerical_linearize_fourwheel(x0,u0,p);
idx=4:8;A=A8(idx,idx);B=B8(idx,:);K=sched.Kfixed;
Co=B;Ap=eye(5);for k=1:4,Ap=Ap*A;Co=[Co Ap*B];end %#ok<AGROW>
h=@(z) meas(z,u0,p);C=numjac(h,[tr.vx;tr.vy;tr.r;tr.delta;tr.throttle]);Ob=C;Ap=eye(5);for k=1:4,Ap=Ap*A;Ob=[Ob;C*Ap];end %#ok<AGROW>
analysis.A=A;analysis.B=B;analysis.C=C;analysis.rankControllability=rank(Co);analysis.rankObservability=rank(Ob);
analysis.openLoopPoles=eig(A);analysis.closedLoopPoles=eig(A-B*K);analysis.trim=tr;
fid=fopen(fullfile(outdir,'control_analysis.txt'),'w');
fprintf(fid,'CONTROL SYSTEM ANALYSIS - 4-WHEEL PLANT, 30 km/h, beta=-30 deg\n\n');
fprintf(fid,'Augmented state: [vx vy r deltaAct throttleAct]\n');
fprintf(fid,'Controllability rank: %d / 5\n',analysis.rankControllability);fprintf(fid,'Observability rank:   %d / 5\n\n',analysis.rankObservability);
fprintf(fid,'A matrix:\n');for i=1:5,fprintf(fid,'% .7g % .7g % .7g % .7g % .7g\n',A(i,:));end
fprintf(fid,'\nB matrix:\n');for i=1:5,fprintf(fid,'% .7g % .7g\n',B(i,:));end
fprintf(fid,'\nClosed-loop poles:\n');for i=1:numel(analysis.closedLoopPoles),q=analysis.closedLoopPoles(i);fprintf(fid,'% .6g %+.6gi\n',real(q),imag(q));end
fclose(fid);
end
function y=meas(z,u,p),x=[0;0;0;z(1);z(2);z(3);z(4);z(5)];dx=four_wheel_vehicle_dynamics(x,u,p);y=[z(1);z(3);dx(5)+z(1)*z(3);z(4);z(5)];end
function J=numjac(fun,x),y=fun(x);J=zeros(numel(y),numel(x));for i=1:numel(x),h=1e-5*max(1,abs(x(i)));xp=x;xm=x;xp(i)=xp(i)+h;xm(i)=xm(i)-h;J(:,i)=(fun(xp)-fun(xm))/(2*h);end,end
