function ref = research_reference(t,p,trim) %#ok<INUSD>
%RESEARCH_REFERENCE Publication benchmark reference.
% The paper benchmark is intentionally a LOCAL DRIFT-STABILISATION test,
% not the Figure-8 visual demo.  LQR/MPC are local controllers designed
% around drift equilibria, so the research benchmark uses a demanding
% off-grid steady operating point inside the gain-schedule envelope.
%
% Target: 32 km/h, beta = -35 deg.  This lies between the 30/40 km/h and
% 30/40 deg schedule nodes and therefore exercises interpolation in the
% gain-scheduled LQR/MPC while keeping the experiment scientifically valid.
N=numel(t);
target.vx=32/3.6; target.beta=deg2rad(-35); target.radius=10;
tr=four_wheel_drift_equilibrium(target,p);
if ~tr.converged
    error('CarDrift:ReferenceTrim','Paper benchmark trim did not converge.');
end
ref.vx=tr.vx*ones(N,1);
ref.vy=tr.vy*ones(N,1);
ref.r=tr.r*ones(N,1);
ref.beta=tr.beta*ones(N,1);
ref.mode=repmat({'Steady drift stabilisation'},N,1);
ref.X=zeros(N,1); ref.Y=zeros(N,1); ref.psi=zeros(N,1);
dt=mean(diff(t));
for k=1:N-1
    ref.psi(k+1)=ref.psi(k)+dt*ref.r(k);
    chi=ref.psi(k)+ref.beta(k);
    ref.X(k+1)=ref.X(k)+dt*ref.vx(k)*cos(chi);
    ref.Y(k+1)=ref.Y(k)+dt*ref.vx(k)*sin(chi);
end
ref.trim=tr;
end
