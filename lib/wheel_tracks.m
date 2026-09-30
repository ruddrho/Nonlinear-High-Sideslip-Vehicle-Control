function tr = wheel_tracks(sim,p)
%WHEEL_TRACKS Compute rear-left/rear-right tyre contact trajectories.
psi=sim.x(:,3); X=sim.x(:,1); Y=sim.x(:,2);
rx=X-p.b*cos(psi); ry=Y-p.b*sin(psi);
lx=-sin(psi); ly=cos(psi);
tr.left=[rx+0.5*p.track*lx, ry+0.5*p.track*ly];
tr.right=[rx-0.5*p.track*lx, ry-0.5*p.track*ly];
end
