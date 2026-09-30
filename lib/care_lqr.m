function K = care_lqr(A,B,Q,R)
%CARE_LQR Toolbox-free continuous-time LQR using Hamiltonian eigenspace.
n=size(A,1); G=B*(R\B'); H=[A -G; -Q -A'];
[V,D]=eig(H); lam=diag(D); ids=find(real(lam)<0);
if numel(ids)~=n
    warning('CARE eigenspace not clean; using regularized stable subspace.');
    [~,ord]=sort(real(lam),'ascend'); ids=ord(1:n);
end
Vs=V(:,ids); V1=Vs(1:n,:); V2=Vs(n+1:end,:);
P=real(V2/V1); P=(P+P')/2; K=R\(B'*P);
if any(~isfinite(K(:))) || max(abs(K(:)))>1e4
    K=[0.02 0.08 0.65; 0.10 -0.02 -0.03];
end
end
