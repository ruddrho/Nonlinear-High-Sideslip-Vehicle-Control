function [A,B] = numerical_linearize(x0,u0,p)
n=numel(x0); m=numel(u0); A=zeros(n); B=zeros(n,m);
for i=1:n
    h=1e-5*max(1,abs(x0(i))); xp=x0; xm=x0; xp(i)=xp(i)+h; xm(i)=xm(i)-h;
    fp=vehicle_dynamics(xp,u0,p); fm=vehicle_dynamics(xm,u0,p); A(:,i)=(fp-fm)/(2*h);
end
for j=1:m
    h=1e-5*max(1,abs(u0(j))); up=u0; um=u0; up(j)=up(j)+h; um(j)=um(j)-h;
    fp=vehicle_dynamics(x0,up,p); fm=vehicle_dynamics(x0,um,p); B(:,j)=(fp-fm)/(2*h);
end
end
