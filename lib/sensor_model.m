function y = sensor_model(x,aux,p,noiseOn)
%SENSOR_MODEL Wheel speed, yaw gyro, lateral accel, steering and throttle.
if nargin<4,noiseOn=true;end;n=zeros(5,1);
if noiseOn,n=[p.sigmaVx*randn;p.sigmaYaw*randn;p.sigmaAy*randn;p.sigmaSteer*randn;p.sigmaThrottle*randn];end
y=[x(4);x(6);aux.ay;x(7);x(8)]+n;
end
