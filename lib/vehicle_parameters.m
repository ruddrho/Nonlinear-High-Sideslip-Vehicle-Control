function p = vehicle_parameters()
% Representative lightweight RWD sports-car parameters.
p.m = 1280;              % kg
p.Iz = 2100;             % kg m^2
p.a = 1.15;              % CG -> front axle (m)
p.b = 1.45;              % CG -> rear axle (m)
p.L = p.a+p.b;
p.track = 1.55;
p.g = 9.81;
p.hCG = 0.48;
p.mu = 1.12;
p.Caf = 76000;           % N/rad, axle cornering stiffness
p.Car = 82000;
p.Fzf = p.m*p.g*p.b/p.L;
p.Fzr = p.m*p.g*p.a/p.L;
p.maxDrive = 12000;       % N at throttle=1
p.CdA = 0.72;            % lumped drag area
p.rho = 1.225;
p.Crr = 0.012;
p.maxSteer = deg2rad(38);
p.maxSteerRate = deg2rad(240); % rad/s
p.tauSteer = 0.075;          % steering actuator time constant (s)
p.tauThrottle = 0.12;
p.minVx = 1.0;
% Sensor standard deviations used by the EKF benchmark.
p.sigmaVx = 0.12;            % m/s wheel-speed estimate
p.sigmaYaw = deg2rad(0.35);  % rad/s yaw gyro
p.sigmaAy = 0.18;            % m/s^2 lateral accelerometer
p.sigmaSteer = deg2rad(0.15); % rad steering position sensor
p.sigmaThrottle = 0.008;      % normalized throttle position sensor
end
