function print_trim(trim,target)
fprintf('Target beta       : %7.2f deg\n',rad2deg(target.beta));
fprintf('Solved beta       : %7.2f deg\n',rad2deg(trim.beta));
fprintf('Speed             : %7.2f km/h\n',trim.vx*3.6);
fprintf('Yaw rate          : %7.2f deg/s\n',rad2deg(trim.r));
fprintf('Counter-steer     : %7.2f deg\n',rad2deg(trim.delta));
fprintf('Throttle          : %7.1f %%\n',100*trim.throttle);
fprintf('Approx. radius    : %7.2f m\n',trim.radius);
fprintf('Residual norm     : %7.4g\n',norm(trim.residual));
if ~trim.converged
    fprintf('Note: exact trim is limited by the configured tyre/actuator bounds.\n');
end
end
