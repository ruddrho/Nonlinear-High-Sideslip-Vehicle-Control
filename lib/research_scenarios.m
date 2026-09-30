function scenarios = research_scenarios()
%RESEARCH_SCENARIOS Standardized, reproducible paper experiment matrix.
% All controllers receive the same reference, actuator limits, disturbance
% timing and random seed within each scenario.
base = struct('name','baseline','label','Dry-road baseline', ...
    'wetEnabled',false,'wetStart',8,'wetEnd',13,'wetMuScale',0.68, ...
    'gustEnabled',false,'gustTime',8,'gustVyImpulse',0.75, ...
    'payloadEnabled',false,'payloadStart',10,'payloadEnd',16,'massScale',1.15,'inertiaScale',1.10, ...
    'sensorNoise',true,'sensorNoiseScale',1.0);
scenarios = repmat(base,1,6);
scenarios(1)=base;
scenarios(2)=base; scenarios(2).name='wet_road'; scenarios(2).label='Wet-road friction drop'; scenarios(2).wetEnabled=true;
scenarios(3)=base; scenarios(3).name='crosswind'; scenarios(3).label='Crosswind impulse'; scenarios(3).gustEnabled=true;
scenarios(4)=base; scenarios(4).name='payload'; scenarios(4).label='+15% mass / +10% inertia mismatch'; scenarios(4).payloadEnabled=true;
scenarios(5)=base; scenarios(5).name='sensor_noise'; scenarios(5).label='High sensor noise'; scenarios(5).sensorNoiseScale=2.5;
scenarios(6)=base; scenarios(6).name='combined'; scenarios(6).label='Combined worst-case challenge'; scenarios(6).wetEnabled=true; scenarios(6).gustEnabled=true; scenarios(6).payloadEnabled=true; scenarios(6).sensorNoiseScale=2.5;
end
