function gate = validate_paper_results(bench,mc,cfg)
%VALIDATE_PAPER_RESULTS Numerical/methodological gate for final research evidence.
% A weak baseline controller is a legitimate research result and therefore
% does NOT invalidate the experiment.  The gate checks that the advanced
% controllers stabilize the nominal benchmark and that the matched
% parameter-uncertainty study is numerically meaningful.
nc=numel(bench.controllers);baseSuccess=false(1,nc);baseBeta=nan(1,nc);baseFinite=false(1,nc);
for c=1:nc
    m=bench.runs{1,c}.metrics;baseSuccess(c)=logical(m.success);baseBeta(c)=m.betaRMSEdeg;baseFinite(c)=isfinite(m.betaRMSEdeg)&&all(isfinite(bench.runs{1,c}.x(:)));
end
gate.baselineSuccess=baseSuccess;gate.baselineBetaRMSEdeg=baseBeta;gate.monteCarloSuccessRate=mc.successRate;
gate.allBaselineFinite=all(baseFinite);gate.allMonteCarloFinite=all(isfinite(mc.betaRMSEdeg(:)));
% PID is intentionally a classical baseline. Require Fixed LQR, GS-LQR and
% constrained MPC to stabilize the nominal case; require at least the two
% modern scheduled/predictive controllers to retain >=50% success under the
% declared matched parametric uncertainty.
gate.advancedBaselinePass=all(baseSuccess(2:4));
gate.robustnessPass=all(mc.successRate(3:4)>=50);
gate.pass=gate.allBaselineFinite && gate.allMonteCarloFinite && gate.advancedBaselinePass && gate.robustnessPass;
fid=fopen(fullfile(cfg.outputDir,'PAPER_VALIDATION_GATE.txt'),'w');
fprintf(fid,'PAPER VALIDATION GATE V3\n========================\n');
fprintf(fid,'Baseline success [PID FixedLQR GS-LQR MPC]: ');fprintf(fid,'%d ',baseSuccess);fprintf(fid,'\n');
fprintf(fid,'Baseline beta RMSE deg: ');fprintf(fid,'%.3f ',baseBeta);fprintf(fid,'\n');
fprintf(fid,'Monte Carlo success rates %%: ');fprintf(fid,'%.2f ',mc.successRate);fprintf(fid,'\n');
fprintf(fid,'All baseline trajectories finite: %d\n',gate.allBaselineFinite);
fprintf(fid,'All Monte Carlo metrics finite: %d\n',gate.allMonteCarloFinite);
fprintf(fid,'Advanced nominal stabilization pass: %d\n',gate.advancedBaselinePass);
fprintf(fid,'GS-LQR/MPC robustness gate >=50%%: %d\n',gate.robustnessPass);
fprintf(fid,'NOTE: PID is a comparison baseline; poor PID performance is reportable and does not by itself invalidate the study.\n');
if gate.pass,fprintf(fid,'STATUS: PASS - numerical results satisfy the V3 research validation gate.\n');else,fprintf(fid,'STATUS: FAIL - inspect controller/scenario results before research-report use.\n');end
fclose(fid);
if gate.pass,fprintf('\nPAPER VALIDATION GATE V3: PASS\n');else,warning('CarDrift:PaperValidation','PAPER VALIDATION GATE V3: FAIL. Inspect PAPER_VALIDATION_GATE.txt.');end
end
