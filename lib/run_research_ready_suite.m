function paper = run_research_ready_suite(p,trim,cfg)
%RUN_RESEARCH_READY_SUITE Full reproducible experiment suite for final research use.
fprintf('\n=== RESEARCH-READY PAPER VALIDATION SUITE ===\n');
cfg.trim=trim;cfg.nominalParams=p;if ~isfolder(cfg.outputDir),mkdir(cfg.outputDir);end
sched=gain_schedule_design(p);analysis=control_analysis(p,sched,cfg.outputDir);
fprintf('\n[1/4] Standardized 4-controller x 6-scenario benchmark...\n');bench=run_paper_scenarios(p,sched,cfg);
fprintf('\n[2/4] Ablation study...\n');ab=run_ablation_study(p,sched,cfg);
fprintf('\n[3/4] Matched Monte Carlo study: %d cases/controller...\n',cfg.paperMonteCarloRuns);mc=monte_carlo_all_controllers(p,sched,cfg);
fprintf('\n[4/4] Writing final research figures/tables...\n');write_paper_parameter_tables(p,sched,cfg,bench);plot_paper_results(bench,ab,mc,analysis,cfg);write_reproducibility_manifest(cfg,bench,mc);
gate=validate_paper_results(bench,mc,cfg);
paper.schedule=sched;paper.analysis=analysis;paper.benchmark=bench;paper.ablation=ab;paper.monteCarlo=mc;paper.validationGate=gate;paper.config=cfg;
save(fullfile(cfg.outputDir,'paper_validation_results.mat'),'paper','-v7.3');
fprintf('\nResearch-ready suite complete. Results: %s\n',cfg.outputDir);
end
