# DYNAMIC-NAV — Pre-submission hardening plan

**Started:** 2026-11-19  
**Base branch:** `main`  
**Working branch:** `hardening/pre-submission-2026-11-19`  
**Baseline commit:** `cf348a3da71fd62a168cd461556417843f0e684d`

## Working rules

- Keep `main` unchanged until a reviewed pull request is explicitly merged.
- Make small, traceable commits on the hardening branch.
- Re-read the current GitHub file before editing it; never overwrite a newer change blindly.
- Preserve historical experiment outputs. New runs must be stored under new phase-specific filenames.
- Never describe a recorded test result as a fresh run.
- A passing test suite does not imply robustness over untested perturbations or a real-time guarantee.
- Report exact commands, environment, counts, and outcomes for every verification.

## Work log

| Item | Status | Evidence |
|---|---|---|
| Create isolated hardening branch | DONE | `hardening/pre-submission-2026-11-19` |
| Add GitHub Actions pytest workflow | COMMITTED / RUN NOT YET OBSERVED | `.github/workflows/tests.yml`, commit `f7ade8c` |
| Observe CI result on the working branch | PENDING | No workflow run was returned when checked |
| Reconcile README claims against raw phase evidence | PENDING | Compare README with phase 6.5.5 / 6.5.6 JSON and reports |
| Audit submission matrix links and completion claims | PENDING | Check repository-relative paths and approval prerequisites |
| Re-run full pytest suite in a controlled environment | PENDING | Record Python/dependency versions and exact output |
| Re-run baseline and selected robustness regressions | PENDING | New output files only; preserve historical evidence |
| Review code-level safety and causal-observation API | PENDING | Focus on DWA entry points and observation-frame fallback |
| Review repository size and release archive contents | PENDING | Preserve one final demo and all scientific raw evidence |
| Final clean-checkout reproduction | PENDING | Fresh environment, install, launch smoke test, full tests |
| Final pull-request review | PENDING | Do not merge until evidence and documentation agree |

## Known issues to resolve

1. The historical Phase 6.5.5 Level 1 robustness sweep records 25 unsafe/failed variants out of 120 despite deterministic repetitions. Do not hide or overwrite this result; rerun only as a new experiment after any algorithm changes.
2. The Phase 6.5.6 baseline and 14-variant NARROW-GATE results are narrower evidence than the full Level 1 sweep. Keep these scopes distinct in every summary.
3. The NARROW-GATE minimum swept clearance is approximately 0.1500188 m against a 0.15 m threshold; this is a very small positive margin, not a comfortable safety buffer.
4. Recorded NARROW-GATE DWA latency includes P99 232.48 ms and 17.25% of calls above 50 ms. Do not claim hard real-time operation.
5. `phase732_submission_compliance_matrix.md` contains machine-local `file:///c:/...` links. Replace with repository-relative links or verified GitHub paths.
6. `phase656_submission_readiness.json` contains a README warning that must be reconciled with the current README, not silently discarded.
7. Team approval and institution-specific submission fields must be marked pending until actually confirmed.

## Acceptance criteria

- CI passes on the hardening branch and on the pull request.
- All documented test and benchmark claims match retained raw evidence.
- Any newly run experiment has its own immutable, phase-specific output.
- The README and submission documents contain no stale paths, contradictory claims, or unsupported guarantees.
- A clean checkout can install dependencies, launch the headless smoke test, and run the complete test suite.
- No changes are merged into `main` without explicit final review.
