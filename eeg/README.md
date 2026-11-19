# EEG: motor-imagery classifiers (BCI Competition IV 2a, "Graz 2a")

Dataset: 9 subjects, 22 EEG channels at 250 Hz, 4 imagined movements, 288 trials per session,
2 sessions on different days. Downloaded automatically by MOABB (a few hundred MB, into `~/mne_data`).

Class → rover command: left hand → **left**, right hand → **right**, feet → **forward**, tongue → **reverse**.

## Step 1: baseline (this folder)
```bash
python3 -m venv ~/eeg-venv && source ~/eeg-venv/bin/activate   # keeps these packages away from ROS 2
pip install -r eeg/requirements.txt
python eeg/train_baseline.py              # all 9 subjects; add --synthetic to test without downloading
```
Trains on day 1, tests on day 2 (the realistic setting for a returning user), with two standard pipelines:
- `csp_lda`: 8–30 Hz band-pass → Common Spatial Patterns → Linear Discriminant Analysis
- `ts_lr`: covariance matrices → Riemannian tangent space → logistic regression

Window: 0.5–2.5 s after the cue. Metrics: accuracy (chance 25 %), Cohen's kappa (the competition metric),
and confusion matrices (rows = intended command, columns = decoded command).

Outputs in `eeg/results/`: `summary.csv`, `summary.json`, `accuracy.png`, `confusion_<pipeline>.png`,
`replay/subject_<N>.npz` (each test trial's true label, predicted label and probabilities, from the better
pipeline for that subject; used by the step 2 EEG replay in the simulator), and `models/` (trained pipelines).
