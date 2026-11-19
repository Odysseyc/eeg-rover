"""Step 1: baseline motor-imagery classifiers on BCI Competition IV dataset 2a ("Graz 2a").

For each of the 9 subjects:
  * train on session 1 (day 1), test on session 2 (day 2), the realistic "new day" setting
  * two standard pipelines:
      csp_lda : band-pass 8-30 Hz -> Common Spatial Patterns -> Linear Discriminant Analysis
      ts_lr   : covariance -> Riemannian tangent space -> logistic regression
  * report accuracy, Cohen's kappa (the competition's metric) and the confusion matrix

Outputs (in eeg/results/):
  summary.csv, summary.json         accuracy and kappa per subject and pipeline
  confusion_<pipeline>.png          3x3 grid of per-subject confusion matrices
  accuracy.png                      accuracy per subject, both pipelines, chance line
  replay/subject_<N>.npz            step 2 replay bank: every test trial's true label, predicted
                                    label and class probabilities (best pipeline per subject)
  models/subject_<N>_<pipeline>.joblib   trained pipelines

Class -> rover command (agreed with the EEG team):
  left_hand -> left, right_hand -> right, feet -> forward, tongue -> reverse

Usage:
  python train_baseline.py                 # all 9 subjects (downloads the data on first run)
  python train_baseline.py --subjects 1 3  # just some
  python train_baseline.py --synthetic     # fake data, to check the pipeline without downloading
"""
import argparse
import json
from pathlib import Path

import numpy as np

CLASSES = ['left_hand', 'right_hand', 'feet', 'tongue']
COMMANDS = {'left_hand': 'left', 'right_hand': 'right', 'feet': 'forward', 'tongue': 'reverse'}
SFREQ = 250


# ---------------------------------------------------------------- data

def load_subject(subject, tmin, tmax):
    """Return (X_train, y_train, X_test, y_test) for one subject, band-passed 8-30 Hz.

    tmin/tmax are seconds after the cue (MOABB measures from the start of the imagery interval).
    """
    from moabb.datasets import BNCI2014_001
    from moabb.paradigms import MotorImagery
    paradigm = MotorImagery(n_classes=4, fmin=8, fmax=30, tmin=tmin, tmax=tmax)
    X, y, meta = paradigm.get_data(dataset=BNCI2014_001(), subjects=[subject])
    sessions = sorted(meta['session'].unique())  # e.g. ['0train', '1test']
    if len(sessions) != 2:
        raise RuntimeError(f'Expected 2 sessions, got {sessions}')
    tr, te = (meta['session'] == sessions[0]).values, (meta['session'] == sessions[1]).values
    return X[tr], np.asarray(y)[tr], X[te], np.asarray(y)[te]


def synthetic_subject(subject, tmin, tmax, n_per_class=72):
    """Fake EEG with motor-imagery-like structure: each class weakens a 10 Hz rhythm over a
    different 'motor' channel group. Day 2 gets a small shift so cross-session is harder."""
    rng = np.random.default_rng(subject)
    n_t = int((tmax - tmin) * SFREQ)
    t = np.arange(n_t) / SFREQ
    groups = {'left_hand': [9, 10, 11], 'right_hand': [7, 8, 13], 'feet': [3, 4], 'tongue': [15, 16, 17]}
    strength = 0.02 + 0.012 * subject  # some subjects are 'better BCI users' than others

    def session(shift):
        X, y = [], []
        for c in CLASSES:
            for _ in range(n_per_class):
                x = rng.normal(0, 1, (22, n_t))
                alpha = np.sin(2 * np.pi * (10 + rng.normal(0, 0.5)) * t + rng.uniform(0, 2 * np.pi))
                x += 1.5 * alpha  # background rhythm everywhere
                x[groups[c]] -= strength * 1.5 * alpha * (1 - shift)  # class-specific desynchronization
                X.append(x)
                y.append(c)
        return np.array(X), np.array(y)

    Xtr, ytr = session(0.0)
    Xte, yte = session(0.15)
    return Xtr, ytr, Xte, yte


# ---------------------------------------------------------------- models

def pipelines():
    from mne.decoding import CSP
    from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
    from sklearn.pipeline import make_pipeline
    pipes = {'csp_lda': make_pipeline(CSP(n_components=8, reg='ledoit_wolf', log=True),
                                      LinearDiscriminantAnalysis())}
    try:
        from pyriemann.estimation import Covariances
        from pyriemann.tangentspace import TangentSpace
        from sklearn.linear_model import LogisticRegression
        pipes['ts_lr'] = make_pipeline(Covariances('oas'), TangentSpace(metric='riemann'),
                                       LogisticRegression(C=1.0, max_iter=2000))
    except ImportError:
        print('pyriemann not installed: skipping ts_lr')
    return pipes


# ---------------------------------------------------------------- plots

def plot_confusions(results, pipe, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    subs = sorted(results)
    n = len(subs)
    cols = min(3, n)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(3.6 * cols, 3.4 * rows), squeeze=False)
    short = ['L', 'R', 'F', 'Rev']
    for ax in axes.flat:
        ax.axis('off')
    for ax, s in zip(axes.flat, subs):
        r = results[s][pipe]
        cm = np.array(r['confusion_normalized'])
        ax.axis('on')
        ax.imshow(cm, vmin=0, vmax=1, cmap='Blues')
        for i in range(4):
            for j in range(4):
                ax.text(j, i, f'{cm[i, j]:.0%}', ha='center', va='center', fontsize=9,
                        color='white' if cm[i, j] > 0.55 else 'black')
        ax.set_xticks(range(4), short)
        ax.set_yticks(range(4), short)
        ax.set_title(f'Subject {s}: {r["accuracy"]:.0%}', fontsize=11)
        ax.set_xlabel('decoded command')
        ax.set_ylabel('intended command')
    fig.suptitle(f'Day-2 confusion matrices ({pipe}); rows = intended, columns = decoded', fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_accuracy(results, names, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    subs = sorted(results)
    x = np.arange(len(subs))
    w = 0.8 / len(names)
    fig, ax = plt.subplots(figsize=(1.0 * len(subs) + 3, 4))
    for k, p in enumerate(names):
        acc = [results[s][p]['accuracy'] for s in subs]
        ax.bar(x + (k - (len(names) - 1) / 2) * w, acc, w, label=f'{p} (mean {np.mean(acc):.0%})')
    ax.axhline(0.25, ls='--', color='gray', lw=1)
    ax.text(len(subs) - 0.5, 0.26, 'chance', color='gray', ha='right', fontsize=9)
    ax.set_xticks(x, [f'S{s}' for s in subs])
    ax.set_ylim(0, 1)
    ax.set_ylabel('day-2 accuracy (4 classes)')
    ax.legend(loc='upper left', fontsize=9)
    ax.set_title('Graz 2a: train on day 1, test on day 2')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--subjects', type=int, nargs='+', default=list(range(1, 10)))
    ap.add_argument('--tmin', type=float, default=0.5, help='window start, s after the cue')
    ap.add_argument('--tmax', type=float, default=2.5, help='window end, s after the cue')
    ap.add_argument('--synthetic', action='store_true', help='use fake data (no download)')
    ap.add_argument('--out', default=str(Path(__file__).parent / 'results'))
    args = ap.parse_args()

    import joblib
    import mne
    mne.set_log_level('ERROR')  # MNE is very chatty during CSP fitting
    from sklearn.base import clone
    from sklearn.metrics import accuracy_score, cohen_kappa_score, confusion_matrix

    out = Path(args.out)
    (out / 'replay').mkdir(parents=True, exist_ok=True)
    (out / 'models').mkdir(parents=True, exist_ok=True)
    pipes = pipelines()
    results = {}

    for s in args.subjects:
        if args.synthetic:
            Xtr, ytr, Xte, yte = synthetic_subject(s, args.tmin, args.tmax)
        else:
            print(f'Subject {s}: loading (first run downloads the data)...')
            Xtr, ytr, Xte, yte = load_subject(s, args.tmin, args.tmax)
        results[s] = {}
        best = None
        for name, pipe in pipes.items():
            model = clone(pipe).fit(Xtr, ytr)
            pred = model.predict(Xte)
            proba = model.predict_proba(Xte)
            order = [list(model.classes_).index(c) for c in CLASSES]  # columns in CLASSES order
            proba = proba[:, order]
            acc = accuracy_score(yte, pred)
            cm = confusion_matrix(yte, pred, labels=CLASSES)
            results[s][name] = {
                'accuracy': float(acc),
                'kappa': float(cohen_kappa_score(yte, pred)),
                'confusion': cm.tolist(),
                'confusion_normalized': (cm / cm.sum(axis=1, keepdims=True)).tolist(),
                'n_train': int(len(ytr)), 'n_test': int(len(yte)),
            }
            joblib.dump(model, out / 'models' / f'subject_{s}_{name}.joblib')
            print(f'  S{s} {name:8s} accuracy {acc:5.1%}  kappa {results[s][name]["kappa"]:.2f}')
            if best is None or acc > best[0]:
                best = (acc, name, pred, proba)
        acc, name, pred, proba = best
        np.savez(out / 'replay' / f'subject_{s}.npz',
                 y_true=np.asarray(yte, dtype=str), y_pred=np.asarray(pred, dtype=str),
                 proba=proba.astype(np.float32), classes=np.array(CLASSES),
                 commands=np.array([COMMANDS[c] for c in CLASSES]),
                 pipeline=name, accuracy=acc, window=np.array([args.tmin, args.tmax]))
        results[s]['replay_pipeline'] = name

    names = list(pipes)
    rows = ['subject,' + ','.join(f'{p}_accuracy,{p}_kappa' for p in names)]
    for s in sorted(results):
        rows.append(f'{s},' + ','.join(f'{results[s][p]["accuracy"]:.4f},{results[s][p]["kappa"]:.4f}' for p in names))
    rows.append('mean,' + ','.join(
        f'{np.mean([results[s][p]["accuracy"] for s in results]):.4f},'
        f'{np.mean([results[s][p]["kappa"] for s in results]):.4f}' for p in names))
    (out / 'summary.csv').write_text('\n'.join(rows) + '\n')
    meta = {'dataset': 'synthetic' if args.synthetic else 'BNCI2014_001 (BCI Competition IV 2a)',
            'window_s_after_cue': [args.tmin, args.tmax], 'band_hz': [8, 30],
            'split': 'train session 1, test session 2', 'classes': CLASSES, 'commands': COMMANDS,
            'results': {str(k): v for k, v in results.items()}}
    (out / 'summary.json').write_text(json.dumps(meta, indent=2))

    for p in names:
        plot_confusions(results, p, out / f'confusion_{p}.png')
    plot_accuracy(results, names, out / 'accuracy.png')

    print('\nMean day-2 accuracy (chance = 25%):')
    for p in names:
        print(f'  {p:8s} {np.mean([results[s][p]["accuracy"] for s in results]):5.1%}')
    print(f'Results in {out}')


if __name__ == '__main__':
    main()
