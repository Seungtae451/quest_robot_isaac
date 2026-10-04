"""Plot locally persisted training metrics without accessing W&B or GPUs."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('history', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rows = {}
    for line in args.history.read_text().splitlines():
        if line.strip():
            row = json.loads(line)
            rows[row['step']] = row  # A resumed step supersedes its earlier value.
    if not rows:
        parser.error('No recorded metrics yet.')
    ordered = [rows[key] for key in sorted(rows)]
    names = [key for key in ('loss', 'grad_norm', 'param_norm')
             if any(key in row for row in ordered)]
    if not names:
        parser.error('History has no loss/grad_norm/param_norm metrics.')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(names), 1, figsize=(10, 3*len(names)), squeeze=False,
                             constrained_layout=True)
    for name, axis in zip(names, axes.flat):
        values = [row for row in ordered if name in row]
        axis.plot([row['step'] for row in values], [row[name] for row in values], marker='.', linewidth=1)
        axis.set(xlabel='Training step', ylabel=name)
        axis.grid(alpha=.25)
    output = args.output or args.history.with_suffix('.png')
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output.resolve())


if __name__ == '__main__':
    main()
