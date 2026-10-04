# GR00T N1.7 open-loop evaluator

Source: https://github.com/NVIDIA/Isaac-GR00T/blob/51d4c89f72fda44cbf77285c6a8114b52676b8a1/gr00t/eval/open_loop_eval.py

The three functions retain their upstream bodies verbatim, including plotting
layout, native-unit MSE/MAE, chunk concatenation, last-chunk truncation and
state plotting only when the full state/action shapes match. Apache-2.0
license and copyright are retained. Imports are replaced by a small F14/OpenPI
adapter; deferred annotations avoid importing GR00T model dependencies.

No GR00T weights, normalization or robot sign conventions are used. The
OpenPI policy handles its own checkpoint normalization and sampling.
