#!/usr/bin/env python3
"""Export a trained MLP policy to a C header for the robot firmware.

Writes robot/main/policy_weights.h, which robot/main/policy.c compiles into
the firmware. The firmware evaluates the network deterministically (the mean
action of the PPO Gaussian) and clamps the output to [-1, 1].

Input: a .npz file with
    W0, W1, ...        layer weights, shape (out, in) like torch.nn.Linear
                       (pass --in-out if yours are (in, out), e.g. JAX/Keras)
    b0, b1, ...        layer biases, shape (out,)
    activations        array of strings, one per layer: tanh | relu | elu | none
                       (usually the last one is "none")
optional observation normalisation, applied as
    clip((obs - obs_mean) / sqrt(obs_var + eps), -obs_clip, obs_clip):
    obs_mean, obs_var  shape (obs_dim,)    (or obs_std instead of obs_var)
    obs_clip           scalar, 0 or missing = no clipping

Example: Stable-Baselines3 PPO (MlpPolicy) with VecNormalize

    import numpy as np, torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize
    pol = PPO.load("model.zip").policy
    lins = [m for m in pol.mlp_extractor.policy_net if isinstance(m, torch.nn.Linear)]
    lins.append(pol.action_net)
    d = {}
    for i, l in enumerate(lins):
        d[f"W{i}"] = l.weight.detach().cpu().numpy()
        d[f"b{i}"] = l.bias.detach().cpu().numpy()
    d["activations"] = np.array(["tanh"] * (len(lins) - 1) + ["none"])
    vn = VecNormalize.load("vecnormalize.pkl", venv)   # skip if not used
    d["obs_mean"], d["obs_var"], d["obs_clip"] = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs
    np.savez("policy.npz", **d)

Then:
    python tools/export_policy.py policy.npz

A placeholder network (all-zero weights, outputs 0) is made with:
    python tools/export_policy.py --placeholder 13
"""
import argparse
import datetime
import pathlib
import sys

import numpy as np

ACTIVATIONS = {"none": 0, "tanh": 1, "relu": 2, "elu": 3}
DEFAULT_OUT = pathlib.Path(__file__).resolve().parent.parent / "robot" / "main" / "policy_weights.h"
# Weights up to this size are copied into internal RAM (faster, no cache misses);
# bigger networks are left in flash.
DRAM_LIMIT_BYTES = 96 * 1024
NUM_TESTS = 4


def apply_activation(x, name):
    if name == "tanh":
        return np.tanh(x)
    if name == "relu":
        return np.maximum(x, 0.0)
    if name == "elu":
        return np.where(x > 0.0, x, np.expm1(x))
    return x


def forward(policy, obs):
    x = obs.astype(np.float64)
    if policy["obs_mean"] is not None:
        x = (x - policy["obs_mean"]) * policy["obs_inv_std"]
    if policy["obs_clip"] > 0:
        x = np.clip(x, -policy["obs_clip"], policy["obs_clip"])
    for W, b, act in zip(policy["W"], policy["b"], policy["act"]):
        x = apply_activation(W.astype(np.float64) @ x + b, act)
    return np.clip(x, -1.0, 1.0)


def load_npz(path, in_out, eps):
    data = np.load(path, allow_pickle=False)
    n = 0
    while f"W{n}" in data:
        n += 1
    if n == 0:
        sys.exit(f"{path}: no W0 found")
    W = [np.asarray(data[f"W{i}"], dtype=np.float32) for i in range(n)]
    if in_out:
        W = [w.T.copy() for w in W]
    b = [np.asarray(data[f"b{i}"], dtype=np.float32).reshape(-1) for i in range(n)]
    if "activations" not in data:
        sys.exit(f"{path}: missing 'activations'")
    act = [str(a).lower() for a in data["activations"]]
    if len(act) != n:
        sys.exit(f"{path}: {n} layers but {len(act)} activations")

    obs_mean = obs_inv_std = None
    if "obs_mean" in data:
        obs_mean = np.asarray(data["obs_mean"], dtype=np.float64).reshape(-1)
        if "obs_var" in data:
            std = np.sqrt(np.asarray(data["obs_var"], dtype=np.float64).reshape(-1) + eps)
        elif "obs_std" in data:
            std = np.asarray(data["obs_std"], dtype=np.float64).reshape(-1)
        else:
            sys.exit(f"{path}: obs_mean given without obs_var / obs_std")
        obs_inv_std = 1.0 / std
    obs_clip = float(data["obs_clip"]) if "obs_clip" in data else 0.0
    return {"W": W, "b": b, "act": act, "obs_mean": obs_mean,
            "obs_inv_std": obs_inv_std, "obs_clip": obs_clip}


def make_placeholder(obs_dim, hidden=32, act_dim=2):
    return {
        "W": [np.zeros((hidden, obs_dim), np.float32), np.zeros((act_dim, hidden), np.float32)],
        "b": [np.zeros(hidden, np.float32), np.zeros(act_dim, np.float32)],
        "act": ["tanh", "none"],
        "obs_mean": None, "obs_inv_std": None, "obs_clip": 0.0,
    }


def validate(policy):
    W, b, act = policy["W"], policy["b"], policy["act"]
    for i, (w, bb, a) in enumerate(zip(W, b, act)):
        if w.ndim != 2:
            sys.exit(f"W{i} must be 2-D, got shape {w.shape}")
        if bb.shape[0] != w.shape[0]:
            sys.exit(f"b{i} has {bb.shape[0]} entries but W{i} has {w.shape[0]} outputs "
                     "(weights may be (in, out): try --in-out)")
        if i > 0 and w.shape[1] != W[i - 1].shape[0]:
            sys.exit(f"W{i} expects {w.shape[1]} inputs but layer {i - 1} outputs {W[i - 1].shape[0]} "
                     "(weights may be (in, out): try --in-out)")
        if a not in ACTIVATIONS:
            sys.exit(f"unknown activation '{a}' for layer {i}; use one of {list(ACTIVATIONS)}")
        if not (np.all(np.isfinite(w)) and np.all(np.isfinite(bb))):
            sys.exit(f"layer {i} contains NaN/Inf")
    obs_dim = W[0].shape[1]
    if policy["obs_mean"] is not None and policy["obs_mean"].shape[0] != obs_dim:
        sys.exit(f"obs_mean has {policy['obs_mean'].shape[0]} entries, network takes {obs_dim}")
    if W[-1].shape[0] != 2:
        print(f"warning: network outputs {W[-1].shape[0]} actions, firmware drives 2 motors",
              file=sys.stderr)


def c_floats(values, per_line=6, indent="    "):
    vals = [f"{float(v):.9e}f" for v in np.asarray(values, dtype=np.float32).reshape(-1)]
    lines = [indent + ", ".join(vals[i:i + per_line]) + "," for i in range(0, len(vals), per_line)]
    return "\n".join(lines) if lines else indent + "0"


def render(policy, source, placeholder):
    W, b, act = policy["W"], policy["b"], policy["act"]
    n = len(W)
    obs_dim, act_dim = W[0].shape[1], W[-1].shape[0]
    max_width = max([obs_dim] + [w.shape[0] for w in W])
    num_params = sum(w.size + bb.size for w, bb in zip(W, b))
    in_dram = num_params * 4 <= DRAM_LIMIT_BYTES

    rng = np.random.default_rng(0)
    if policy["obs_mean"] is not None:
        test_obs = policy["obs_mean"] + rng.standard_normal((NUM_TESTS, obs_dim)) / policy["obs_inv_std"]
    else:
        test_obs = rng.standard_normal((NUM_TESTS, obs_dim))
    test_obs = test_obs.astype(np.float32)
    test_act = np.stack([forward(policy, o) for o in test_obs])

    out = []
    out.append("/* GENERATED by tools/export_policy.py - do not edit by hand.")
    out.append(f"   Source: {source}")
    out.append(f"   Generated: {datetime.datetime.now().isoformat(timespec='seconds')}")
    arch = " -> ".join([str(obs_dim)] + [f"{w.shape[0]} ({a})" for w, a in zip(W, act)])
    out.append(f"   Network: {arch}, {num_params} parameters */")
    out.append("#pragma once")
    out.append("")
    out.append('#include "esp_attr.h"')
    out.append("")
    out.append(f"#define POLICY_IS_PLACEHOLDER   {1 if placeholder else 0}")
    out.append(f"#define POLICY_OBS_DIM          {obs_dim}")
    out.append(f"#define POLICY_ACT_DIM          {act_dim}")
    out.append(f"#define POLICY_NUM_LAYERS       {n}")
    out.append(f"#define POLICY_MAX_WIDTH        {max_width}")
    out.append(f"#define POLICY_NUM_PARAMS       {num_params}")
    out.append(f"#define POLICY_HAS_OBS_NORM     {0 if policy['obs_mean'] is None else 1}")
    out.append(f"#define POLICY_OBS_CLIP         {policy['obs_clip']:.9e}f   /* 0 = no clipping */")
    out.append(f"#define POLICY_WEIGHT_ATTR      {'DRAM_ATTR' if in_dram else ''}"
               f"   /* {'internal RAM' if in_dram else 'flash'} */")
    out.append("")
    out.append("static const int policy_layer_in[POLICY_NUM_LAYERS] = {"
               + ", ".join(str(w.shape[1]) for w in W) + "};")
    out.append("static const int policy_layer_out[POLICY_NUM_LAYERS] = {"
               + ", ".join(str(w.shape[0]) for w in W) + "};")
    out.append("static const int policy_layer_act[POLICY_NUM_LAYERS] = {"
               + ", ".join(f"POLICY_ACTIVATION_{a.upper()}" for a in act) + "};")
    out.append("")
    for i, (w, bb) in enumerate(zip(W, b)):
        out.append(f"/* layer {i}: {w.shape[1]} -> {w.shape[0]}, row-major [out][in] */")
        out.append(f"static const float POLICY_WEIGHT_ATTR policy_w{i}[{w.size}] = {{")
        out.append(c_floats(w))
        out.append("};")
        out.append(f"static const float POLICY_WEIGHT_ATTR policy_b{i}[{bb.size}] = {{")
        out.append(c_floats(bb))
        out.append("};")
        out.append("")
    out.append("static const float *const policy_w[POLICY_NUM_LAYERS] = {"
               + ", ".join(f"policy_w{i}" for i in range(n)) + "};")
    out.append("static const float *const policy_b[POLICY_NUM_LAYERS] = {"
               + ", ".join(f"policy_b{i}" for i in range(n)) + "};")
    out.append("")
    if policy["obs_mean"] is not None:
        out.append("static const float POLICY_WEIGHT_ATTR policy_obs_mean[POLICY_OBS_DIM] = {")
        out.append(c_floats(policy["obs_mean"]))
        out.append("};")
        out.append("static const float POLICY_WEIGHT_ATTR policy_obs_inv_std[POLICY_OBS_DIM] = {")
        out.append(c_floats(policy["obs_inv_std"]))
        out.append("};")
        out.append("")
    out.append("/* Reference inputs/outputs computed in numpy, checked at boot */")
    out.append(f"#define POLICY_NUM_TESTS {NUM_TESTS}")
    out.append("static const float policy_test_obs[POLICY_NUM_TESTS][POLICY_OBS_DIM] = {")
    for o in test_obs:
        out.append("    {\n" + c_floats(o, indent="        ") + "\n    },")
    out.append("};")
    out.append("static const float policy_test_act[POLICY_NUM_TESTS][POLICY_ACT_DIM] = {")
    for a in test_act:
        out.append("    {\n" + c_floats(a, indent="        ") + "\n    },")
    out.append("};")
    out.append("")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("npz", nargs="?", help="exported weights (.npz)")
    src.add_argument("--placeholder", type=int, metavar="OBS_DIM",
                     help="write an all-zero network with this many observations")
    ap.add_argument("-o", "--output", type=pathlib.Path, default=DEFAULT_OUT,
                    help=f"output header (default: {DEFAULT_OUT})")
    ap.add_argument("--in-out", action="store_true", help="weights are stored (in, out)")
    ap.add_argument("--obs-eps", type=float, default=1e-8,
                    help="epsilon added to obs_var (SB3 VecNormalize uses 1e-8)")
    args = ap.parse_args()

    if args.placeholder is not None:
        policy = make_placeholder(args.placeholder)
        source = f"placeholder, {args.placeholder} observations"
    else:
        policy = load_npz(args.npz, args.in_out, args.obs_eps)
        source = pathlib.Path(args.npz).name
    validate(policy)

    args.output.write_text(render(policy, source, args.placeholder is not None))
    W = policy["W"]
    print(f"wrote {args.output}: {W[0].shape[1]} obs -> {W[-1].shape[0]} actions, "
          f"{len(W)} layers, {sum(w.size + bb.size for w, bb in zip(W, policy['b']))} parameters")


if __name__ == "__main__":
    main()
