# Model artifacts

- `checkpoints/blueboat_coupled_4m_mac.model`: original 7,141,893-byte ZIP/Orbax checkpoint, unchanged.
- SHA-256: `43213ea3bc2219d93acf991fdb74c02a559c1d2ed77a8af266138474eb1269e4`.
- `checkpoints/actor_weights.npz`: lossless extracted actor parameter arrays, including unused exploration parameters.
- `checkpoints/provenance.json`: source commit and copied-file hashes.
- `inference/blueboat_actor.onnx`: link to the ROS package's canonical 1,520,697-byte model.
- ONNX SHA-256: `63cf44c2461a1819d32aa31245370dc3d4001b3ba05da5f85bda56d6157cb8fd`.
- `inference/observation_contract.json`: link to the exact normalized template and indices in the ROS package.
- `inference/equivalence.json`: original JAX vs ONNX numerical comparison.

The ROS package includes its own model, contract and SHA manifest. Copying that package alone is sufficient for inference. No symlink from the package points outside it. Runtime verifies hashes before loading ONNX, then warms the CPU session. The original checkpoint is preserved for provenance/re-export and is not loaded by the deployed node.

## Optional re-export

This is developer work; Alberto does not need the research environment. To regenerate original reference vectors, use the original JAX/Flax/MuJoCo/RL-X environment and an explicit source checkout at the provenance commit:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$RLX_CHECKOUT" "$RESEARCH_PYTHON" scripts/extract_reference.py --source "$LOCO_MJX_CHECKOUT"
python -m pip install -r requirements-dev.txt
python scripts/export_onnx.py
python -m pytest -q tests/test_policy.py
```

`RLX_CHECKOUT`, `RESEARCH_PYTHON` and `LOCO_MJX_CHECKOUT` are operator-provided external paths. The extraction uses `plane.xml`, not the legacy model. It loads the original checkpoint through the original actor and observation adapter, then stores 1,024 JAX outputs and inputs in `tests/reference_vectors.npz`. Tests in this repository require only the fixtures and ONNX; they do not import research code. Re-exporting the same model should preserve its hash; any deliberate artifact change requires regenerating and reviewing `assets/manifest.json` as well as the parity report. No automatic trust update occurs at runtime.
