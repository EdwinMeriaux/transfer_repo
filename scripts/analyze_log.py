import argparse
import json
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('log', type=Path)
args = parser.parse_args()
rows = [json.loads(line) for line in args.log.read_text().splitlines()]
valid = [row for row in rows if 'command' in row and 'velocity_flu' in row]
if not valid:
    raise SystemExit('No valid inference rows; inspect fault/status transitions')
errors = np.array([[r['velocity_flu'][0] - r['command'][0], r['angular_velocity_flu'][2] - r['command'][1]] for r in valid])
print(json.dumps(dict(samples=len(valid), rmse_surge_yaw=np.sqrt(np.mean(errors**2, axis=0)).tolist(), inference_p99_ms=float(np.quantile([r['inference_s'] for r in valid], .99) * 1000), status_counts={status:sum(r['status'] == status for r in rows) for status in sorted(set(r['status'] for r in rows))}), indent=2))
