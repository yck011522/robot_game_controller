"""Read-only Parquet adapter invoked by server.mjs; no controller processes are imported.

CLI (from repository root):
  record_player/.venv/Scripts/python record_player/data.py index
  record_player/.venv/Scripts/python record_player/data.py game 2026-10-03/18-37-04
  record_player/.venv/Scripts/python record_player/data.py scene
  record_player/.venv/Scripts/python record_player/data.py analyze
Outputs JSON to stdout. `analyze` caches exploratory metrics for all local games.
"""
import csv
import json
import math
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

HERE = Path(__file__).resolve().parent  # App/cache directory; source recordings remain read-only.
ROOT = HERE.parent  # Repository root, independent of shell working directory.
RECORDINGS = ROOT / 'recordings'  # Copy additional day folders here to make them playable.
CACHE = HERE / '.cache'  # Disposable derived metrics, invalidated by source fingerprint.
HZ = 20  # Preview telemetry sampling rate; source files are never altered.
STALE = 0.25  # Maximum sample age (seconds) for telemetry comparisons.
VERSION = 3  # Increment when changing conversion or metric definitions.


def clean(value):
    """Recursively round floats and replace nonfinite numbers before strict JSON output."""
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, float):
        return round(value, 5) if math.isfinite(value) else None
    return value


def game_path(game_id):
    """Resolve a validated two-component recording ID; called by all game readers."""
    import re
    if not re.fullmatch(r'\d{4}-\d{2}-\d{2}/\d{2}-\d{2}-\d{2}(?:_\d+)?', game_id):
        raise ValueError('Invalid recording ID')
    return RECORDINGS / 'games' / game_id


def fingerprint(folder):
    """Invalidate derived metrics when any local source file changes or is added."""
    return [VERSION, [[str(file.relative_to(folder)), file.stat().st_size, file.stat().st_mtime_ns]
                      for file in sorted(folder.rglob('*.parquet'))]]


def ledger():
    """Read the permanent CSV index, including games whose recordings are offline."""
    with (RECORDINGS / 'games_index.csv').open(encoding='utf-8-sig', newline='') as handle:
        return list(csv.DictReader(handle))


def seconds(value):
    """Parse recorded ISO timestamps with their explicit timezone; missing values stay null."""
    return datetime.fromisoformat(value).timestamp() if value else None


def index():
    """Build browser rows from the ledger plus local availability and cached analysis."""
    output = []  # Rows returned to the sortable browser.
    for row in ledger():
        game_id = row['date'] + '/' + row['time']  # Stable date/time key.
        folder = game_path(game_id)  # Optional local recording folder.
        start, end = seconds(row['play_entered_at']), seconds(row['play_ended_at'])  # Actual Play interval.
        item = dict(id=game_id, date=row['date'], time=row['time'], profile=row['profile_name'],
                    duration=float(row['total_game_time_s'] or 0),
                    play_duration=max(0, end-start) if start is not None and end is not None else None,
                    score_a=float(row['score_a'] or 0), score_b=float(row['score_b'] or 0),
                    travel_a=sum(float(row.get(f'a_joint{j}_distance_rad') or 0) for j in range(1, 7)),
                    travel_b=sum(float(row.get(f'b_joint{j}_distance_rad') or 0) for j in range(1, 7)),
                    local=(folder / 'state_global.parquet').exists())
        item['flags'] = []  # Descriptive clues, never automatic exclusions or skill ratings.
        if item['play_duration'] is not None and item['play_duration'] < 60:
            item['flags'].append('short play')
        if item['travel_a'] + item['travel_b'] < 2:
            item['flags'].append('low travel')
        if item['local']:
            item['skeletons'] = sum((folder / f'skeleton/{team}/frames.parquet').exists() for team in 'ab')
            item['microphones'] = sum((folder / f'audio/{team}{j}/opensmile_lld.parquet').exists() for team in 'ab' for j in range(1, 7))
            if item['skeletons'] < 2 or item['microphones'] < 12:
                item['flags'].append('partial media')
            stats_file = CACHE / (game_id.replace('/', '_') + '.stats.json')  # Reusable exploratory metrics.
            if stats_file.exists():
                saved = json.loads(stats_file.read_text())  # Cached metrics with source fingerprint.
                if saved['fingerprint'] == fingerprint(folder):
                    item.update(saved['metrics'])
        output.append(item)
    return output


def table(folder, relative, warnings):
    """Read one optional stream, reporting corrupt/missing files without inventing samples."""
    try:
        return pq.read_table(folder / relative)
    except Exception as error:
        warnings.append(f'{relative}: {type(error).__name__}: {error}')
        return None


def stream(folder, relative, origin_ns, warnings, fields=None, rate=HZ):
    """Convert wall-clock telemetry to compact column arrays, keeping last sample per preview bin."""
    source = table(folder, relative, warnings)  # Original typed Parquet data.
    if source is None or source.num_rows == 0:
        return dict(t=[], info=dict(rows=0))
    fields = fields or [name for name in source.column_names if name != 'ts_wall_ns']  # Requested telemetry fields.
    times_ns = source['ts_wall_ns'].to_numpy()  # Int64 arithmetic preserves timestamp precision.
    order = np.argsort(times_ns, kind='stable')  # Handle any out-of-order source packets.
    times = (times_ns[order] - origin_ns) / 1e9  # Seconds relative to Tutorial entry.
    bins = np.floor(times * rate).astype(np.int64)  # Preview time buckets.
    keep = np.r_[np.flatnonzero(np.diff(bins) != 0), len(times)-1] if rate else np.arange(len(times))  # Keep final sample in each bucket.
    result = {'t': times[keep].tolist(), 'info': {'rows': source.num_rows,
              'preview_rows': len(keep), 'max_gap_s': float(np.max(np.diff(times))) if len(times)>1 else 0}}
    for field in fields:
        if field in source.column_names:
            result[field] = source[field].take(order[keep]).to_pylist()
    return result


def aligned(source, field, times, max_age=STALE):
    """Last-known-value alignment for metrics; stale/pre-stream values become NaN, never zero."""
    if not source['t'] or field not in source:
        return np.full((len(times), 6), np.nan)
    indices = np.searchsorted(source['t'], times, side='right') - 1  # Causal sample index.
    values = np.asarray(source[field], dtype=float)[np.maximum(indices, 0)].copy()  # Gather vectors.
    ages = times - np.asarray(source['t'])[np.maximum(indices, 0)]  # Elapsed sample age.
    values[(indices < 0) | (ages > max_age)] = np.nan
    return values


def infer_ratios(haptic):
    """Recover recorded signed gearing from paired raw/joint angles instead of current config."""
    raw = np.degrees(np.asarray(haptic.get('dial_pos_rad', []), dtype=float))  # Raw dial degrees.
    mapped = np.asarray(haptic.get('dial_robot_deg', []), dtype=float)  # Recorded mapped joint degrees.
    ratios = []  # Six independently recovered gear ratios; null means unavailable.
    for joint in range(6):
        valid = np.abs(raw[:, joint]) > 1 if raw.size and mapped.shape == raw.shape else np.array([], dtype=bool)
        candidates = mapped[valid, joint] / raw[valid, joint] if valid.any() else np.array([])  # Nonzero paired ratios.
        ratios.append(float(np.median(candidates)) if len(candidates) else None)
    return ratios


def skeleton(folder, team, origin_ns, warnings):
    """Join pose detections to frame timestamps, retaining frames with zero detected people."""
    frames = table(folder, f'skeleton/{team}/frames.parquet', warnings)  # Camera timing and dimensions.
    poses = table(folder, f'skeleton/{team}/skeleton.parquet', warnings)  # COCO-17 pose detections.
    if frames is None:
        return dict(t=[], people=[], width=1280, height=1024)
    metadata = frames.schema.metadata or {}  # Recorded camera width/height.
    grouped = {}  # Pose detections indexed by frame ID; person_id is not assumed to be player ID.
    for pose in poses.to_pylist() if poses is not None else []:
        grouped.setdefault(pose['frame_id'], []).append([pose['person_id'], pose['kp_x'], pose['kp_y'], pose['kp_score']])
    rows = sorted(frames.to_pylist(), key=lambda row: row['ts_unix_ms'])  # Frames in timestamp order.
    return dict(t=[(row['ts_unix_ms'] * 1000000-origin_ns)/1e9 for row in rows],
                people=[grouped.get(row['frame_id'], []) for row in rows],
                width=int(metadata.get(b'width', b'1280')), height=int(metadata.get(b'height', b'1024')))


def audio(folder, player, warnings):
    """Read relative audio timestamps; preserve gaps and label the unknown wall-clock start."""
    source = table(folder, f'audio/{player}/opensmile_lld.parquet', warnings)  # Per-player openSMILE stream.
    if source is None or not source.num_rows:
        return dict(t=[], loudness=[])
    times = source['time_ms'].to_numpy()/1000  # Relative capture seconds; zero is assumed Tutorial entry.
    values = source['Loudness_sma3'].to_numpy()  # Recorded loudness, not calibrated dB SPL.
    order = np.argsort(times, kind='stable')  # Monotonic timestamps for binary search.
    times, values = times[order], values[order]
    keep = np.r_[np.flatnonzero(np.diff(np.floor(times*HZ)) != 0), len(times)-1]  # 20 Hz preview.
    return dict(t=times[keep].tolist(), loudness=values[keep].tolist())


def game(game_id):
    """Assemble one playable recording and write only its disposable derived metrics."""
    folder = game_path(game_id)  # Validated recording directory.
    row = next((row for row in ledger() if row['date']+'/'+row['time'] == game_id), None)  # Ledger metadata.
    if row is None or not (folder/'state_global.parquet').exists():
        raise FileNotFoundError('Recording is not available on this computer')
    origin_ns = round(seconds(row['tutorial_entered_at'])*1e9)  # Common wall-clock origin.
    warnings = []  # Stream read failures surfaced in the viewer.
    result = dict(id=game_id, metadata=row, duration=float(row['total_game_time_s']), teams={}, warnings=warnings,
                  play_start=(seconds(row['play_entered_at'])-origin_ns/1e9) if row['play_entered_at'] else None,
                  sampling_hz=HZ,
                  audio_note='Audio has relative timestamps only: zero aligned provisionally to Tutorial entry. Adjust audio offset; device clock synchronization is unverified.')
    result['global'] = stream(folder, 'state_global.parquet', origin_ns, warnings, rate=1000)
    metrics = {}  # Play-only exploratory metrics for the browser; no player-experience classification.
    for team in 'ab':
        data = {}  # Independent streams for this team.
        for name in ['robot_actual', 'haptic', 'game_controller', 'weight']:
            data[name] = stream(folder, f'{team}/{name}.parquet', origin_ns, warnings, rate=1000 if name == 'game_controller' else HZ)
        data['gear_ratio'] = infer_ratios(data['haptic'])
        data['skeleton'] = skeleton(folder, team, origin_ns, warnings)
        data['audio'] = [audio(folder, f'{team}{joint}', warnings) for joint in range(1, 7)]
        times = np.arange(result['play_start'] if result['play_start'] is not None else result['duration'], result['duration'], 1/HZ)  # Uniform Play-only analysis grid.
        lead = aligned(data['haptic'], 'dial_robot_deg', times)-np.degrees(aligned(data['robot_actual'], 'q_rad', times))  # Signed unwrapped lead in joint degrees.
        finite = np.abs(lead[np.isfinite(lead)])  # Valid per-joint observations only.
        metrics[f'lead_{team}'] = float(np.percentile(finite, 95)) if finite.size else None
        metrics[f'coverage_{team}'] = float(np.isfinite(lead).mean()*100) if lead.size else None
        control = data['game_controller']  # Full-rate controller flags.
        indices = np.searchsorted(control['t'], times, side='right')-1  # Causal alignment to uniform grid.
        if len(control['t']) and len(times):
            valid = (indices >= 0) & (times-np.asarray(control['t'])[np.maximum(indices, 0)] <= STALE)
            flags = np.asarray(control.get('clamp_final', []), dtype=float)  # Global motion scale, not a per-joint collision claim.
            metrics[f'limited_{team}'] = float(np.mean(flags[indices[valid]] < .99)*100) if valid.any() and flags.size else None
        else:
            metrics[f'limited_{team}'] = None
        result['teams'][team] = data
    result['metrics'] = metrics
    CACHE.mkdir(exist_ok=True)
    target = CACHE/(game_id.replace('/', '_')+'.stats.json')  # Final per-recording statistics file.
    temporary = target.with_suffix(f'.{os.getpid()}.tmp')  # Unique staging file prevents partial reads during index refresh.
    temporary.write_text(json.dumps(clean(dict(fingerprint=fingerprint(folder), metrics=metrics))))
    temporary.replace(target)
    return result


def mesh_geometry(mesh, scale=1):
    """Convert embedded COMPAS mesh vertices/faces to browser BufferGeometry arrays."""
    data = mesh.get('data', mesh)  # Unwrap serialized COMPAS type.
    vertices = data['vertex']  # Named vertices in source order.
    keys = {key: index for index, key in enumerate(vertices)}  # Original keys to dense indices.
    positions = [vertex[axis]*scale for vertex in vertices.values() for axis in ['x', 'y', 'z']]  # Metres.
    indices = []  # Fan-triangulated faces; curated mesh faces are convex.
    for face in data['face'].values():
        for offset in range(1, len(face)-1):
            indices.extend(keys[str(key)] for key in [face[0], face[offset], face[offset+1]])
    return dict(positions=positions, indices=indices)


def visuals(link):
    """Extract local visual meshes and origins from a URDF-derived COMPAS link."""
    output = []  # Browser mesh descriptors for this link.
    for visual in link.get('visual', []):
        shape = visual['geometry']['shape']['data']  # Embedded MeshDescriptor.
        for mesh in shape.get('meshes', []):
            output.append(dict(**mesh_geometry(mesh), origin=visual.get('origin'), scale=shape.get('scale', [1, 1, 1])))
    return output


def scene():
    """Export the exact curated URDF-derived robot, attached bucket, and environment in metres."""
    source = json.loads((ROOT/'src/subsystems/robot/assets/robot_cell_and_state.json').read_text())  # Shared collision scene.
    cell, state = source['robot_cell']['data'], source['robot_cell_state']['data']  # Model and placement.
    robot = cell['robot_model']  # UR10e URDF-derived kinematic tree.
    return dict(joints=robot['joints'], links=[dict(name=link['name'], visuals=visuals(link)) for link in robot['links']],
                base=state['robot_base_frame']['data'],
                tool=[visual for link in cell['tool_models']['Bucket']['links'] for visual in visuals(link)],
                tool_state=state['tool_states']['Bucket']['data'],
                bodies=[dict(name=name, frame=state['rigid_body_states'][name]['data']['frame']['data'],
                             hidden=('player' in name or name == 'ceiling'),
                             meshes=[mesh_geometry(mesh, body['native_scale']) for mesh in body['visual_meshes']])
                        for name, body in cell['rigid_body_models'].items()])


def main():
    """Dispatch the documented worker CLI; errors go to stderr for the Node API to report."""
    action = sys.argv[1] if len(sys.argv)>1 else 'index'  # Worker operation requested by server.
    if action == 'analyze':
        completed = []  # IDs successfully analyzed in this batch.
        for row in index():
            if row['local']:
                game(row['id'])
                completed.append(row['id'])
                print(f"Analyzed {row['id']}", file=sys.stderr)
        output = dict(analyzed=len(completed))
    elif action == 'game':
        output = game(sys.argv[2])
    elif action == 'scene':
        output = scene()
    elif action == 'index':
        output = index()
    else:
        raise ValueError('Expected index, game, scene, or analyze')
    print(json.dumps(clean(output), separators=(',', ':'), allow_nan=False))


if __name__ == '__main__':
    main()
