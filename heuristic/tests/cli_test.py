# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
"""CLI/configuration contract; test outputs stay in a temporary directory."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

exe, instance = sys.argv[1:]
with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    config = root / 'options.json'
    config.write_text(json.dumps({'max_new_blocks': 4, 'paths_per_group': 16}))
    for method in ['arc', 'heuristic']:
        output_dir = root / method
        output_dir.mkdir()
        result = output_dir / 'result.json'
        command = [exe, instance, method, '2', str(result)]
        if method == 'heuristic':
            command += ['--config', str(config)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        assert list(output_dir.iterdir()) == [result]
        data = json.loads(result.read_text())
        assert abs(data['objective'] - 28) < 1e-6
        assert abs(data['validated_objective'] - data['objective']) < 1e-6
        if method == 'heuristic':
            assert data['search_options']['max_new_blocks'] == 4
            assert data['search_options']['paths_per_group'] == 16
            assert data['gap'] is None
    # Final result JSON supplies a validated warm start to either solver.
    final_result = root / 'heuristic' / 'result.json'
    final = json.loads(final_result.read_text())
    old = final_result.read_bytes()
    for method in ['arc', 'heuristic']:
        resumed = root / (method + '-warm.json')
        subprocess.run([exe, instance, method, '2', str(resumed), '--incumbent', str(final_result)],
                       check=True, capture_output=True)
        assert json.loads(resumed.read_text())['objective'] <= final['objective']
        assert final_result.read_bytes() == old
    removed_output = root / 'removed-option.json'
    removed_directory = root / 'removed-option'
    proc = subprocess.run([exe, instance, 'heuristic', '1', str(removed_output),
                           '--checkpoint-dir', str(removed_directory)], capture_output=True)
    assert proc.returncode and b'Unknown option: --checkpoint-dir' in proc.stderr
    assert not removed_output.exists() and not removed_directory.exists()
    # Zero seconds must stay alive even after an optimal pool gives no improvement.
    live_dir = root / 'continuous'
    live_dir.mkdir()
    live_log = root / 'continuous.log'
    with live_log.open('w') as log:
        proc = subprocess.Popen([exe, instance, 'heuristic', '0', str(live_dir / 'result.json'),
                                 '--incumbent', str(final_result)],
                                stdout=log, stderr=subprocess.STDOUT)
        try:
            until = time.monotonic() + 8
            while time.monotonic() < until and proc.poll() is None:
                text = live_log.read_text()
                if 'SEARCH_CYCLE iteration=2' in text and 'PATH_MIP_SKIP' in text:
                    break
                time.sleep(.02)
            assert proc.poll() is None, 'Unlimited search returned after pool exhaustion'
            text = live_log.read_text()
            assert 'PATH_MIP_DONE' in text and 'PATH_MIP_SKIP' in text
            assert 'SEARCH_CYCLE iteration=2' in text
            time.sleep(.2)
            assert proc.poll() is None
        finally:
            if proc.poll() is None:
                proc.terminate()
            proc.wait(timeout=10)
    assert not list(live_dir.iterdir())
    assert final_result.read_bytes() == old
    for removed in ['bpc', 'single-cg', 'arc-lp']:
        result = root / 'removed.json'
        proc = subprocess.run([exe, instance, removed, '1', str(result)], capture_output=True)
        assert proc.returncode and not result.exists()
    config.write_text('{"max_new_block":4}')
    proc = subprocess.run([exe, instance, 'heuristic', '1', str(root/'bad.json'), '--config', str(config)], capture_output=True)
    assert proc.returncode and not (root/'bad.json').exists()
    result = root / 'expired.json'
    subprocess.run([exe, instance, 'heuristic', '1', str(result), '--deadline', str(time.time()-1)], check=True, capture_output=True)
    data = json.loads(result.read_text())
    assert data['status'] == 'input_time_limit' and data['solution'] is None
    config.write_text(json.dumps({'pool_capture_only': True}))
    capture = root / 'capture.json'
    subprocess.run([exe, instance, 'heuristic', '3', str(capture), '--config', str(config)],
                   check=True, capture_output=True)
    snapshot = json.loads(capture.read_text())
    state = snapshot['profile']['candidate_pool']
    assert snapshot['status'] == 'pool_prepared' and snapshot['profile']['pool_mip_calls'] == 0
    config.write_text(json.dumps({'pool_fix_fraction': .5, 'pool_extra_paths': 4,
                                 'record_candidate_pool': True}))
    replay = root / 'replay.json'
    command = [exe, instance, 'heuristic', '3', str(replay), '--config', str(config),
               '--pool-state', str(capture)]
    subprocess.run(command, check=True, capture_output=True)
    data = json.loads(replay.read_text())
    assert data['objective'] <= state['incumbent_objective'] + 1e-6
    assert data['profile']['cutoff'] == state['incumbent_objective']
    assert data['profile']['candidate_pool']['physical_paths'] == state['physical_paths']
    config.write_text(json.dumps({'record_candidate_pool': True}))
    subprocess.run(command, check=True, capture_output=True)
    data = json.loads(replay.read_text())
    assert data['profile']['candidate_pool']['logical_paths'] == state['logical_paths']
    assert data['profile']['candidate_pool']['physical_paths'] == state['physical_paths']
    pg = root / 'pg.json'
    config.write_text(json.dumps({'guidance_max_iterations': 1, 'guidance_solver_log': False}))
    subprocess.run([exe, instance, 'pg', '3', str(pg), '--pool-state', str(capture),
                    '--config', str(config)], check=True, capture_output=True)
    generated = json.loads(pg.read_text())
    generated_pool = generated['profile']['candidate_pool']
    assert generated['method'] == 'pg' and generated['status'] == 'paths_generated'
    assert generated['objective'] == state['incumbent_objective']
    assert generated['profile']['mip_calls'] == 0
    assert generated_pool['incumbent'] == state['incumbent']
    assert generated_pool['physical_paths'] == state['physical_paths']
    assert all(all(path in after for path in before)
               for before, after in zip(state['logical_paths'], generated_pool['logical_paths']))
    assert generated['profile']['path_generation_history'][-1]['terminal_optimal']
    subprocess.run([exe, instance, 'heuristic', '3', str(replay), '--pool-state', str(pg),
                    '--config', str(config)], check=True, capture_output=True)
    assert json.loads(replay.read_text())['objective'] <= generated['objective'] + 1e-6
    snapshot['instance'] = 'wrong instance'
    capture.write_text(json.dumps(snapshot))
    replay.unlink()
    proc = subprocess.run(command, capture_output=True)
    assert proc.returncode and not replay.exists()
print('CLI methods, configuration, and expired-deadline fallback passed')
