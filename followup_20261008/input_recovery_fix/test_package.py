from pathlib import Path
import importlib.util
import sys
import pytest
H=Path(__file__).resolve().parent

def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def test_recording_adds_diagnostics_without_changing_required_topics_or_storage():
 before=load('before_profile',H/'baseline/drive_debug_profile.py');after=load('after_profile',H/'candidate/drive_debug_profile.py')
 assert set(after.TOPICS)-set(before.TOPICS)=={'/knu_batch/run_permission','/knu_batch/recorder_ready','/knu_batch/abort','/px4/sih/odom','/px4_ekf_bridge/status'}
 assert set(before.TOPICS)<=set(after.TOPICS) and after.REQUIRED_TOPICS==before.REQUIRED_TOPICS
 after.PACKAGE=before.PACKAGE
 assert after.recorder_command('/tmp/bag')[:-len(after.TOPICS)]==before.recorder_command('/tmp/bag')[:-len(before.TOPICS)]

@pytest.fixture
def package(tmp_path,monkeypatch):
 m=load('promotion',H/'promote.py');monkeypatch.setattr(m,'DEST',tmp_path);monkeypatch.setattr(m,'active_nodes',lambda:[])
 for n in m.FILES:(tmp_path/n).write_bytes((H/'baseline'/n).read_bytes())
 return m

def test_apply_and_rollback_restore_exact_preexisting_bytes(package,monkeypatch):
 m=package;monkeypatch.setattr(sys,'argv',['promote.py','--apply']);m.main()
 assert all((m.DEST/n).read_bytes()==(H/'candidate'/n).read_bytes() for n in m.FILES)
 monkeypatch.setattr(sys,'argv',['promote.py','--rollback']);m.main()
 assert all((m.DEST/n).read_bytes()==(H/'baseline'/n).read_bytes() for n in m.FILES)

def test_refuses_to_overwrite_later_user_edit(package,monkeypatch):
 m=package;p=m.DEST/m.FILES[1];p.write_text('new user edit');monkeypatch.setattr(sys,'argv',['promote.py','--apply'])
 with pytest.raises(SystemExit,match='Current file changed'):m.main()
 assert p.read_text()=='new user edit'
 assert (m.DEST/m.FILES[0]).read_bytes()==(H/'baseline'/m.FILES[0]).read_bytes()

def test_running_nodes_block_promotion(package,monkeypatch):
 m=package;monkeypatch.setattr(m,'active_nodes',lambda:[('123',['knu_batch_runner.py'])]);monkeypatch.setattr(sys,'argv',['promote.py','--apply'])
 with pytest.raises(SystemExit,match='processes are active'):m.main()
 assert all((m.DEST/n).read_bytes()==(H/'baseline'/n).read_bytes() for n in m.FILES)
