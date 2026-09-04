from pathlib import Path

from skye_data_recorder.data_recorder_node import next_episode_path


def test_next_episode_path_skips_existing(tmp_path: Path):
    (tmp_path / "episode_0000").mkdir()
    path = next_episode_path(str(tmp_path))
    assert path.name == "episode_0001"
