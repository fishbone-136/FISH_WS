"""A stopped estimator must withdraw latched live maps, not erase memory."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from obstacle_memory import RememberedObstacles
from environment_terrain_display import EnvironmentTerrainDisplay


def fixture():
    source = OccupancyGrid()
    source.header.frame_id = 'odom_lio'
    source.info.width, source.info.height, source.info.resolution = 2, 2, .08
    source.info.origin.position.x = 1.2
    source.info.origin.orientation.w = 1.
    source.data = [100, 0, -1, 90]
    stamp = source.header.stamp
    node = SimpleNamespace(localized=False, pose=(0.,0.,0.), fault=None,
        maps={'local': source}, map_resolutions={'local': .04}, published_maps={},
        map_publishers={'local': Mock()}, cpu_budget=SimpleNamespace(last_jobs={}),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)),
        memory=SimpleNamespace(cells={(1,1): ('keep',)}))
    return node, source


def test_invalid_localization_replaces_occupied_and_free_with_unknown():
    node, source = fixture()
    RememberedObstacles.publish_maps(node, force=True)
    out = node.map_publishers['local'].publish.call_args.args[0]
    assert (out.info.width, out.info.height) == (4,4)
    assert np.all(np.asarray(out.data) == -1)
    assert out.info.origin.position.x == source.info.origin.position.x
    assert list(source.data) == [100,0,-1,90]
    assert node.memory.cells == {(1,1): ('keep',)}
    RememberedObstacles.publish_maps(node, force=True)
    assert node.map_publishers['local'].publish.call_count == 1


def test_fault_with_valid_flag_also_withdraws_old_map():
    node, _ = fixture(); node.localized = True; node.fault = 'worker failed'
    RememberedObstacles.publish_maps(node, force=True)
    assert set(node.map_publishers['local'].publish.call_args.args[0].data) == {-1}


def test_localization_loss_publishes_immediately():
    node = SimpleNamespace(localized=True, publish_maps=Mock())
    RememberedObstacles.on_localization_valid(node, Bool(data=False))
    assert node.localized is False
    node.publish_maps.assert_called_once_with(force=True)


def test_historical_color_overlay_is_cleared_and_ignores_queued_old_clouds():
    stamp = OccupancyGrid().header.stamp
    node = SimpleNamespace(localized=True, output=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)))
    EnvironmentTerrainDisplay.on_localization_valid(node, Bool(data=False))
    out = node.output.publish.call_args.args[0]
    assert out.width == 0 and len(out.data) == 0
    assert [f.name for f in out.fields] == ['x','y','z','intensity']
    EnvironmentTerrainDisplay.on_cloud(node, PointCloud2())
    assert node.output.publish.call_count == 1
