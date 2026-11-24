"""A progressing but unsynchronized sensor must not be called ROS-time fresh."""
from pathlib import Path
import importlib.util
import unittest

source=Path(__file__).resolve().parents[1]/'scripts/check_near_lidar.py'
spec=importlib.util.spec_from_file_location('near_lidar_audit',source)
audit=importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class NearLidarClockTests(unittest.TestCase):
    def test_fifteen_day_offset_is_reported_without_silent_clock_correction(self):
        result=audit.clock_diagnostics([(10.+i*.1,1309810.03+i*.1) for i in range(10)])
        self.assertTrue(result['device_time_progressing'])
        self.assertFalse(result['host_clock_compatible'])
        self.assertGreater(result['offset_seconds_min_median_max'][0],1.e6)

    def test_regular_host_synchronized_cloud_is_compatible(self):
        result=audit.clock_diagnostics([(10.+i*.1,10.04+i*.1) for i in range(10)])
        self.assertTrue(result['device_time_progressing'])
        self.assertTrue(result['host_clock_compatible'])

    def test_frozen_time_and_backward_reset_cannot_qualify_as_progress(self):
        for samples in ([(10.,10.03+i*.1) for i in range(10)],
                        [(10.,10.03),(10.1,10.13),(1.,10.23)]):
            self.assertFalse(audit.clock_diagnostics(samples)['device_time_progressing'])

    def test_no_samples_cannot_be_reported_as_fresh(self):
        self.assertFalse(audit.clock_diagnostics([])['host_clock_compatible'])
