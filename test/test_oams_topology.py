# Pure unit tests for oams_topology (configuration model + validation).
#
#   cd klipper_openams && python3 -m unittest discover -s test

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import oams_topology as T


def oams(name, idx, fps=None):
    return T.OamsSpec(name=name, idx=idx, fps=fps)


def single_lane():
    # One FPS, one OAMS (idx 1), four single-bay groups — the shipped layout.
    return T.build_topology(
        ["fps1"],
        [oams("oams1", 1)],
        [("T0", [("oams1", 0)]), ("T1", [("oams1", 1)]),
         ("T2", [("oams1", 2)]), ("T3", [("oams1", 3)])])


class BuildHappyPathTests(unittest.TestCase):
    def test_single_lane_resolves(self):
        t = single_lane()
        self.assertEqual(t.idx_of("oams1"), 1)
        self.assertEqual(t.lane_of_oams("oams1"), "fps1")
        self.assertEqual(t.lane_of_group("T0"), "fps1")
        self.assertEqual(t.group_bays_idx("T2"), ((1, 2),))
        self.assertEqual(t.oams_on_lane("fps1"), ("oams1",))

    def test_group_config_value_roundtrips(self):
        t = T.build_topology(["fps1"], [oams("oams1", 1)],
                             [("G", [("oams1", 0), ("oams1", 3)])])
        self.assertEqual(t.group_config_value("G"), "oams1-0,oams1-3")

    def test_multi_lane_explicit_fps(self):
        t = T.build_topology(
            ["fps1", "fps2"],
            [oams("oams1", 1, "fps1"), oams("oams2", 2, "fps2")],
            [("T0", [("oams1", 0)]), ("T1", [("oams2", 0)])])
        self.assertEqual(t.lane_of_group("T1"), "fps2")
        self.assertEqual(t.group_bays_idx("T1"), ((2, 0),))


class BuildErrorTests(unittest.TestCase):
    def _err(self, *args):
        with self.assertRaises(T.TopologyError) as cm:
            T.build_topology(*args)
        return str(cm.exception)

    def test_no_fps(self):
        self.assertIn("at least one FPS", self._err([], [], []))

    def test_oams_missing_fps_when_multiple(self):
        msg = self._err(["fps1", "fps2"], [oams("oams1", 1)], [])
        self.assertIn("must set 'fps:'", msg)

    def test_oams_unknown_fps(self):
        msg = self._err(["fps1"], [oams("oams1", 1, "fpsX")], [])
        self.assertIn("unknown fps", msg)

    def test_duplicate_oams_idx(self):
        msg = self._err(["fps1"], [oams("oams1", 1), oams("oams2", 1)], [])
        self.assertIn("unique oams_idx", msg)

    def test_group_unknown_oams(self):
        msg = self._err(["fps1"], [oams("oams1", 1)],
                        [("G", [("oams9", 0)])])
        self.assertIn("unknown OAMS", msg)

    def test_group_bay_out_of_range(self):
        msg = self._err(["fps1"], [oams("oams1", 1)],
                        [("G", [("oams1", 7)])])
        self.assertIn("out of range", msg)

    def test_group_spans_lanes(self):
        msg = self._err(
            ["fps1", "fps2"],
            [oams("oams1", 1, "fps1"), oams("oams2", 2, "fps2")],
            [("G", [("oams1", 0), ("oams2", 0)])])
        self.assertIn("spans FPS lanes", msg)

    def test_bay_in_two_groups(self):
        msg = self._err(["fps1"], [oams("oams1", 1)],
                        [("A", [("oams1", 0)]), ("B", [("oams1", 0)])])
        self.assertIn("only one filament_group", msg)

    def test_duplicate_group_name(self):
        msg = self._err(["fps1"], [oams("oams1", 1)],
                        [("G", [("oams1", 0)]), ("G", [("oams1", 1)])])
        self.assertIn("must be unique", msg)


class MutationTests(unittest.TestCase):
    def test_create_and_delete_group(self):
        t = single_lane()
        t2 = T.with_group(t, "NEW")
        self.assertIn("NEW", t2.groups)
        self.assertEqual(t2.groups["NEW"], ())
        self.assertNotIn("NEW", t.groups)               # immutable: original intact
        t3 = T.without_group(t2, "NEW")
        self.assertNotIn("NEW", t3.groups)

    def test_create_existing_group_errors(self):
        with self.assertRaises(T.TopologyError):
            T.with_group(single_lane(), "T0")

    def test_create_group_rejects_unwritable_names(self):
        # Runtime-created names must round-trip through a config file: no
        # empty/whitespace names, and nothing that breaks a section header or
        # gets eaten by Klipper's comment stripping.
        for bad in ("", " ", "two words", "a]b", "a[b", "a#b", "a;b",
                    "new\nline"):
            with self.assertRaises(T.TopologyError, msg=repr(bad)):
                T.with_group(single_lane(), bad)

    def test_create_group_accepts_sane_names(self):
        t = single_lane()
        for good in ("PLA", "T4", "red-petg", "user_group.2"):
            self.assertIn(good, T.with_group(t, good).groups)

    def test_assign_bay_moves_it_between_groups(self):
        t = single_lane()                                # bay (oams1,3) is in T3
        t2 = T.with_bay(t, "T0", "oams1", 3)
        self.assertIn(("oams1", 3), t2.groups["T0"])
        self.assertNotIn(("oams1", 3), t2.groups["T3"])  # moved, not copied
        self.assertEqual(t2.group_config_value("T0"), "oams1-0,oams1-3")

    def test_assign_unknown_group_errors(self):
        with self.assertRaises(T.TopologyError):
            T.with_bay(single_lane(), "NOPE", "oams1", 0)

    def test_assign_bay_enforces_single_lane(self):
        t = T.build_topology(
            ["fps1", "fps2"],
            [oams("oams1", 1, "fps1"), oams("oams2", 2, "fps2")],
            [("G", [("oams1", 0)])])
        # G is on fps1; adding an fps2 bay must be rejected.
        with self.assertRaises(T.TopologyError):
            T.with_bay(t, "G", "oams2", 0)

    def test_unassign_bay(self):
        t = T.with_bay(single_lane(), "T0", "oams1", 1)  # T0 now has bays 0,1
        t2 = T.without_bay(t, "T0", "oams1", 1)
        self.assertNotIn(("oams1", 1), t2.groups["T0"])

    def test_unassign_missing_bay_errors(self):
        with self.assertRaises(T.TopologyError):
            T.without_bay(single_lane(), "T0", "oams1", 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
