# Pure tests for the [filament_group] config writeback editor.
#
#   cd klipper_openams && python3 -m unittest discover -s test

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import oams_config_io as C


SAMPLE = """\
# OpenAMS config
[oams oams1]
mcu: oams_mcu1
oams_idx: 1

[filament_group T0]
group: oams1-0

[filament_group T1]
# a comment inside the section
group: oams1-1

[fps]
pin: fps:PA2
"""


class ApplyGroupEditsTests(unittest.TestCase):
    def test_update_value_in_place(self):
        out = C.apply_group_edits(SAMPLE, [("T0", "oams1-0,oams1-3")])
        self.assertIn("[filament_group T0]\ngroup: oams1-0,oams1-3\n", out)
        # everything else untouched
        self.assertIn("[oams oams1]", out)
        self.assertIn("mcu: oams_mcu1", out)
        self.assertIn("[fps]", out)
        self.assertIn("pin: fps:PA2", out)

    def test_update_preserves_other_lines_in_section(self):
        out = C.apply_group_edits(SAMPLE, [("T1", "oams1-2")])
        self.assertIn("# a comment inside the section", out)
        self.assertIn("group: oams1-2", out)
        self.assertNotIn("group: oams1-1", out)

    def test_create_appends_new_section(self):
        out = C.apply_group_edits(SAMPLE, [("T9", "oams1-2")])
        self.assertIn("[filament_group T9]\ngroup: oams1-2", out)
        # original groups intact
        self.assertIn("group: oams1-0", out)

    def test_delete_removes_section(self):
        out = C.apply_group_edits(SAMPLE, [("T0", None)])
        self.assertNotIn("[filament_group T0]", out)
        self.assertIn("[filament_group T1]", out)   # sibling kept
        self.assertIn("[oams oams1]", out)

    def test_lookalike_section_not_matched(self):
        # [filament_group_other thing] is NOT a [filament_group thing] section;
        # deleting group "thing" must leave it untouched (and there is no real
        # group "thing" to remove).
        text = "[filament_group_other thing]\ngroup: x\n"
        out = C.apply_group_edits(text, [("thing", None)])
        self.assertEqual(out, text)                  # unchanged

    def test_insert_group_option_when_absent(self):
        text = "[filament_group E]\n"
        out = C.apply_group_edits(text, [("E", "oams1-0")])
        self.assertIn("[filament_group E]\ngroup: oams1-0", out)

    def test_equals_separator_and_indent_preserved(self):
        text = "[filament_group T0]\n  group = oams1-0\n"
        out = C.apply_group_edits(text, [("T0", "oams1-1")])
        self.assertIn("  group: oams1-1", out)

    def test_no_trailing_newline_roundtrip(self):
        text = "[filament_group T0]\ngroup: oams1-0"     # no final newline
        out = C.apply_group_edits(text, [("T0", "oams1-1")])
        self.assertFalse(out.endswith("\n"))
        self.assertIn("group: oams1-1", out)

    def test_delete_then_recreate_is_idempotent_shape(self):
        gone = C.apply_group_edits(SAMPLE, [("T0", None)])
        back = C.apply_group_edits(gone, [("T0", "oams1-0")])
        self.assertIn("[filament_group T0]\ngroup: oams1-0", back)


class SetOptionTests(unittest.TestCase):
    def test_replace_existing_option(self):
        out = C.set_option(SAMPLE, "oams oams1", "oams_idx", "2")
        self.assertIn("oams_idx: 2", out)
        self.assertNotIn("oams_idx: 1", out)
        self.assertIn("mcu: oams_mcu1", out)        # other option preserved

    def test_insert_option_when_absent(self):
        out = C.set_option(SAMPLE, "oams oams1", "ptfe_length", "1234")
        self.assertIn("ptfe_length: 1234", out)
        self.assertIn("oams_idx: 1", out)           # existing options kept

    def test_only_named_section_touched(self):
        # set on [oams oams1]; [fps] and groups untouched
        out = C.set_option(SAMPLE, "oams oams1", "ptfe_length", "5")
        self.assertIn("[fps]\npin: fps:PA2", out)
        self.assertIn("group: oams1-0", out)

    def test_append_section_if_missing(self):
        out = C.set_option(SAMPLE, "oams oams2", "ptfe_length", "9")
        self.assertIn("[oams oams2]\nptfe_length: 9", out)


if __name__ == "__main__":
    unittest.main(verbosity=2)
