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

    def test_inline_comment_header_bounds_section(self):
        # Klipper strips '#'/';' comments before parsing, so
        # "[oams oams1] ; note" is a valid header. Deleting the group before
        # it must NOT swallow that section.
        text = ("[filament_group T0]\n"
                "group: oams1-0\n"
                "\n"
                "[oams oams1] ; main unit\n"
                "mcu: oams_mcu1\n")
        out = C.apply_group_edits(text, [("T0", None)])
        self.assertNotIn("[filament_group T0]", out)
        self.assertIn("[oams oams1] ; main unit", out)
        self.assertIn("mcu: oams_mcu1", out)
        # and lookups see through the comment too
        self.assertTrue(C.has_section(text, "oams oams1"))

    def test_multiline_value_fully_replaced(self):
        # configparser treats deeper-indented lines as value continuations;
        # replacing 'group:' must remove them or the old bays resurrect on
        # the next restart.
        text = ("[filament_group T0]\n"
                "group:\n"
                "  oams1-0,\n"
                "  oams1-1\n")
        out = C.apply_group_edits(text, [("T0", "oams1-2")])
        self.assertIn("group: oams1-2", out)
        self.assertNotIn("oams1-0", out)
        self.assertNotIn("oams1-1", out)

    def test_continuation_lookalike_not_matched(self):
        # An indented 'group:'-looking line inside ANOTHER option's value is a
        # continuation, not the option; the real one must be edited instead.
        text = ("[filament_group T0]\n"
                "info: line1\n"
                "  group: part-of-info\n"
                "group: oams1-0\n")
        out = C.apply_group_edits(text, [("T0", "oams1-3")])
        self.assertIn("  group: part-of-info", out)   # continuation untouched
        self.assertIn("group: oams1-3", out)
        self.assertNotIn("group: oams1-0", out)

    def test_crlf_preserved(self):
        text = "[filament_group T0]\r\ngroup: oams1-0\r\n"
        out = C.apply_group_edits(text, [("T0", "oams1-1")])
        self.assertEqual(out, "[filament_group T0]\r\ngroup: oams1-1\r\n")

    def test_multiword_section_matched_by_last_token(self):
        # Klipper names the object by the LAST token (get_name().split()[-1]),
        # so '[filament_group cool blue]' is group 'blue' — edits must target
        # it in place, not append a second section.
        text = "[filament_group cool blue]\ngroup: oams1-0\n"
        out = C.apply_group_edits(text, [("blue", "oams1-1")])
        self.assertEqual(out, "[filament_group cool blue]\ngroup: oams1-1\n")
        self.assertTrue(C.has_group(text, "blue"))


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

    def test_duplicate_sections_all_updated(self):
        # Duplicate sections merge with later-wins on parse; every copy
        # holding the option must be rewritten so the parsed value changes.
        text = ("[oams o1]\nptfe_length: 100\n\n"
                "[fps]\npin: x\n\n"
                "[oams o1]\nptfe_length: 100\n")
        out = C.set_option(text, "oams o1", "ptfe_length", "200")
        self.assertEqual(out.count("ptfe_length: 200"), 2)
        self.assertNotIn("ptfe_length: 100", out)

    def test_duplicate_sections_insert_goes_to_last(self):
        # If no copy holds the option, insert into the LAST one (later wins).
        text = ("[oams o1]\nmcu: a\n\n"
                "[oams o1]\nmcu: b\n")
        out = C.set_option(text, "oams o1", "ptfe_length", "9")
        self.assertEqual(out.count("ptfe_length: 9"), 1)
        self.assertGreater(out.index("ptfe_length: 9"), out.index("mcu: b"))

    def test_set_option_replaces_continuations(self):
        text = ("[oams o1]\n"
                "hub_hes_on:\n"
                "  0.1, 0.2,\n"
                "  0.3, 0.4\n"
                "mcu: a\n")
        out = C.set_option(text, "oams o1", "hub_hes_on", "0.5,0.5,0.5,0.5")
        self.assertIn("hub_hes_on: 0.5,0.5,0.5,0.5", out)
        self.assertNotIn("0.1", out)
        self.assertIn("mcu: a", out)                    # sibling option kept


if __name__ == "__main__":
    unittest.main(verbosity=2)
