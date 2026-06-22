# OpenAMS config writeback — pure text editor for [filament_group] sections
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Klipper's SAVE_CONFIG only rewrites the main printer.cfg (its autosave block);
# it cannot edit an included subfile such as oams.cfg, where the OpenAMS sections
# live. So the plugin persists runtime group edits by surgically rewriting just
# the [filament_group ...] sections of its config file itself.
#
# This module is INTENTIONALLY PURE (no I/O): it transforms file text, so the
# rewrite rules are unit-testable. The manager handles the atomic read/write.
#
# Assumption: a group's `group:` value is a single line (a comma list), as in
# every real config and the shipped sample.

import re

_SECTION_RE = re.compile(r'^\s*\[([^\]]+)\]\s*$')
_GROUP_OPT_RE = re.compile(r'^(\s*)group\s*[:=].*$')


def _opt_re(option):
    return re.compile(r'^(\s*)%s\s*[:=].*$' % re.escape(option))


def _is_section_header(line):
    return _SECTION_RE.match(line) is not None


def _section_name(line):
    """The exact section name if `line` is a section header, else None."""
    m = _SECTION_RE.match(line)
    return m.group(1).strip() if m else None


def set_option(text, section, option, value):
    """Return `text` with `<option>: <value>` set in section `[<section>]`,
    preserving every other line. Replaces the option in place if present,
    inserts it after the header if absent, and appends the whole section if it
    does not exist. Used for OpenAMS config writeback (e.g. a calibrated
    ptfe_length on an [oams ...] section) which, like group edits, must land in
    the included subfile that SAVE_CONFIG cannot reach."""
    opt_re = _opt_re(option)
    had_trailing_nl = text.endswith("\n")
    lines = text.split("\n")
    if had_trailing_nl:
        lines = lines[:-1]

    out = []
    done = False
    i, n = 0, len(lines)
    while i < n:
        if not done and _section_name(lines[i]) == section:
            j = i + 1
            while j < n and not _is_section_header(lines[j]):
                j += 1
            sect = lines[i:j]
            out.extend(_set_in_section(sect, opt_re, option, value))
            done = True
            i = j
            continue
        out.append(lines[i])
        i += 1

    if not done:
        if out and out[-1].strip() != "":
            out.append("")
        out.append("[%s]" % section)
        out.append("%s: %s" % (option, value))

    result = "\n".join(out)
    if had_trailing_nl:
        result += "\n"
    return result


def _set_in_section(section, opt_re, option, value):
    out = [section[0]]
    replaced = False
    for line in section[1:]:
        m = opt_re.match(line)
        if m and not replaced:
            out.append("%s%s: %s" % (m.group(1), option, value))
            replaced = True
        else:
            out.append(line)
    if not replaced:
        out.insert(1, "%s: %s" % (option, value))
    return out


def _filament_group_name(line):
    """The group name if `line` is a [filament_group <name>] header, else None."""
    m = _SECTION_RE.match(line)
    if not m:
        return None
    parts = m.group(1).strip().split(None, 1)
    if len(parts) == 2 and parts[0] == "filament_group":
        return parts[1].strip()
    return None


def apply_group_edits(text, edits):
    """Return `text` with the given filament-group edits applied.

    edits: ordered list of (group_name, value) where `value` is the new `group:`
    option string, or None to delete the whole [filament_group <name>] section.
    An existing section's `group:` line is replaced in place (every other line —
    comments, blank lines, other options — is preserved); a group with no
    existing section is appended; a None value removes the section.
    """
    updates = {}
    order = []
    for name, value in edits:
        if name not in updates:
            order.append(name)
        updates[name] = value

    had_trailing_nl = text.endswith("\n")
    lines = text.split("\n")
    if had_trailing_nl:
        lines = lines[:-1]            # drop the empty field after the final \n

    out = []
    seen = set()
    i, n = 0, len(lines)
    while i < n:
        name = _filament_group_name(lines[i])
        if name is not None and name in updates:
            j = i + 1
            while j < n and not _is_section_header(lines[j]):
                j += 1
            section = lines[i:j]
            seen.add(name)
            if updates[name] is not None:
                out.extend(_rewrite_group_value(section, updates[name]))
            # else: delete -> drop the section's lines entirely
            i = j
            continue
        out.append(lines[i])
        i += 1

    for name in order:                # append groups that had no section yet
        value = updates[name]
        if name in seen or value is None:
            continue
        if out and out[-1].strip() != "":
            out.append("")
        out.append("[filament_group %s]" % name)
        out.append("group: %s" % value)

    result = "\n".join(out)
    if had_trailing_nl:
        result += "\n"
    return result


def _rewrite_group_value(section, value):
    out = [section[0]]                 # keep the header verbatim
    replaced = False
    for line in section[1:]:
        m = _GROUP_OPT_RE.match(line)
        if m and not replaced:
            out.append("%sgroup: %s" % (m.group(1), value))
            replaced = True
        else:
            out.append(line)
    if not replaced:
        out.insert(1, "group: %s" % value)
    return out
