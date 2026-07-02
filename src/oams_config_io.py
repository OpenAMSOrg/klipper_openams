# OpenAMS config writeback — pure text editor for the OpenAMS config file
#
# Copyright (C) 2025-2026 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Klipper's SAVE_CONFIG only rewrites the main printer.cfg (its autosave block);
# it cannot edit an included subfile such as oams.cfg, where the OpenAMS sections
# live. So the plugin persists runtime group edits and calibration results by
# surgically rewriting its config file itself.
#
# This module is INTENTIONALLY PURE (no I/O): it transforms file text, so the
# rewrite rules are unit-testable. The manager handles the atomic read/write.
#
# The transforms mirror how Klipper actually reads a config:
# - Section headers start at column 0 and may carry a trailing inline comment
#   ("[oams oams1] ; main unit") — Klipper strips '#'/';' comments before
#   parsing, so such headers are valid and must bound sections here too.
# - A non-blank line indented deeper than the option line before it is a
#   CONTINUATION of that option's value, not a new option. Replacing an option
#   therefore replaces its continuation lines as well, and an indented
#   lookalike inside another option's value is never matched.
# - Duplicate sections merge on parse with later-wins semantics; edits are
#   applied to every duplicate (and inserts go into the last one) so the
#   parsed result always reflects the edit.
# - Section-name matching for [filament_group ...] uses the LAST whitespace
#   token, matching Klipper's get_name().split()[-1] convention used at load.

import re

# Column-0 header with optional trailing inline comment (Klipper strips
# comments before configparser sees the line, so this is a legal header).
_SECTION_RE = re.compile(r'^\[([^\]]+)\]\s*(?:[#;].*)?$')


def _opt_re(option):
    return re.compile(r'^(\s*)%s\s*[:=].*$' % re.escape(option))


def _is_section_header(line):
    return _SECTION_RE.match(line) is not None


def _section_name(line):
    """The exact section name if `line` is a section header, else None."""
    m = _SECTION_RE.match(line)
    return m.group(1).strip() if m else None


def _filament_group_name(line):
    """The group name if `line` is a [filament_group <name>] header, else None.
    Uses the last whitespace token, matching the runtime name Klipper derives
    via get_name().split()[-1] (so '[filament_group cool blue]' is group
    'blue' both at load time and here)."""
    name = _section_name(line)
    if name is None:
        return None
    tokens = name.split()
    if len(tokens) >= 2 and tokens[0] == "filament_group":
        return tokens[-1]
    return None


def has_section(text, section):
    """True when `[<section>]` exists in `text`. Used to refuse a writeback
    that would otherwise APPEND a section which really lives in a different
    config file (e.g. printer.cfg): duplicate sections across includes merge
    with later-wins semantics, so an appended copy silently diverges from (or
    silently overrides) the real one depending on include order."""
    lines, _nl, _had = _split_lines(text)
    return any(_section_name(line) == section for line in lines)


def has_group(text, name):
    """True when a [filament_group <name>] section exists in `text`."""
    lines, _nl, _had = _split_lines(text)
    return any(_filament_group_name(line) == name for line in lines)


# ------------------------------------------------------------ line plumbing

def _split_lines(text):
    """Split into lines, remembering the newline style and trailing newline so
    the transform can round-trip CRLF files without mixing line endings."""
    newline = "\r\n" if "\r\n" in text else "\n"
    body = text.replace("\r\n", "\n")
    had_trailing_nl = body.endswith("\n")
    lines = body.split("\n")
    if had_trailing_nl:
        lines = lines[:-1]            # drop the empty field after the final \n
    return lines, newline, had_trailing_nl


def _join_lines(lines, newline, had_trailing_nl):
    result = "\n".join(lines)
    if had_trailing_nl:
        result += "\n"
    if newline != "\n":
        result = result.replace("\n", newline)
    return result


def _find_sections(lines, matches):
    """[(start, end)] extents (header..next header) of sections whose header
    satisfies `matches(header_line)`."""
    out = []
    i, n = 0, len(lines)
    while i < n:
        if _is_section_header(lines[i]) and matches(lines[i]):
            j = i + 1
            while j < n and not _is_section_header(lines[j]):
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


def _option_starts(section):
    """Indices (into `section`, whose element 0 is the header) of lines that
    START an option. Mirrors configparser: a non-blank line indented deeper
    than the current option line continues that option's value."""
    starts = []
    cur_indent = None
    for i in range(1, len(section)):
        line = section[i]
        if not line.strip():
            continue                   # blank: may sit inside a value
        indent = len(line) - len(line.lstrip())
        if cur_indent is not None and indent > cur_indent:
            continue                   # continuation of the previous option
        cur_indent = indent
        starts.append(i)
    return starts


def _section_has_option(section, opt_re):
    return any(opt_re.match(section[i]) for i in _option_starts(section))


def _set_in_section(section, option, value):
    """Replace `option` (and its continuation lines) with a single-line value,
    preserving every other line. Inserts at the end of the section when the
    option is absent (column 0, so it can never be swallowed as a continuation
    of an indented option above it)."""
    opt_re = _opt_re(option)
    starts = _option_starts(section)
    target = next((i for i in starts if opt_re.match(section[i])), None)
    if target is None:
        end = len(section)
        while end - 1 >= 1 and not section[end - 1].strip():
            end -= 1                   # keep trailing blanks as spacing
        return section[:end] + ["%s: %s" % (option, value)] + section[end:]
    later = [i for i in starts if i > target]
    end = later[0] if later else len(section)
    while end - 1 > target and not section[end - 1].strip():
        end -= 1                       # keep trailing blanks, drop continuations
    indent = section[target][:len(section[target])
                             - len(section[target].lstrip())]
    return (section[:target]
            + ["%s%s: %s" % (indent, option, value)]
            + section[end:])


# ================================================================== editors

def apply_group_edits(text, edits):
    """Return `text` with the given filament-group edits applied.

    edits: ordered list of (group_name, value) where `value` is the new `group:`
    option string, or None to delete the whole [filament_group <name>] section.
    An existing section's `group:` option (with any continuation lines) is
    replaced in place — every other line, comment and option is preserved; a
    group with no existing section is appended; a None value removes the
    section. Duplicate sections for the same group are all updated/removed.
    """
    updates = {}
    order = []
    for name, value in edits:
        if name not in updates:
            order.append(name)
        updates[name] = value

    lines, newline, had_trailing_nl = _split_lines(text)
    out = []
    seen = set()
    i, n = 0, len(lines)
    while i < n:
        name = _filament_group_name(lines[i])
        if name is not None and name in updates:
            j = i + 1
            while j < n and not _is_section_header(lines[j]):
                j += 1
            seen.add(name)
            if updates[name] is not None:
                out.extend(_set_in_section(lines[i:j], "group", updates[name]))
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

    return _join_lines(out, newline, had_trailing_nl)


def set_option(text, section, option, value):
    """Return `text` with `<option>: <value>` set in section `[<section>]`,
    preserving every other line. Replaces the option (and its continuation
    lines) in place where present; when the section is duplicated, every copy
    holding the option is updated and, if none holds it, the LAST copy gets
    the insert (later-wins parse semantics). Appends the whole section if it
    does not exist. Used for OpenAMS config writeback (e.g. a calibrated
    ptfe_length on an [oams ...] section) which, like group edits, must land
    in the included subfile that SAVE_CONFIG cannot reach."""
    lines, newline, had_trailing_nl = _split_lines(text)
    secs = _find_sections(lines, lambda hdr: _section_name(hdr) == section)
    if not secs:
        out = list(lines)
        if out and out[-1].strip() != "":
            out.append("")
        out.append("[%s]" % section)
        out.append("%s: %s" % (option, value))
        return _join_lines(out, newline, had_trailing_nl)

    opt_re = _opt_re(option)
    holding = [(s, e) for (s, e) in secs
               if _section_has_option(lines[s:e], opt_re)]
    targets = set(holding) if holding else {secs[-1]}

    out = []
    prev = 0
    for (s, e) in secs:
        out.extend(lines[prev:s])
        seg = lines[s:e]
        out.extend(_set_in_section(seg, option, value)
                   if (s, e) in targets else seg)
        prev = e
    out.extend(lines[prev:])
    return _join_lines(out, newline, had_trailing_nl)
