"""
The tune being edited: the raw page bytes, edits with undo/redo, tune files
and comparison. No Qt here; the GUI and the ECU link listen for changes.

Tune file (JSON, the same format tools/ecu_cli.py writes):
    signature, layout_hash, pages {id: hex}, values {name: shown value(s)}
`pages` is used when the layout matches; `values` (by name, in real units)
lets a file from another firmware layout be imported.
"""
import json
from contextlib import contextmanager


class TuneFileError(ValueError):
    pass


class Tune:
    UNDO_LIMIT = 500

    def __init__(self, definition, pages=None):
        self.d = definition
        self.pages = {p: bytearray(b) for p, b in (pages or definition.defaults).items()}
        self._check_sizes(self.pages)
        # f(changes) after every edit: changes = [(page, offset, bytes)]; [] = only the undo history changed
        self.listeners = []
        self.undo_stack = []        # (description, [(page, offset, old, new)])
        self.redo_stack = []
        self._group = None
        self._saved = self.snapshot()

    # ------------------------------------------------------------ reading --
    def _check_sizes(self, pages):
        for p, size in self.d.page_sizes.items():
            if p not in pages or len(pages[p]) != size:
                raise TuneFileError(f"page {p} should be {size} bytes")

    def snapshot(self) -> dict:
        return {p: bytes(b) for p, b in self.pages.items()}

    def field(self, name):
        return self.d.fields[name]

    def raw(self, name, index=0) -> int:
        f = self.d.fields[name]
        return f.read_raw(self.pages[f.page], index)

    def raws(self, name) -> list:
        f = self.d.fields[name]
        return [f.read_raw(self.pages[f.page], i) for i in range(f.count)]

    def shown(self, name, index=0) -> float:
        return self.d.fields[name].to_shown(self.raw(name, index))

    def get(self, name):
        """Shown value: option name for option lists, list for arrays, number otherwise."""
        f = self.d.fields[name]
        if f.bits:
            return f.options[self.raw(name)]
        vals = [f.to_shown(r) for r in self.raws(name)]
        return vals if f.shape else vals[0]

    # ------------------------------------------------------------ editing --
    @contextmanager
    def group(self, description):
        """Several edits as one undo step."""
        if self._group is not None:             # already inside a group
            yield
            return
        self._group = []
        try:
            yield
        finally:
            parts, self._group = self._group, None
            if parts:
                self._push(description, parts)
                for fn in list(self.listeners):     # the undo history changed (no new bytes)
                    fn([])

    def _push(self, description, parts):
        self.undo_stack.append((description, parts))
        del self.undo_stack[:-self.UNDO_LIMIT]
        self.redo_stack.clear()

    def _apply(self, parts, reverse=False):
        changes = []
        for page, offset, old, new in (reversed(parts) if reverse else parts):
            data = old if reverse else new
            self.pages[page][offset:offset + len(data)] = data
            changes.append((page, offset, bytes(data)))
        for fn in list(self.listeners):
            fn(changes)

    def _edit(self, description, page, offset, new):
        old = bytes(self.pages[page][offset:offset + len(new)])
        if old == new:
            return False
        part = (page, offset, old, bytes(new))
        if self._group is not None:
            self._group.append(part)
        else:
            self._push(description, [part])
        self._apply([part])
        return True

    def set_raw(self, name, values: dict, description=None) -> bool:
        """values = {index: raw}. Raw values are clamped to the field's range. One undo step."""
        f = self.d.fields[name]
        page = self.pages[f.page]
        writes = {}
        for i, raw in values.items():
            raw = min(f.raw_max, max(f.raw_min, int(raw)))
            if raw != f.read_raw(page, i):
                writes[i] = f.pack_raw(page, i, raw)
        if not writes:
            return False
        lo, hi = min(writes), max(writes)
        start, end = f.element_offset(lo), f.element_offset(hi) + f.item_size
        buf = bytearray(page[start:end])
        for i, b in writes.items():
            o = f.element_offset(i) - start
            buf[o:o + f.item_size] = b
        return self._edit(description or f"change {f.label}", f.page, start, bytes(buf))

    def set(self, name, value, description=None) -> bool:
        """
        Set from shown units: a number, a list (whole array), or an option name / index.
        Raises ValueError for anything outside the field's range.
        """
        f = self.d.fields[name]
        if f.bits:
            if isinstance(value, str):
                opts = [o.lower() for o in f.options]
                if value.lower() not in opts:
                    raise ValueError(f"{f.label}: choose one of {f.options}")
                value = opts.index(value.lower())
            if not 0 <= int(value) < len(f.options):
                raise ValueError(f"{f.label}: choose one of {f.options}")
            return self.set_raw(name, {0: int(value)}, description)
        vals = list(value) if isinstance(value, (list, tuple)) else [value]
        if len(vals) != f.count:
            raise ValueError(f"{f.label}: needs {f.count} values, got {len(vals)}")
        for v in vals:
            if not f.in_range(float(v)):
                raise ValueError(f"{f.label}: {v} is outside {f.format(f.min)} .. {f.format(f.max)} {f.units}")
        return self.set_raw(name, {i: f.to_raw(float(v)) for i, v in enumerate(vals)}, description)

    def replace(self, pages, description=None):
        """Replace every page. With a description it is one undoable step; without, history is cleared."""
        pages = {p: bytes(b) for p, b in pages.items()}
        self._check_sizes(pages)
        if description is None:
            self.undo_stack.clear()
            self.redo_stack.clear()
            parts = [(p, 0, bytes(self.pages[p]), pages[p]) for p in self.pages if self.pages[p] != pages[p]]
            self._apply(parts)
        else:
            with self.group(description):
                for p in self.pages:
                    self._edit(description, p, 0, pages[p])

    def can_undo(self):
        return bool(self.undo_stack)

    def can_redo(self):
        return bool(self.redo_stack)

    def undo(self):
        if not self.undo_stack:
            return None
        entry = self.undo_stack.pop()
        self.redo_stack.append(entry)
        self._apply(entry[1], reverse=True)
        return entry[0]

    def redo(self):
        if not self.redo_stack:
            return None
        entry = self.redo_stack.pop()
        self.undo_stack.append(entry)
        self._apply(entry[1])
        return entry[0]

    # -------------------------------------------------------------- files --
    @property
    def modified(self) -> bool:
        """Changed since the last save / load."""
        return self.snapshot() != self._saved

    def mark_saved(self):
        self._saved = self.snapshot()

    def to_file_dict(self) -> dict:
        return {"signature": self.d.signature, "layout_hash": self.d.layout_hash,
                "pages": {str(p): bytes(b).hex() for p, b in self.pages.items()},
                "values": {name: self.get(name) for name in self.d.fields}}

    def save(self, path):
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.to_file_dict(), fh, indent=1)
        self.mark_saved()


def read_tune_file(definition, path):
    """
    Returns (pages, notes). A file from the same layout loads exactly (notes is
    empty). A file from another layout is imported by setting name: `notes`
    lists everything that could not be carried over unchanged.
    """
    try:
        with open(path, encoding="utf-8") as fh:
            t = json.load(fh)
    except (OSError, ValueError) as e:
        raise TuneFileError(f"cannot read {path}: {e}") from e
    if not isinstance(t, dict) or "layout_hash" not in t:
        raise TuneFileError("this is not an E36 ECU tune file")

    if t["layout_hash"] == definition.layout_hash:
        try:
            pages = {int(p): bytes.fromhex(h) for p, h in t["pages"].items()}
        except (KeyError, ValueError, AttributeError) as e:
            raise TuneFileError("the tune file is damaged") from e
        Tune(definition, pages)                 # checks the page sizes
        return pages, []

    values = t.get("values")
    if not isinstance(values, dict):
        raise TuneFileError("the tune file is for another firmware layout and has no named values to import")
    tune = Tune(definition)                     # start from the defaults
    notes = []
    for name, f in definition.fields.items():
        if name not in values:
            notes.append(f"{f.label}: not in the file, left at its default")
            continue
        v = values[name]
        try:
            if f.bits:
                tune.set(name, v)
                continue
            vals = [float(x) for x in (v if isinstance(v, list) else [v])]
            if len(vals) != f.count:
                notes.append(f"{f.label}: the file has {len(vals)} values, this firmware {f.count}; left at its default")
                continue
            if not all(f.in_range(x) for x in vals):
                notes.append(f"{f.label}: limited to {f.format(f.min)} .. {f.format(f.max)} {f.units}".rstrip())
            tune.set_raw(name, {i: f.to_raw(x) for i, x in enumerate(vals)})
        except (ValueError, TypeError):
            notes.append(f"{f.label}: value {v!r} not understood, left at its default")
    for name in values:
        if name not in definition.fields:
            notes.append(f"{name}: not a setting in this firmware, ignored")
    if not notes:
        notes.append("Imported from another firmware layout; every setting carried over.")
    return tune.snapshot(), notes


def diff(definition, pages_a, pages_b):
    """Fields that differ: [{'field', 'indices', 'a', 'b'}] with shown values (option names for lists)."""
    out = []
    for f in definition.fields.values():
        ra = [f.read_raw(pages_a[f.page], i) for i in range(f.count)]
        rb = [f.read_raw(pages_b[f.page], i) for i in range(f.count)]
        idx = [i for i in range(f.count) if ra[i] != rb[i]]
        if not idx:
            continue
        show = (lambda r: f.options[r]) if f.bits else f.to_shown
        out.append({"field": f, "indices": idx, "a": [show(r) for r in ra], "b": [show(r) for r in rb]})
    return out
