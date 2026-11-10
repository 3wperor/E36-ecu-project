"""
Table-editing maths on plain numbers (no Qt): interpolate, smooth, and
clipboard text. Cells are addressed (row, col); values are in shown units.
"""


def _frac(axis, lo, hi, i):
    """Position of i between lo and hi, by the axis values if they are usable, else by index."""
    if axis is not None:
        a, b, x = axis[lo], axis[hi], axis[i]
        if a != b and min(a, b) <= x <= max(a, b):
            return (x - a) / (b - a)
    return (i - lo) / (hi - lo) if hi != lo else 0.0


def interpolate(values: dict, rows_axis=None, cols_axis=None) -> dict:
    """
    values: {(row, col): value} for a rectangular selection. The corner (or end)
    cells stay; everything between is interpolated: along a single row or column,
    or bilinearly from the four corners. Axis values (indexed by row / col) make
    the result follow unevenly spaced bins.
    """
    if not values:
        return {}
    rows = sorted({r for r, _ in values})
    cols = sorted({c for _, c in values})
    r0, r1, c0, c1 = rows[0], rows[-1], cols[0], cols[-1]
    if len(values) != (r1 - r0 + 1) * (c1 - c0 + 1):
        raise ValueError("select a rectangular block to interpolate")
    out = {}
    for (r, c) in values:
        fr = _frac(rows_axis, r0, r1, r)
        fc = _frac(cols_axis, c0, c1, c)
        top = values[(r0, c0)] + (values[(r0, c1)] - values[(r0, c0)]) * fc
        bot = values[(r1, c0)] + (values[(r1, c1)] - values[(r1, c0)]) * fc
        out[(r, c)] = top + (bot - top) * fr
    return out


def smooth(grid: dict, selected) -> dict:
    """
    Each selected cell moves toward the average of its neighbours (up, down,
    left, right): new = (4 x cell + neighbours) / (4 + number of neighbours).
    `grid` holds every cell of the table, so cells at the edge of the selection
    still see their unselected neighbours.
    """
    out = {}
    for (r, c) in selected:
        near = [grid[k] for k in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)) if k in grid]
        out[(r, c)] = (4 * grid[(r, c)] + sum(near)) / (4 + len(near))
    return out


def to_clipboard(rows) -> str:
    """Rows of numbers -> tab-separated text (pastes into Excel / LibreOffice)."""
    return "\n".join("\t".join(f"{v:g}" for v in row) for row in rows)


def from_clipboard(text: str):
    """Tab / semicolon / space separated text -> rows of floats. Decimal commas are accepted."""
    rows = []
    for line in text.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        if "\t" in line:
            cells = line.split("\t")
        elif ";" in line:
            cells = line.split(";")
        else:
            cells = line.split()
        row = []
        for cell in cells:
            cell = cell.strip()
            if "," in cell and "." not in cell:
                cell = cell.replace(",", ".")
            row.append(float(cell))             # ValueError for text that is not a number
        rows.append(row)
    if not rows:
        raise ValueError("the clipboard holds no numbers")
    return rows
