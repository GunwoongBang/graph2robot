"""Shared plain-text validation logger for graph2robot thesis evaluation.

Emits ``[TAG]: message`` lines (same style as project.log) to stdout AND
appends them to a single shared log file so the metrics from every pipeline
node land in one place. This module is intentionally free of ROS dependencies
so it can be imported from any package (robot_task / robot_gazebo / robot_rviz).

Format
------
    [GRAPH2ROBOT]: <event / block header>
    [VALIDATION]: [n] <section title>
      <key>            : <value>          # 2-space-indented metric line
    --------------------------------------------------------------------------------

The log path defaults to the repository's ``src/project.log`` and can be
overridden with the ``GRAPH2ROBOT_VALIDATION_LOG`` environment variable (useful
when the workspace lives elsewhere). Precision conventions: distances in mm
(2 decimals), times in ms (1 decimal), angles in degrees (1 decimal).
"""

import os
import threading

DEFAULT_LOG_PATH = '/home/gunwoong/workspace/graph2robot/src/robot_validation/output/project.log'
RULE = '-' * 80

# One lock per process guards concurrent writes from a node's callbacks. Cross
# -process interleaving is fine: every block is self-describing and tagged.
_FILE_LOCK = threading.Lock()


class ValidationLog:
    """Thin formatter that mirrors project.log's ``[TAG]: message`` style."""

    def __init__(self, path: str | None = None) -> None:
        self._path = (
            path
            or os.environ.get('GRAPH2ROBOT_VALIDATION_LOG')
            or DEFAULT_LOG_PATH
        )

    # ── raw sink ──────────────────────────────────────────────────────────────
    def _write(self, line: str) -> None:
        print(line, flush=True)
        try:
            with _FILE_LOCK:
                with open(self._path, 'a') as f:
                    f.write(line + '\n')
        except OSError as exc:  # logging must never break the pipeline
            print(f'[VALIDATION]: (log-file write failed: {exc})', flush=True)

    # ── line kinds ────────────────────────────────────────────────────────────
    def event(self, message: str) -> None:
        """A ``[GRAPH2ROBOT]:`` event / block header line."""
        self._write(f'[GRAPH2ROBOT]: {message}')

    def header(self, message: str) -> None:
        """A ``[VALIDATION]: [n] Title`` section header line."""
        self._write(f'[VALIDATION]: {message}')

    def line(self, message: str) -> None:
        """A 2-space-indented ``key : value`` metric line under a header."""
        self._write(f'  {message}')

    def rule(self) -> None:
        """A dashed separator between major blocks."""
        self._write(RULE)

    # ── value formatters (keep precision consistent across nodes) ─────────────
    @staticmethod
    def mm(value: float | None) -> str:
        return 'N/A' if value is None else f'{float(value):.2f}'

    @staticmethod
    def deg(value: float | None) -> str:
        return 'N/A' if value is None else f'{float(value):.1f}'

    @staticmethod
    def ms(value: float | None) -> str:
        return 'N/A' if value is None else f'{float(value):.1f}'

    @staticmethod
    def xyz(vec, scale: float = 1.0, nd: int = 2) -> str:
        """Format a 3-vector as ``(x, y, z)`` applying an optional scale."""
        if vec is None or len(vec) < 3:
            return 'N/A'
        return (f'({float(vec[0]) * scale:.{nd}f}, '
                f'{float(vec[1]) * scale:.{nd}f}, '
                f'{float(vec[2]) * scale:.{nd}f})')
