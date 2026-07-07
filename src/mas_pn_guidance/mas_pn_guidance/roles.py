"""Role -> PX4 namespace resolution for the PN interception harness.

Decouples logical roles (interceptor, target, observer_a, ...) from PX4
namespaces (px4_1, px4_2, ...) so the cooperative layout is a config edit rather
than a code change. Pure-Python (only PyYAML) so the conductor, launch files, and
CLI tools can all import it without a ROS dependency.

Config: ``mas_pn_guidance/config/roles.yaml`` (installed to the package share
dir). A layout is a ``role -> namespace`` dict; ``active`` selects one.

CLI (entry point ``roles``)::

    ros2 run mas_pn_guidance roles                 # print the active layout
    ros2 run mas_pn_guidance roles --role interceptor   # print one namespace
    ros2 run mas_pn_guidance roles --layout cooperative # inspect another layout
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, Optional

import yaml


def _default_roles_path() -> str:
    """Locate roles.yaml, preferring the installed share dir, then the source tree."""
    # Installed location (ament resource): <prefix>/share/mas_pn_guidance/config/roles.yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('mas_pn_guidance')
        candidate = os.path.join(share, 'config', 'roles.yaml')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    # Source-tree fallback (running before a build / from a checkout).
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'config', 'roles.yaml'))


class Roles:
    """Resolved role<->namespace map for one active layout."""

    def __init__(self, mapping: Dict[str, str], layout_name: str):
        self.layout_name = layout_name
        self._by_role: Dict[str, str] = dict(mapping)
        self._by_ns: Dict[str, str] = {ns: role for role, ns in mapping.items()}

    @classmethod
    def load(cls, path: Optional[str] = None, layout: Optional[str] = None) -> 'Roles':
        path = path or _default_roles_path()
        with open(path, 'r') as fh:
            doc = yaml.safe_load(fh) or {}
        layouts = doc.get('layouts') or {}
        if not layouts:
            raise ValueError(f"roles.yaml has no 'layouts': {path}")
        name = layout or doc.get('active')
        if name not in layouts:
            raise KeyError(
                f"layout '{name}' not in roles.yaml (have: {sorted(layouts)})")
        mapping = layouts[name] or {}
        return cls(mapping, name)

    def namespace(self, role: str) -> str:
        """PX4 namespace for a logical role (e.g. 'interceptor' -> 'px4_1')."""
        if role not in self._by_role:
            raise KeyError(
                f"role '{role}' not in layout '{self.layout_name}' "
                f"(have: {sorted(self._by_role)})")
        return self._by_role[role]

    def role(self, namespace: str) -> str:
        """Logical role for a PX4 namespace (reverse lookup)."""
        if namespace not in self._by_ns:
            raise KeyError(
                f"namespace '{namespace}' not in layout '{self.layout_name}'")
        return self._by_ns[namespace]

    def as_dict(self) -> Dict[str, str]:
        return dict(self._by_role)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Resolve PN harness roles.")
    parser.add_argument('--path', default=None, help="roles.yaml path override")
    parser.add_argument('--layout', default=None, help="layout name override")
    parser.add_argument('--role', default=None,
                        help="print only this role's namespace")
    args = parser.parse_args(argv)

    roles = Roles.load(path=args.path, layout=args.layout)
    if args.role:
        print(roles.namespace(args.role))
        return 0
    print(f"layout: {roles.layout_name}")
    for role, ns in roles.as_dict().items():
        print(f"  {role:12s} -> {ns}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
