import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PY_ROOTS = [ROOT / "backend" / "app", ROOT / "robot_agent", ROOT / "simulation"]
BANNED = ("rmf", "rclpy", "nav2", "open_rmf", "rmf_fleet")


def test_sources_do_not_import_rmf_or_ros():
    offenders = []
    for base in PY_ROOTS:
        for path in base.rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                names = []
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module:
                    names = [node.module]
                for name in names:
                    lowered = name.lower()
                    if any(lowered == banned or lowered.startswith(banned + ".") for banned in BANNED):
                        offenders.append(f"{path}:{name}")
    assert offenders == []


def test_pyproject_has_no_rmf_dependency():
    text = (ROOT / "backend" / "pyproject.toml").read_text(encoding="utf-8").lower()
    dependency_block = text.split("[project]", 1)[1].split("[project.optional-dependencies]", 1)[0]
    assert "rmf" not in dependency_block
    assert "rclpy" not in dependency_block
    assert "nav2" not in dependency_block
