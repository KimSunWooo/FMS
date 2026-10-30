import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PY_ROOTS = [ROOT / "backend" / "app", ROOT / "robot_agent", ROOT / "robot_receiver", ROOT / "simulation"]
# The Pinky ROS workspace is ROS code by design. Generated install/build trees
# also contain dangling symlinks and are not FMS source.
ROS_WORKSPACE = ROOT / "simulation" / "ros_ws"
# These two files are subprocess entry points. The API must not import them.
WORKER_ENTRYPOINTS = {
    ROOT / "backend" / "app" / "services" / "nav2_forward_5cm_worker.py",
    ROOT / "backend" / "app" / "services" / "live_tf_worker.py",
    ROOT / "backend" / "app" / "services" / "nav2_chain_probe.py",
    ROOT / "backend" / "app" / "services" / "map_identity_probe.py",
    ROOT / "robot_receiver" / "ros_gateway_node.py",
    ROOT / "robot_receiver" / "ros_task_node.py",
    ROOT / "robot_receiver" / "ros_messages.py",
    ROOT / "robot_receiver" / "nav2_executor_node.py",
}
BANNED = ("rmf", "rclpy", "nav2", "open_rmf", "rmf_fleet")


def _python_sources():
    for base in PY_ROOTS:
        if not base.exists():
            continue
        for path in base.rglob("*.py"):
            if path == ROS_WORKSPACE or ROS_WORKSPACE in path.parents:
                continue
            if path in WORKER_ENTRYPOINTS:
                continue
            if path.is_symlink() and not path.exists():
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except OSError:
                continue
            yield path, text


def test_sources_do_not_import_rmf_or_ros():
    offenders = []
    for path, text in _python_sources():
        tree = ast.parse(text)
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


def test_api_does_not_import_ros_worker_entrypoints():
    banned = {
        "app.services.nav2_forward_5cm_worker",
        "app.services.live_tf_worker",
        "app.services.nav2_chain_probe",
        "app.services.map_identity_probe",
        "robot_receiver.ros_gateway_node",
        "robot_receiver.ros_task_node",
        "robot_receiver.ros_messages",
        "robot_receiver.nav2_executor_node",
    }
    offenders = []
    for path, text in _python_sources():
        tree = ast.parse(text)
        for node in ast.walk(tree):
            modules = []
            if isinstance(node, ast.Import):
                modules = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules = [node.module]
            offenders.extend(f"{path}:{name}" for name in modules if name in banned)
    assert offenders == []


def test_pyproject_has_no_rmf_dependency():
    text = (ROOT / "backend" / "pyproject.toml").read_text(encoding="utf-8").lower()
    dependency_block = text.split("[project]", 1)[1].split("[project.optional-dependencies]", 1)[0]
    assert "rmf" not in dependency_block
    assert "rclpy" not in dependency_block
    assert "nav2" not in dependency_block
