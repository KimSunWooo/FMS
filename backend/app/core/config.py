from __future__ import annotations

from pathlib import Path

from pydantic import Field, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

ROOT = Path(__file__).resolve().parents[3]

_LEGACY_DB_MARKERS = (
    "care_rmf",
    "care-rmf",
    "db_care",
    "openrmf",
    "open-rmf",
    "open_rmf",
    "rmf_web",
    "rmf-web",
    "care-move-rmf-starter",
)

_PLACEHOLDER_SECRETS = {
    "replace-with-a-new-local-secret",
    "development-only-not-a-production-secret",
    "change-me",
    "",
}


class Settings(BaseSettings):
    """Process-local configuration.

    Physical motion and the ROS bridge cannot be turned on by environment
    variables in this build. A shared legacy database URL is refused.
    """

    model_config = SettingsConfigDict(
        env_prefix="FMS_",
        env_file=str(ROOT / ".env"),
        extra="ignore",
    )

    env: str = "development"
    environment_id: str = "care-move-fms-local"
    lease_secret: str = "development-only-not-a-production-secret"
    database_url: str = ""
    data_dir: Path = Field(default_factory=lambda: ROOT / "data")
    log_dir: Path = Field(default_factory=lambda: ROOT / "logs")
    zones_path: Path = Field(default_factory=lambda: ROOT / "config" / "zones.json")
    fleet_path: Path = Field(default_factory=lambda: ROOT / "config" / "fleet.json")
    ros_domain_id: int = 73
    api_host: str = "0.0.0.0"
    api_port: int = 8741
    physical_control_enabled: bool = False
    ros_bridge_enabled: bool = False
    # mock: in-process agents only. gazebo: external agent may claim a Nav2
    # send after preflight. physical: process refuses to start.
    execution_mode: str = "mock"
    physical_ssh_targets: str = ""
    physical_ssh_password: str = ""
    physical_nav2_auto_start: bool = False
    physical_nav2_poll_seconds: float = 6.0
    physical_nav2_robot_id: str = "pinky-1"
    # Per-robot workers use registry domains. This flag does not join those domains
    # from the API process itself.
    domain_workers_enabled: bool = True
    physical_map_yaml: Path = Field(
        default_factory=lambda: ROOT / "config/maps/physical_site/1f_map_f.yaml"
    )
    physical_fleet_path: Path = Field(
        default_factory=lambda: ROOT / "config" / "physical_fleet.json"
    )
    # 140 mm lattice: 110x120 mm body plus 20 mm margin on each axis.
    # The map file stays 0.01 m/px. This spacing is only the planner lattice.
    planning_resolution_m: float = 0.14
    planning_robot_radius_m: float = 0.096
    # Time slots stay at the previous 0.05 m / 0.15 m/s step. They do not grow with the lattice.
    planning_depart_slot_s: float = 0.05 / 0.15
    planning_max_candidates: int = 3
    planning_allow_diagonal: bool = True
    # Planning speed is a schedule estimate, not a measured Pinky velocity.
    planning_speed_m_s: float = 0.15
    planning_max_depart_slots: int = 8
    planning_goal_dwell_s: float = 0.5
    # Replans WAITING work only. It does not dispatch a task or send a Nav2 goal.
    fleet_replan_enabled: bool = True
    fleet_replan_interval_s: float = 5.0
    fleet_replan_max_attempts: int = 5
    # How long a simulated pickup arrival keeps occupancy while waiting for boarding.
    call_boarding_hold_s: float = 300.0
    fleet_dispatch_mode: str = "ros_topic"
    ros_task_command_topic: str = "/fms/task/command"
    ros_task_ack_topic: str = "/fms/task/ack"
    ros_task_cancel_topic: str = "/fms/task/cancel"
    ros_task_cancel_ack_topic: str = "/fms/task/cancel_ack"
    ros_robot_status_topic: str = "/fms/robot/status"
    ros_task_ack_timeout_s: float = 8.0
    ros_task_max_attempts: int = 2
    facilities_path: Path = Field(default_factory=lambda: ROOT / "config" / "facilities.json")
    physical_5cm_test_enabled: bool = False
    physical_5cm_test_token: str = ""
    physical_initial_pose_yaw_deg: float = 0.0
    physical_ssh_timeout_seconds: float = 3.0
    auto_tick: bool = True
    seed_fleet: bool = True
    process_lock: bool = True
    tick_interval_seconds: float = 0.25
    mock_goal_duration_seconds: float = 1.2
    heartbeat_timeout_seconds: float = 8.0
    cancel_timeout_seconds: float = 8.0
    lease_ttl_seconds: float = 30.0
    pose_tolerance_m: float = 0.25
    battery_min_percent: float = 15.0
    weight_travel: float = 1.0
    weight_battery: float = 0.05
    weight_congestion: float = 2.0
    weight_queue: float = 0.5
    congestion_radius_m: float = 2.0
    max_dispatch_attempts: int = 3
    dispatch_backoff_seconds: float = 1.0
    floor_width_m: float = 12.0
    floor_height_m: float = 8.0

    @model_validator(mode="after")
    def _guard(self) -> "Settings":
        if self.physical_control_enabled:
            raise ValueError(
                "This Phase 1 build refuses to start with physical control enabled."
            )
        if self.ros_bridge_enabled:
            raise ValueError(
                "This Phase 1 build refuses to start with the ROS bridge enabled."
            )
        if self.execution_mode not in {"mock", "gazebo", "physical"}:
            raise ValueError("FMS_EXECUTION_MODE must be mock, gazebo, or physical.")
        if self.execution_mode == "physical":
            raise ValueError("This build refuses to start in physical execution mode.")
        if not self.database_url:
            self.database_url = f"sqlite:///{self.data_dir / 'fms.sqlite3'}"
        haystack = self.database_url.lower()
        for marker in _LEGACY_DB_MARKERS:
            if marker in haystack:
                raise ValueError(
                    "Refusing a database URL that looks like a legacy Care-Move/RMF store."
                )
        if self.env.lower() == "production" and self.lease_secret in _PLACEHOLDER_SECRETS:
            raise ValueError("Refusing a placeholder lease secret when FMS_ENV=production.")
        if self.ros_domain_id < 0 or self.ros_domain_id > 232:
            raise ValueError("FMS_ROS_DOMAIN_ID must be in 0..232.")
        # Process environment overrides .env. A missing file is a configuration
        # error. Do not replace it with another map.
        map_path = Path(self.physical_map_yaml)
        if not map_path.is_file():
            official = ROOT / "config/maps/physical_site/1f_map_f.yaml"
            raise ValueError(
                f"FMS_PHYSICAL_MAP_YAML does not exist: {map_path}. "
                f"Official map: {official}. "
                "The process environment overrides .env. This build does not substitute another map."
            )
        return self

    @property
    def using_placeholder_secret(self) -> bool:
        return self.lease_secret in _PLACEHOLDER_SECRETS

    @property
    def distance_model(self) -> str:
        return "euclidean_phase1_not_obstacle_aware"
