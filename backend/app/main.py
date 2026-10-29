from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.agent import router as agent_router
from app.api.fleet import router as fleet_router
from app.api.physical import router as physical_router
from app.api.fleet_tasks import router as fleet_tasks_router
from app.api.planning import router as planning_router
from app.api.robots import install_error_handler
from app.api.robots import router as robots_router
from app.api.system import router as system_router
from app.api.tasks import router as tasks_router
from app.api.traffic import router as traffic_router
from app.core.config import Settings
from app.services.runtime import Runtime
from app.services.physical_nav2_auto import PhysicalNav2Auto
from app.services.domain_supervisor import DomainSupervisor
from app.services.forward_5cm_service import PinkyFiveCmTest  # FMS_PINKY1_5CM_TEST_V1


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or Settings()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        app.state.runtime.startup()
        app.state.physical_nav2 = None
        app.state.forward_5cm = None
        app.state.domain_supervisor = None
        nav2_task = None
        pose_task = None  # FMS_DIRECT_5CM_LIVE_POSE_V1
        domain_task = None
        if settings.domain_workers_enabled:
            app.state.domain_supervisor = DomainSupervisor(settings)
            domain_task = asyncio.create_task(app.state.domain_supervisor.run())
        if settings.physical_nav2_auto_start:
            app.state.physical_nav2 = PhysicalNav2Auto(settings)
            nav2_task = asyncio.create_task(app.state.physical_nav2.run())
            pose_task = asyncio.create_task(app.state.physical_nav2.run_live_pose())
            app.state.forward_5cm = PinkyFiveCmTest(settings, app.state.physical_nav2)
        try:
            yield
        finally:
            if pose_task is not None:
                pose_task.cancel()
                try:
                    await pose_task
                except asyncio.CancelledError:
                    pass
            if app.state.forward_5cm is not None:
                await app.state.forward_5cm.shutdown()
            if nav2_task is not None:
                nav2_task.cancel()
                try:
                    await nav2_task
                except asyncio.CancelledError:
                    pass
                await app.state.physical_nav2.shutdown()
            if domain_task is not None:
                domain_task.cancel()
                try:
                    await domain_task
                except asyncio.CancelledError:
                    pass
                await app.state.domain_supervisor.shutdown()
            app.state.runtime.shutdown()

    app = FastAPI(
        title="Care-Move FMS",
        version="0.1.0",
        summary="Pinky fleet manager without an Open-RMF runtime.",
        lifespan=lifespan,
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://127.0.0.1:8742",
            "http://localhost:8742",
        ],
        allow_methods=["GET", "POST"],
        allow_headers=["*"],
    )
    app.state.settings = settings
    app.state.runtime = Runtime(settings)
    install_error_handler(app)
    app.include_router(agent_router)
    app.include_router(robots_router)
    app.include_router(tasks_router)
    app.include_router(fleet_router)
    app.include_router(physical_router)
    app.include_router(planning_router)
    app.include_router(fleet_tasks_router)
    app.include_router(traffic_router)
    app.include_router(system_router)
    return app


app = create_app()
