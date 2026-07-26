from fastapi import APIRouter, Request

router = APIRouter()


@router.get("/health")
async def health(request: Request) -> dict[str, object]:
    state = request.app.state
    return {
        "status": "ok",
        "backend": state.backend_name,
        "real_robot_mode": state.real_robot_mode,
        "real_robot_enabled": state.real_robot_enabled,
        "preflight_ready": state.preflight_ready,
        "preflight_reason": state.preflight_reason,
    }
