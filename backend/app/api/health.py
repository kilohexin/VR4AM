from fastapi import APIRouter

router = APIRouter()


@router.get("/health")
async def health() -> dict[str, object]:
    return {
        "status": "ok",
        "backend": "SIMULATOR",
        "real_robot_enabled": False,
    }
