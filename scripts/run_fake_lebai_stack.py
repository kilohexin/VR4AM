from app.digital_twin.runtime import create_digital_twin_app

import uvicorn


def main() -> int:
    uvicorn.run(
        create_digital_twin_app(),
        host="127.0.0.1",
        port=8000,
        log_level="info",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
