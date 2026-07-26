class NoopRecorder:
    async def start(self) -> None:
        return None

    async def write_vr_frame(self, frame: object, server_mono_ns: int) -> None:
        return None

    async def write_robot_state(self, state: object, server_mono_ns: int) -> None:
        return None

    async def write_event(self, event: object, server_mono_ns: int) -> None:
        return None

    async def write_critical_event(
        self,
        kind: str,
        payload: object,
        server_mono_ns: int,
    ) -> None:
        return None

    async def write_camera_frame(self, frame: object) -> None:
        return None

    async def close(self) -> None:
        return None
