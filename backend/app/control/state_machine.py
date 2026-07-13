from app.schemas.messages import TeleopMode


class TeleopStateMachine:
    def __init__(self) -> None:
        self.mode = TeleopMode.DISCONNECTED
        self._grip_released = False

    @property
    def can_arm(self) -> bool:
        return self.mode == TeleopMode.READY and self._grip_released

    def connect(self) -> None:
        if self.mode != TeleopMode.DISCONNECTED:
            raise RuntimeError("connect_requires_disconnected")
        self.mode = TeleopMode.READY
        self._grip_released = False

    def disconnect(self) -> None:
        self.mode = TeleopMode.DISCONNECTED
        self._grip_released = False

    def observe_grip(self, pressed: bool) -> None:
        if not pressed:
            self._grip_released = True
            if self.mode == TeleopMode.DISARMED:
                self.mode = TeleopMode.READY
            elif self.mode == TeleopMode.ACTIVE:
                self.mode = TeleopMode.HOLD
        elif self.mode in {TeleopMode.ARMED, TeleopMode.HOLD}:
            self.mode = TeleopMode.ACTIVE

    def arm(self) -> None:
        if not self.can_arm:
            raise RuntimeError("arm_requires_grip_release")
        self.mode = TeleopMode.ARMED

    def stale(self) -> None:
        self.mode = TeleopMode.STALE
        self._grip_released = False

    def stop_complete(self) -> None:
        if self.mode not in {TeleopMode.STALE, TeleopMode.FAULT}:
            raise RuntimeError("stop_complete_requires_stale_or_fault")
        self.mode = TeleopMode.DISARMED

    def fault(self) -> None:
        self.mode = TeleopMode.FAULT
        self._grip_released = False

    def disarm(self) -> None:
        self.mode = TeleopMode.DISARMED
        self._grip_released = False
