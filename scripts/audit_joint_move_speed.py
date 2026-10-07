"""Analyze an existing session offline; never connect to a robot."""
from app.commissioning.joint_move_speed import main

if __name__ == '__main__':
    raise SystemExit(main())
