# Fake Lebai offline acceptance

This procedure validates the LM3 software path in simulation. It
does not connect to a robot, import or call the real Lebai SDK, request a
real-robot confirmation, or verify a physical safety property. The generated
report always identifies the runtime as `LEBAI_FAKE`, sets `digital_twin` to
`true`, and sets `hardware_verified` to `false`.

## Start the offline stack

From the repository root, start the Fake Lebai backend in one terminal:

```powershell
python scripts/run_fake_lebai_stack.py
```

In a second terminal, start the frontend so a Quest on the trusted local
network can load the UI:

```powershell
cd web
npm.cmd run dev -- --host 0.0.0.0
```

Open the frontend from the PC or Quest and confirm the identity shown by the
HUD/diagnostics is `LEBAI_FAKE`. Only one controller is allowed: close the PC
control page before using Quest, and close the Quest page before using PC.
There is no observer or second control connection; the current owner receives
robot state and diagnostics on the same WebSocket.

For a Quest session, release the controls before requesting arm, hold Grip to
establish a reference, move slowly, then release Grip and confirm a stop.
Close the Quest page before handing control back to the PC.

## Read diagnostics and exercise fault handling

The PC rail and Quest HUD show the runtime identity and control state. The
diagnostics payload also includes actual and target joint velocity/
acceleration, target TCP, SDK latency fields (`get_kin_data` and
`move_pvat`), PVAT send rate, recorder session location, dropped-event count,
and a bounded recent-event stream. Treat a fault, stale state, or ownership
rejection as a stop/lock condition; do not use page refresh to bypass it.

The acceptance scenarios inject and verify five digital-twin faults:
IK failure, PVAT write failure, disconnect, stale snapshot, and stop failure.
They also cover signed translation and rotation, gripper open/close, Home,
and repeated stop. These are software-path checks, not a substitute for an
onsite machine, gripper, E-stop, direction, or latency verification.

## Run the offline gate

From the repository root, run the complete acceptance command once:

```powershell
python scripts/accept_fake_lebai.py
```

It runs the full backend pytest suite, full frontend Vitest suite, Vite
production build, and the Fake Lebai scenarios. It atomically writes
`artifacts/acceptance/fake-lebai-latest.json`, including Git provenance and
SHA-256 hashes of the kinematics JSON, LM3 GLB, and Fake Lebai configuration.
The command exits nonzero for a command or scenario failure.

The report leaves these hardware categories pending: SDK connection,
TCP/Home/joint limits, translation direction, rotation direction, gripper
direction/force, PVAT tracking latency, stop distance/E-stop, and lightweight
grasp/release. Perform those checks only through the staged onsite procedure
in `docs/real-robot-deployment.md`; this gate never marks them verified.
