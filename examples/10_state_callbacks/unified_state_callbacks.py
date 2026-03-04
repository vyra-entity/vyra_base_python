"""
Example 10 — UnifiedStateMachine callbacks

Demonstrates:
- lifecycle/operational/health transitions
- per-layer and global callback registration
- transition history inspection
"""

from vyra_base.state import UnifiedStateMachine


def main() -> None:
    usm = UnifiedStateMachine()

    def on_any(layer: str, old: str, new: str):
        print(f"[ANY] {layer}: {old} -> {new}")

    def on_lifecycle(layer: str, old: str, new: str):
        print(f"[LIFECYCLE] {old} -> {new}")

    usm.on_any_change(on_any)
    usm.on_lifecycle_change(on_lifecycle)

    usm.start({"source": "example"})
    usm.complete_initialization({"ready": True})
    usm.set_ready({"source": "example"})
    usm.start_task({"task": "calibration"})
    usm.report_warning({"temp": "high"})
    usm.pause("operator requested")
    usm.resume()
    usm.stop({"result": "ok"})
    usm.recover({"action": "clear warning"})

    print("\nCurrent states:", usm.get_all_states())
    print("History length:", len(usm.get_history()))


if __name__ == "__main__":
    main()
