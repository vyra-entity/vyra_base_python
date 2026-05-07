"""
Example: Unified State Machine (3-Layer FSM)

Demonstrates the UnifiedStateMachine combining:
- Lifecycle layer (Offline → Initializing → Active → ShuttingDown → Offline)
- Operational layer (Idle → Ready → Running / Paused / Stopped / Error)
- Health layer (OK → Warning → Error)

Usage:
  python examples/unified_state_machine_example.py
"""

from vyra_base.state import UnifiedStateMachine


def main():
    print("=" * 60)
    print("Unified State Machine Example (3 Layers)")
    print("=" * 60)

    usm = UnifiedStateMachine()
    print("\n1. Initial state (after creation)")
    print("   ", usm.get_all_states())

    # --- Lifecycle ---
    print("\n2. Lifecycle: start() → Offline to Initializing")
    usm.start()
    print("   ", usm.get_all_states())

    print("\n3. Lifecycle: complete_initialization() → Active")
    usm.complete_initialization()
    print("   ", usm.get_all_states())

    # --- Operational ---
    print("\n4. Operational: set_ready() → Idle to Ready")
    usm.set_ready()
    print("   ", usm.get_all_states())

    print("\n5. Operational: start_task() → Running")
    usm.start_task({"task_id": "demo-1"})
    print("   ", usm.get_all_states())

    print("\n6. Health: report_warning() → Health Warning")
    usm.report_warning({"cpu": "85%"})
    print("   ", usm.get_all_states())

    print("\n7. Health: clear_warning() → Health OK")
    usm.health.clear_warning()
    print("   ", usm.get_all_states())

    print("\n8. Operational: pause() → Paused")
    usm.pause("user requested")
    print("   ", usm.get_all_states())

    print("\n9. Operational: resume() → Ready, then start_task() → Running")
    usm.resume()
    usm.start_task({"task_id": "demo-2"})
    print("   ", usm.get_all_states())

    print("\n10. Operational: stop() → Stopped")
    usm.stop()
    print("   ", usm.get_all_states())

    print("\n11. Operational: reset() → Idle")
    usm.reset()
    print("   ", usm.get_all_states())

    # --- Helpers ---
    print("\n12. Helpers")
    print("    is_operational():", usm.is_operational())
    print("    is_healthy():", usm.is_healthy())
    print("    get_lifecycle_state():", usm.get_lifecycle_state())
    print("    get_operational_state():", usm.get_operational_state())
    print("    get_health_state():", usm.get_health_state())

    # --- Shutdown (optional) ---
    print("\n13. Lifecycle: shutdown() → ShuttingDown, complete_shutdown() → Offline")
    usm.shutdown("example finished")
    usm.complete_shutdown()
    print("   ", usm.get_all_states())

    print("\n" + "=" * 60)
    print("Unified State Machine example completed.")
    print("=" * 60)


if __name__ == "__main__":
    main()
