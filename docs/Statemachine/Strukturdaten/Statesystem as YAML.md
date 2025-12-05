
```yaml
module_state_system:

  lifecycle:
    description: "Lifecycle states define existence and high-level life of the module."
    states:
      - Uninitialized
      - Initializing
      - Active
      - Recovering
      - ShuttingDown
      - Deactivated

    transitions:
      - from: Uninitialized
        event: start
        to: Initializing

      - from: Initializing
        event: init_success
        to: Active
      - from: Initializing
        event: init_failure
        to: Recovering

      - from: Active
        event: shutdown
        to: ShuttingDown
      - from: ShuttingDown
        event: finished
        to: Deactivated

      - from: Active
        event: fault_detected
        to: Recovering
      - from: Recovering
        event: recovery_success
        to: Active
      - from: Recovering
        event: recovery_failed
        to: Deactivated

  operational:
    description: "Operational states define the activity of a module during runtime."
    states:
      - Idle        # (Resting)
      - Ready       # (Attentive)
      - Running     # (Active)
      - Processing  # (Reflecting)
      - Delegating
      - Paused
      - Blocked
      - Completed

    transitions:
      - from: Idle
        event: ready
        to: Ready

      - from: Ready
        event: task_start
        to: Running

      - from: Running
        event: task_pause
        to: Paused
      - from: Paused
        event: resume
        to: Running

      - from: Running
        event: block_detected
        to: Blocked
      - from: Blocked
        event: unblock
        to: Running

      - from: Running
        event: delegate_to_other
        to: Delegating
      - from: Delegating
        event: delegate_done
        to: Running

      - from: Running
        event: background_processing
        to: Processing
      - from: Processing
        event: processing_done
        to: Running

      - from: Running
        event: task_complete
        to: Completed
      - from: Completed
        event: auto_reset
        to: Ready

      - from: "*"
        event: fault_detected
        to: Idle

  health:
    description: "Health states describe integrity and error conditions of the module."
    states:
      - OK
      - Warning           # (Alert)
      - Overloaded
      - Faulted
      - Critical

    transitions:
      - from: OK
        event: warn
        to: Warning

      - from: Warning
        event: overload
        to: Overloaded
      - from: Warning
        event: fault
        to: Faulted

      - from: Overloaded
        event: load_reduced
        to: Warning

      - from: Faulted
        event: recover
        to: OK
      - from: Faulted
        event: escalate
        to: Critical

      - from: Critical
        event: reset
        to: Faulted

  interrupt_handling:
    description: "Interrupts are global events, not states. They override the current FSM."
    events:
      - interrupt
      - emergency_stop
      - priority_override

    rules:
      - description: "Interrupt forces operational state into Paused or Blocked depending on context."
        mapping:
          Running: Paused
          Delegating: Paused
          Processing: Paused
          Ready: Ready
          Idle: Idle

      - description: "Critical interrupts update health state to Warning or Faulted."
        mapping:
          interrupt: Warning
          emergency_stop: Faulted
          priority_override: Warning
```