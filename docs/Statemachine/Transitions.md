
**A) Lifecycle-State-Transitions**

|From|Event|To|
|---|---|---|
|Uninitialized|start()|Initializing|
|Initializing|init_success|Active|
|Initializing|init_failure|Recovering|
|Active|shutdown()|ShuttingDown|
|ShuttingDown|finished|Deactivated|
|Active|fault_detected|Recovering|
|Recovering|recovery_success|Active|
|Recovering|recovery_failed|Deactivated|

**B) Operational-State-Transitions**

|From|Event|To|
|---|---|---|
|Idle|ready()|Ready|
|Ready|task_start|Running|
|Running|pause|Paused|
|Running|block_detected|Blocked|
|Running|delegate_to_other|Delegating|
|Delegating|delegate_done|Running/Completed|
|Paused|resume|Running|
|Blocked|unblock|Running|
|Running|task_complete|Completed|
|Completed|auto_reset|Ready|
|Any|fault_detected|Idle or Blocked or Stopped (module-specific)|

**C) Health-State-Transitions**

|From|Event|To|
|---|---|---|
|OK|warning|Warning|
|Warning|overload|Overloaded|
|Warning|fault|Faulted|
|Overloaded|reduce_load|Warning|
|Faulted|recover|OK|
|Faulted|critical_failure|Critical|
|Critical|reset|Recovering|