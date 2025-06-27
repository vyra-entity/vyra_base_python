# Lifecycle

## States

- **Resting**  
    Module is inactive, gathering energy, and not performing any active tasks.

- **Awakening**  
    Module is starting up, loading resources, and preparing for operation.

- **Attentive**  
    Module is awake and ready to receive input, but not yet actively engaged.

- **Active**  
    Module is actively working, performing tasks, and interacting with its environment.

- **Reflecting**  
    Module is processing information in the background, analyzing, and planning next steps.

- **Learning**  
    Module is self-training, optimizing models, and adapting to new requirements.

- **Alert**  
    Module is vigilant and ready to respond immediately to specific events or dangers.

- **Delegating**  
    Module hands off tasks to other modules or systems and waits for their results.

- **Overloaded**  
    Module is overloaded or in an error state and requires intervention.

- **Recovering**  
    Module is recovering from errors, repairing itself, and preparing to restart.

- **ShuttingDown**  
    Module is actively terminating processes, saving data, and shutting down in a controlled manner.

- **Interrupting**  
    Module is moved into this state from any other by an external event (e.g., emergency, urgent interruption). In this state, the module temporarily stops its current activity, prioritizes handling the disruption, and prepares to either return to the previous state or transition to another appropriate state (e.g., Overloaded, Alert).

## Transitions

- **Resting → Awakening**  
    Module starts up and prepares for operation.

- **Awakening → Attentive**  
    Module is ready to receive input and start tasks.

- **Attentive → Active**  
    Module begins active work.

- **Active → Reflecting**  
    Module performs background processing and analysis.

- **Reflecting → Learning**  
    Module initiates a learning or training process.

- **Learning → Active**  
    Learning process is complete, module returns to active work.

- **Attentive → Alert**  
    Module enters alert mode, responding to triggers.

- **Attentive → Delegating**  
    Module delegates tasks to other modules or systems.

- **Delegating → Attentive**  
    Module receives a response from delegates and is ready again.

- **Active → Delegating**  
    Part of the active work is offloaded.

- **Any State → Overloaded**  
    Module detects overload or error.

- **Overloaded → Recovering**  
    Module starts the recovery process.

- **Recovering → Attentive**  
    Errors are resolved, module is ready again.

- **Any State → ShuttingDown**  
    Module shuts down in a controlled manner.

- **ShuttingDown → Resting**  
    Module is inactive and ready for the next start.

- **Any State → Interrupting**  
    Module is interrupted by a disruption or urgent event.

- **Interrupting → previous state**  
    After handling the interruption, the module returns to its previous state.

- **Interrupting → Overloaded**  
    In case of severe problems, the module transitions to the error/overload state.

- **Interrupting → Alert**  
    For urgent warnings or monitoring, the module transitions to alert state.

- **Interrupting → ShuttingDown**  
    If immediate shutdown is required, the module shuts down in a controlled manner.