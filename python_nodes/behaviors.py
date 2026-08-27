from statemachine import StateChart


class StateBehavior:
    """Abstract base class for all state behaviors."""
    def on_enter(self, node): pass
    def tick(self, node): pass
    def on_leave(self, node): pass


class IdleBehavior(StateBehavior):
    """Default pass-through behavior when no explicit handler is attached."""
    pass


class BehaviorGraph(StateChart):
    def __init__(self, node, default_behavior = None):
        self.node = node
        self.behaviors = {}
        self.default_behavior = default_behavior if default_behavior is not None else IdleBehavior()
        self.active_behavior = self.default_behavior
        self.fsm_ok = False
        super().__init__()

    def attach_behavior(self, state_id: str, behavior: StateBehavior):
        """Registers a behavior strategy for a specific state ID."""
        self.behaviors[state_id] = behavior
        return self

    def validate(self):
        """Ensures the graph is non-empty and bootstraps the initial state."""
        if not self.states:
            print("FSM Validation Failed: BehaviorGraph contains no declared states.")
            return self.fsm_ok


        initial_id = next(iter(self.configuration_values), None)
        if initial_id is None:
            print("FSM Validation Failed: BehaviorGraph contains no initial state.")
            return self.fsm_ok

        self.active_behavior = self._get_behavior_for_state(initial_id)

        self.fsm_ok = True
        return self.fsm_ok

    def _get_behavior_for_state(self, state_id):
        """Retrieves registered behavior or falls back to default_behavior with a warning."""
        if state_id in self.behaviors:
            return self.behaviors[state_id]

        return self.default_behavior

    def start_fsm(self):
        self.active_behavior.on_enter(self.node)

    def on_transition(self, event, state, target):
        """Lifecycle hook: cleanly exit active behavior and transition to target."""
        self.active_behavior.on_leave(self.node)
        self.active_behavior = self._get_behavior_for_state(target.id)
        self.active_behavior.on_enter(self.node)

    def tick(self):
        """Executes active behavior on loop cycle."""
        if self.fsm_ok:
            self.active_behavior.tick(self.node)
