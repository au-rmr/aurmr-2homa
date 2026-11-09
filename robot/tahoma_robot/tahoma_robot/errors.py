"""Real failures raise, so a forgotten check can't let the next move run."""


class RobotError(Exception):
    pass


class PlanningFailed(RobotError):
    pass


class PartialPath(PlanningFailed):
    """A straight path could not be completed, so nothing moved."""


class ExecutionFailed(RobotError):
    pass


class Stalled(ExecutionFailed):
    """The arm stopped making progress: tablet slider at 0, a protective stop, or far over time."""


class ForceLimitExceeded(RobotError):
    pass


class SensorStale(RobotError):
    pass


class GuardLost(RobotError):
    pass
