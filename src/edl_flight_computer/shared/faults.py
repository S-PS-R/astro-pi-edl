"""Range-check sensor readings and report fault / cleared transitions.

Limits are {channel: (low, high)} in sensor units; None means no limit on that
side. A reading must stay outside its range for HOLD_S to raise a fault, and
back inside for HOLD_S to clear it, so single noisy samples do not flap.
"""

HOLD_S = 0.5


class FaultMonitor:
    def __init__(self, limits, hold_s=HOLD_S):
        self.limits = dict(limits or {})
        self.hold_s = hold_s
        self.active = {}      # channel -> value when the fault was raised
        self._changing = {}   # channel -> time the reading crossed its limit

    def update(self, now, readings):
        """Check readings at real time `now`; return fault and fault_cleared transitions."""
        transitions = []
        for channel, (low, high) in self.limits.items():
            value = readings.get(channel)
            if value is None:
                continue
            outside = (low is not None and value < low) or (high is not None and value > high)
            # Outside while nominal, or inside while faulted, must persist for hold_s
            if outside == (channel in self.active):
                self._changing.pop(channel, None)
                continue
            since = self._changing.setdefault(channel, now)
            if now - since < self.hold_s:
                continue
            del self._changing[channel]
            if outside:
                self.active[channel] = value
                side = f'above limit {high:g}' if high is not None and value > high else f'below limit {low:g}'
                transitions.append(dict(event='fault', channel=channel, value=value, low=low, high=high,
                                        message=f'{channel} = {value:.2f} {side}'))
            else:
                del self.active[channel]
                transitions.append(dict(event='fault_cleared', channel=channel, value=value, low=low,
                                        high=high, message=f'{channel} = {value:.2f} back in range'))
        return transitions


def report(transitions, downlink, time_s):
    """Send fault transitions to the ground station (also printed locally)."""
    for item in transitions:
        data = {key: item[key] for key in ('channel', 'value', 'low', 'high')}
        downlink.event(item['event'], time_s, item['message'], **data)
