"""Explain existing AR start gates without changing motion or dataset values."""


def start_decision(attachment, *, fresh, receiver_ready, restarted, mapper_ready,
                   episode, episode_available, command_busy):
    reasons = list(attachment['reasons'])
    if not fresh:
        reasons.append('TRACKING_OR_SCENE_STALE')
    if not receiver_ready:
        reasons.append('ISAAC_FEEDBACK_STALE')
    if restarted:
        reasons.append('ISAAC_RESTARTED')
    if episode is not None:
        if not episode_available:
            reasons.append('RECORDER_UNAVAILABLE')
        elif episode['state'] != 'READY':
            reasons.append('EPISODE_' + episode['state'])
        if not mapper_ready:
            reasons.append('NEUTRAL_NOT_READY')
        if not episode.get('start_allowed', False):
            reasons.extend((episode.get('start_readiness') or {}).get('reasons') or ['HOME_NOT_READY'])
        if command_busy:
            reasons.append('EPISODE_COMMAND_PENDING')
    reasons = list(dict.fromkeys(reasons))
    return {'allowed': not reasons, 'reasons': reasons,
            'distances_m': attachment['distances_m'], 'radius_m': attachment['radius_m'],
            'home': episode.get('start_readiness') if episode else None}


def start_message(decision):
    distances = decision['distances_m']
    distance_text = ('L/R unknown' if distances is None else
                     f'L={distances[0]*100:.2f}cm R={distances[1]*100:.2f}cm')
    reason_text = ', '.join(decision['reasons']) or 'READY'
    return f'{distance_text} (both <= {decision["radius_m"]*100:.2f}cm); {reason_text}'
