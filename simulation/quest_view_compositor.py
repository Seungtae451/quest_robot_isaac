"""Compose copies of raw RGB into an operator-only, 1280x720 dashboard.

Raw VLA buffers are never modified. Body occupies 64% of usable width; each
wrist occupies 18%. Aspect-preserving letterboxing prevents distorted views.
This module has no Isaac imports and is independently testable.
"""
import cv2
import numpy as np

from config import teleop_config as cfg


def compose_quest_view(images, status="", recording_status=""):
    canvas = np.zeros((cfg.QUEST_VIEW_HEIGHT, cfg.QUEST_VIEW_WIDTH, 3), np.uint8)
    gap = cfg.PANEL_GAP
    usable = cfg.QUEST_VIEW_WIDTH - 4 * gap
    main_width = int(usable * cfg.MAIN_VIEW_FRACTION)
    side_width = (usable - main_width) // 2
    x = gap
    for name, label, width in (("left_wrist", "LEFT WRIST", side_width),
                               ("front", "BODY", main_width),
                               ("right_wrist", "RIGHT WRIST", side_width)):
        rgb = images[name]
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError(f"{name}: expected HxWx3 uint8 RGB, got {rgb.shape}/{rgb.dtype}")
        scale = min(width / rgb.shape[1], (cfg.QUEST_VIEW_HEIGHT - 120) / rgb.shape[0])
        size = (max(1, round(rgb.shape[1] * scale)), max(1, round(rgb.shape[0] * scale)))
        resized = cv2.resize(rgb, size, interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
        y = (cfg.QUEST_VIEW_HEIGHT - size[1]) // 2
        left = x + (width - size[0]) // 2
        canvas[y:y + size[1], left:left + size[0]] = resized
        cv2.putText(canvas, label, (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, .55, (220, 220, 220), 1, cv2.LINE_AA)
        x += width + gap
    cv2.putText(canvas, status[:145], (20, cfg.QUEST_VIEW_HEIGHT - 24), cv2.FONT_HERSHEY_SIMPLEX, .5, (200, 200, 200), 1)
    if recording_status:
        color = (255, 80, 80) if recording_status.startswith("REC RECORDING") else (255, 220, 80)
        cv2.putText(canvas, recording_status[:140], (20, cfg.QUEST_VIEW_HEIGHT - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, color, 1, cv2.LINE_AA)
    return canvas
