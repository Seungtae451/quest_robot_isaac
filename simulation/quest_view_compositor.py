"""Show only the body camera in Quest; raw dataset RGB is never modified."""
import cv2
import numpy as np

from config import teleop_config as cfg


def compose_quest_view(images,status='',recording_status=''):
    canvas=np.zeros((cfg.QUEST_VIEW_HEIGHT,cfg.QUEST_VIEW_WIDTH,3),np.uint8)
    rgb=images['front']
    if rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8:
        raise ValueError(f'front: expected HxWx3 uint8 RGB, got {rgb.shape}/{rgb.dtype}')
    gap=cfg.PANEL_GAP
    usable_height=cfg.QUEST_VIEW_HEIGHT-80
    scale=min((cfg.QUEST_VIEW_WIDTH-2*gap)/rgb.shape[1],(usable_height-2*gap)/rgb.shape[0])
    size=(max(1,round(rgb.shape[1]*scale)),max(1,round(rgb.shape[0]*scale)))
    resized=cv2.resize(rgb,size,interpolation=cv2.INTER_AREA if scale<1 else cv2.INTER_LINEAR)
    x=(cfg.QUEST_VIEW_WIDTH-size[0])//2;y=(usable_height-size[1])//2
    canvas[y:y+size[1],x:x+size[0]]=resized
    cv2.putText(canvas,status[:145],(20,cfg.QUEST_VIEW_HEIGHT-24),cv2.FONT_HERSHEY_SIMPLEX,.5,(200,200,200),1)
    if recording_status:
        color=(255,80,80) if recording_status.startswith('REC RECORDING') else (255,220,80)
        cv2.putText(canvas,recording_status[:140],(20,cfg.QUEST_VIEW_HEIGHT-50),cv2.FONT_HERSHEY_SIMPLEX,.6,color,1,cv2.LINE_AA)
    return canvas
