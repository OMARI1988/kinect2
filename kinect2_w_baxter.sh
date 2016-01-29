#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'kinect_viewer'
tmux new-window -t $SESSION:1 -n 'kinect_frame'
tmux new-window -t $SESSION:2 -n 'rviz'

tmux select-window -t $SESSION:0
tmux send-keys "rosrun kinect2_viewer kinect2_viewer kinect2 sd cloud" C-m

tmux select-window -t $SESSION:1
tmux send-keys "rosrun baxter_pykdl kinect_frame.py"

tmux select-window -t $SESSION:2
tmux send-keys "rviz"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
