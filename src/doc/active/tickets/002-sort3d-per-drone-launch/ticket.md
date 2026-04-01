## Ticket #002: sort3d per-drone launch in multiview tmux


### What
`multiview.tmuxp.yaml` launches sort3d globally; it should launch per-drone with correct `self_camera_index`

### Why
Ray selection requires each drone's sort3d to know which camera is its own. Current global launch breaks the ID-based ego-ray matching.

### Scope boundary
Only change tmux/launch config. Do not modify sort3d node code.

### Affected modules
`tmux/`, `mas_tracker/sort3d.launch.py`

### Acceptance criteria
Each drone's sort3d instance launches with correct `self_camera_index` and `num_cameras` params; `chosen_target_ray_w` publishes under each drone's namespace

### Flow
Direct fix

### Status
Not started
