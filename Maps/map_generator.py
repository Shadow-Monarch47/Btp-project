import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import json
import os

GRID_SIZE = 55
JSON_FILE = "Maps/map_obstacles.json"

# ---------- Load existing ----------
if os.path.exists(JSON_FILE):
    with open(JSON_FILE, "r") as f:
        data = json.load(f)
        obstacles = set(tuple(o) for o in data.get("obstacles", []))
else:
    obstacles = set()

patches_dict = {}
mouse_mode = None   # "add" or "remove"
last_cell = None


# ---------- Plot ----------
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, GRID_SIZE)
ax.set_ylim(0, GRID_SIZE)
ax.set_aspect('equal')

ax.set_xticks(range(GRID_SIZE + 1))
ax.set_yticks(range(GRID_SIZE + 1))
ax.grid(True)
ax.set_title("Left drag = add | Right drag = remove | S = save")


# ---------- Patch functions ----------
def draw_cell(x, y):
    if (x, y) in patches_dict:
        return
    rect = Rectangle((x, y), 1, 1)
    ax.add_patch(rect)
    patches_dict[(x, y)] = rect

def remove_cell(x, y):
    rect = patches_dict.pop((x, y), None)
    if rect:
        rect.remove()


# Draw existing obstacles
for x, y in obstacles:
    draw_cell(x, y)

fig.canvas.draw()


# ---------- Save ----------
def save():
    data = {
        "grid_size": GRID_SIZE,
        "obstacles": [list(o) for o in sorted(obstacles)]
    }
    with open(JSON_FILE, "w") as f:
        json.dump(data, f, indent=2)


# ---------- Mouse logic ----------
def get_cell(event):
    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return None
    x = int(event.xdata)
    y = int(event.ydata)
    if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
        return (x, y)
    return None


def apply_action(cell):
    global obstacles

    x, y = cell

    if mouse_mode == "add":
        if cell not in obstacles:
            obstacles.add(cell)
            draw_cell(x, y)

    elif mouse_mode == "remove":
        if cell in obstacles:
            obstacles.remove(cell)
            remove_cell(x, y)


# Mouse press
def on_press(event):
    global mouse_mode, last_cell

    if event.button == 1:
        mouse_mode = "add"
    elif event.button == 3:
        mouse_mode = "remove"
    else:
        return

    cell = get_cell(event)
    if cell:
        apply_action(cell)
        last_cell = cell
        fig.canvas.draw_idle()


# Mouse drag
def on_motion(event):
    global last_cell

    if mouse_mode is None:
        return

    cell = get_cell(event)
    if cell and cell != last_cell:
        apply_action(cell)
        last_cell = cell
        fig.canvas.draw_idle()


# Mouse release
def on_release(event):
    global mouse_mode, last_cell
    mouse_mode = None
    last_cell = None


# ---------- Keyboard / Close ----------
def on_key(event):
    if event.key.lower() == 's':
        save()
        print(f"Saved {len(obstacles)} obstacles")


def on_close(event):
    save()
    print(f"Auto-saved {len(obstacles)} obstacles")


# ---------- Connect ----------
fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("key_press_event", on_key)
fig.canvas.mpl_connect("close_event", on_close)

plt.show()
