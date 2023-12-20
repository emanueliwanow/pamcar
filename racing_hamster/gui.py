import tkinter as tk
import numpy as np
import path_handling
from definitions import Target, RacingEnvironment, Car, InputTrajectory
from tkinter import simpledialog, ttk
from time import time
from typing import List, Tuple

env = RacingEnvironment('test')
Ts = 0.1
resolution = 100  # pixel per meter
grid_resolution_in_m = 0.5

root = tk.Tk()
root.title(env.get_name())

x_min, x_max = env.get_x_limits()
y_min, y_max = env.get_y_limits()
canvas_width_in_m = (x_max - x_min)
canvas_height_in_m = (y_max - y_min)

canvas_height_in_px = canvas_height_in_m * resolution
canvas_width_in_px = canvas_width_in_m * resolution

canvas = tk.Canvas(
    root,
    width=canvas_width_in_px,
    height=canvas_height_in_px)

cars: List[Car] = []
input_trajectories: List[InputTrajectory] = []
traveled_paths: List[List[Tuple[float, float]]] = []

colors = ['blue', 'red', 'green', 'yellow']

create_new_trajectory = False
draw_trajectory = False
current_time = 0
simulation_index = 0
simulation_state = 0

first_click = True
first_x, first_y = 0, 0

current_user_input = np.zeros(2)


def transform_coordinates_to_pixel(x, y):
    return int((x - x_min) * resolution), int((y - y_min) * resolution)


def transform_pixel_to_coordinates(x, y):
    return x / resolution + x_min, y / resolution + y_min


def create_coordinate_system():
    # Create border
    border_color = "black"
    border_width = 2

    # Create a border rectangle around the canvas
    canvas.create_rectangle(
        2 * border_width,
        2 * border_width,
        canvas_width_in_px,
        canvas_height_in_px,
        outline=border_color,
        width=border_width
    )
    # Create horizontal gridlines
    for y in np.arange(y_min, y_max, grid_resolution_in_m)[1:]:
        _, y_px = transform_coordinates_to_pixel(0, y)
        canvas.create_line(0, y_px, canvas_width_in_px, y_px, fill="lightgray")
        if y != 0:
            canvas.create_text(canvas_width_in_px / 2, y_px, text=y, anchor="e")

    # Create vertical gridlines
    for x in np.arange(x_min, x_max, grid_resolution_in_m)[1:]:
        x_px, _ = transform_coordinates_to_pixel(x, 0)
        canvas.create_line(x_px, 0, x_px, canvas_height_in_px, fill="lightgray")
        if x != 0:
            canvas.create_text(x_px, canvas_height_in_px / 2, text=x, anchor="n")

    # Create x-axis and y-axis

    canvas.create_line(0, canvas_height_in_px / 2, canvas_width_in_px, canvas_height_in_px / 2, fill="black", width=2)
    canvas.create_line(canvas_width_in_px / 2, 0, canvas_width_in_px / 2, canvas_height_in_px, fill="black", width=2)


def create_target(event):
    global first_click, first_x, first_y
    if first_click:
        canvas.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5, fill="red", tags='first_click')
        first_x, first_y = (event.x, event.y)
    else:
        canvas.delete('first_click')
        second_x, second_y = (event.x, event.y)
        target = Target(*transform_pixel_to_coordinates(first_x, first_y),
                        *transform_pixel_to_coordinates(second_x, second_y))
        env.add_target(target)
        draw_target(target)

    first_click = not first_click


def draw_target(target: Target):
    first_x, first_y = transform_coordinates_to_pixel(target.x1, target.y1)
    second_x, second_y = transform_coordinates_to_pixel(target.x2, target.y2)
    canvas.create_line(first_x, first_y, second_x, second_y, fill="red", tags="target")
    canvas.create_oval(first_x - 5, first_y - 5, first_x + 5, first_y + 5, fill="green", tags="target")
    canvas.create_oval(second_x - 5, second_y - 5, second_x + 5, second_y + 5, fill="green", tags="target")


def show_trajectory():
    global draw_trajectory
    draw_trajectory = not draw_trajectory
    visualize_trajectory(draw_all=True)
    if draw_trajectory:
        draw_trajectory_button.config(text="Hide Trajectory")
    else:
        draw_trajectory_button.config(text="Draw Trajectory")


def visualize_trajectory(draw_all=False):
    global draw_trajectory
    if draw_trajectory and len(traveled_paths) != 0:
        traveled_paths_np = np.array(traveled_paths)
        for color, trajectory in zip(colors, traveled_paths_np.transpose((1, 0, 2))):
            if draw_all:
                start = 1
            else:
                start = max(1, simulation_index - 1)
            for i in range(start, len(trajectory)):
                x1, y1 = transform_coordinates_to_pixel(*trajectory[i - 1])
                x2, y2 = transform_coordinates_to_pixel(*trajectory[i])
                canvas.create_line(x1, y1, x2, y2, fill=color, tags='trajectory')
    else:
        canvas.delete('trajectory')


def key_press(event):
    global current_user_input
    if event.keysym == "Up":
        current_user_input[0] = .50
    elif event.keysym == "Down":
        current_user_input[0] = -.75
    elif event.keysym == "Left":
        current_user_input[1] = -1.5
    elif event.keysym == "Right":
        current_user_input[1] = 1.5


def key_release(event):
    global current_user_input

    if event.keysym in ("Up", "Down"):
        current_user_input[0] = 0.0
    if event.keysym in ("Left", "Right"):
        current_user_input[1] = 0.0


def render_cars():
    canvas.delete("car")
    for color, car in zip(colors, cars):
        x, y = transform_coordinates_to_pixel(*car.get_car_position())
        canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill=color, tags="car")


def update_environment_view():
    root.title(env.get_name())
    canvas.delete("target")
    for target in env.get_targets():
        draw_target(target)


def move_cars(simulation_index):
    if create_new_trajectory:
        input_trajectories[0].append(current_user_input)
    new_car_positions = []
    for car, input_trajectory in zip(cars, input_trajectories):
        try:
            u = input_trajectory.get_input()[simulation_index]
            car.step(u, Ts)
        except IndexError:
            pass
        assert input_trajectory.get_sampling_time() == Ts

        new_car_positions.append(car.get_car_position())

    render_cars()
    traveled_paths.append(new_car_positions)
    visualize_trajectory()


def simulate_car_motion():
    start = time()
    global current_time
    global simulation_index
    if simulation_state == 1:
        move_cars(simulation_index)
        current_time += Ts
        simulation_index += 1

    canvas.delete('time')
    canvas.create_text(50, 20, text=f"Current Time: {current_time:.2f} s", anchor="w", tags='time')

    canvas.delete('speed')
    canvas.create_text(canvas_width_in_px - 200, 20, text=f"Current Speed:", anchor="w", tags='speed')
    for i in range(len(cars)):
        if simulation_index < len(input_trajectories[i].get_input()) or i == 0 and create_new_trajectory:
            speed = cars[i].get_speed()
        else:
            speed = 0
        canvas.create_text(canvas_width_in_px - 200, 20 * (i + 2),
                           text=f"{input_trajectories[i].get_name()}: {speed:.2f} m/s",
                           anchor="w", tags='speed', fill=colors[i])
    duration = time() - start
    root.after(int((Ts - duration) * 1000), simulate_car_motion)


def toggle_simulation():
    global simulation_state
    if simulation_state == 0:
        start_stop_button.config(text="Stop Simulation")
        new_simulation_state = 1
    elif simulation_state == 1:
        if create_new_trajectory:
            input_trajectories[0].save_trajectory_to_disk()
        start_stop_button.config(text="Reset Simulation")
        new_simulation_state = 2
    else:
        start_stop_button.config(text="Start Simulation")
        global simulation_index
        reset_simulation()
        new_simulation_state = 0

    simulation_state = new_simulation_state


def reset_simulation():
    global current_time
    global traveled_paths
    global simulation_index
    simulation_index = 0
    current_time = 0
    traveled_paths = []
    for car in cars:
        car.reset()
    render_cars()
    visualize_trajectory()


def enter_name():
    result = ask_for_user_input("Enter name of the new environment")
    if result:
        env.set_name(result)
        update_environment_view()


def ask_for_user_input(promt):
    return simpledialog.askstring("Input", promt)


def save_environment():
    env.save_state_to_disk()


create_coordinate_system()
render_cars()
canvas.pack()

draw_trajectory_button = tk.Button(root, text="Draw Trajectory", command=show_trajectory)
start_stop_button = tk.Button(root, text="Start Simulation", command=toggle_simulation)
input_button = tk.Button(root, text="Set Environment Name", command=enter_name)
save_button = tk.Button(root, text="Save Environment", command=save_environment)

draw_trajectory_button.pack(side="left", padx=10)
start_stop_button.pack(side="left", padx=10)
input_button.pack(side="left", padx=10)
save_button.pack(side="left", padx=10)


def select_input_trajectory():
    global create_new_trajectory
    global cars
    global input_trajectories

    reset_simulation()
    cars = []
    input_trajectories = []
    create_new_trajectory = False
    selected_items = [listbox.get(index) for index in listbox.curselection()]
    for item in selected_items:
        if item == "Create New Input Trajectory":
            input_trajectory_name = ask_for_user_input('Enter name of new trajectory')
            create_new_trajectory = True
            cars.insert(0, Car())
            input_trajectories.insert(0, InputTrajectory(input_trajectory_name, Ts=Ts))
        else:
            cars.append(Car())
            input_trajectory = InputTrajectory(item)
            input_trajectory.load_trajectory_from_disk()
            input_trajectories.append(input_trajectory)
    top.destroy()  # Close the pop-up window
    render_cars()


def open_option_window():
    global top, listbox  # Declare listbox as a global variable
    top = tk.Toplevel(root)
    top.title("Select Input Trajectories")

    listbox = tk.Listbox(top, selectmode=tk.MULTIPLE)
    listbox.pack()

    options = ['Create New Input Trajectory'] + path_handling.get_file_names_in_dir(path_handling.INPUT_TRAJECTORY_DIR,
                                                                                    remove_extension=True)
    for option in options:
        listbox.insert(tk.END, option)

    select_button = tk.Button(top, text="Select", command=select_input_trajectory)
    select_button.pack(pady=10)


def open_option_window_for_env():
    global top, listbox
    top = tk.Toplevel(root)
    top.title("Select Environment")

    listbox = tk.Listbox(top)
    listbox.pack()

    options = path_handling.get_file_names_in_dir(path_handling.ENVIRONMENT_DIR, remove_extension=True)
    for option in options:
        listbox.insert(tk.END, option)

    select_button = tk.Button(top, text="Select", command=select_env)
    select_button.pack(pady=10)


def select_env():
    selected_item = listbox.get(listbox.curselection())
    env.set_name(selected_item)
    env.load_state_from_disk()
    update_environment_view()
    top.destroy()


button = tk.Button(root, text="Select Input Trajectories", command=open_option_window)
button.pack(pady=2)
button = tk.Button(root, text="Select Environment", command=open_option_window_for_env)
button.pack(pady=2)

root.bind("<KeyPress>", key_press)
root.bind("<KeyRelease>", key_release)
root.bind("<Button-2>", create_target)

simulate_car_motion()

# Run the GUI main loop
root.mainloop()
