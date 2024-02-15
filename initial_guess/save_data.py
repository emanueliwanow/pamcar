import pickle
import os

def create_file(file_name):
    with open(file_name, 'w') as file:
        print("File", file_name, "created sucessfully.")


def save_data(doors, order, x, y, v, theta, time, file_name):
    data = {'doors': doors, 'order': order, 'x_values': x, 'y_values': y, 'v_values': v, 'theta_values': theta, "time": time}
    create_file(file_name)
    with open(file_name, 'wb') as file:
        pickle.dump(data, file)
    print("Data save successfully in ", file_name)


def load_data(file_name):
    if os.path.exists(file_name):
        with open(file_name, 'rb') as file:
            data = pickle.load(file)
            doors = data['doors']
            order = data['order']
            x = data['x_values']
            y = data['y_values']
            v = data['v_values']
            theta = data['theta_values']
            time = data['time']
        print("Data loaded successfully from", file_name)
        return doors, order, x, y, v, theta, time
    else:
        print("File not found")
        return None