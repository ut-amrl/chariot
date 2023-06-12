"""author: Corrie Van Sice, 2023."""

from chariot.control.odrive_manager import OdriveMotorManager
from live_plotter import LivePlotter
import time
import structlog
import matplotlib.pyplot as plt
import matplotlib.animation as animation

"""This is an example script that prints all errors to the log."""
LOG = structlog.get_logger()

# Motor IDs
BACK_LEFT = 0
BACK_RIGHT = 1
FRONT_LEFT = 3
FRONT_RIGHT = 2

# Motor info variables for logger
motor_names = ['BL', 'BR', 'FR', 'FL']
axis_ids = list(motor_name.length())
pos_estimates = []  # Position estimates from encoder
vel_estimates = []  # Velocity estimates from encoder
voltages = []  # The controller's VBus voltage
axis_states = []  # The state the axis is in
errors = []  # Odrive errors
last_errors = [] # for tracking last found list of errors
controller_error = False
encoder_error = False
motor_error = False
odrive_error = False

# Variables for animated plots
times = []
vels = []
volts = []
poss = []
# fig = plt.figure()
#a_velFig = velFig.add_subplot(1, 1, 1)
fig, axs = plt.subplots(3, sharex=True)

def clear_lists():
    """Reset all logging variables."""
    pos_estimates.clear()
    vel_estimates.clear()
    voltages.clear()
    axis_states.clear()
    errors.clear()
    global controller_error
    global encoder_error
    global motor_error
    global odrive_error
    controller_error = False
    encoder_error = False
    motor_error = False
    odrive_error = False


def openFile():
    """Create a log file to output data."""
    try:
        filehandle = open('../logs/' + time.strftime('%Y-%m-%d_%H:%M:%S') + '.log', 'w')
        return filehandle
    except FileNotFoundError:
        print("The 'logs' directory does not exist")


def updatePlotVals(xs, ys, x, y):
    """Update the plot animation with new data"""
    xs.append(x)
    ys.append(y)
    xs = xs[-200:]
    ys = ys[-200:]

def updatePlotYVals(ys, y):
    """Update the plot animation with new data - only the y-axis vals"""
    ys.append(y)
    ys = ys[-200:]


def drawPlot(subplt, xs, ys, motor_n='', y_label=''):
    """Draw the plot animation"""
    subplt.clear()
    subplt.plot(xs, ys, label=motor_n)

    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.xlabel('Time (s)')
    plt.ylabel(y_label)


def update(i):
    "Get values from CAN interface and write them to file"

    # Get the motor info
    t = time.time()
    heartbeats = mgr.get_heartbeat()
    estimates = mgr.get_encoder_estimate()
    voltages = mgr.get_vbus_voltage()

    # Get details from motor info and log to file
    for id in axis_ids:
        pos_estimates.append(estimates[id].pos_estiamte)
        vel_estimates.append(estimates[id].vel_estimate)
        axis_states.append(heartbeats[id].axis_state)

        if (heartbeats[id].controller_flags != '0x0'):
            controller_error = True
            errors.append('Motor ' + str(id) + motor_names[id] + ', Controller error')
            errors.append(heartbeats[id].controller_flags)
        if (heartbeats[id].encoder_flags != '0x0'):
            encoder_error = True
            errors.append('Motor ' + str(id) + motor_names[id] + ', Encoder error')
            errors.append(heartbeats[id].encoder_flags)
        if (heartbeats[id].motor_flags != '0x0'):
            motor_error = True
            errors.append('Motor ' + str(id) + motor_names[id] + ', Motor error')
            errors.append(heartbeats[id].motor_flags)
        if (heartbeats[id].axis_error != '0x0'):
            odrive_error = True
            errors.append('Motor ' + str(id) + motor_names[id] + ', Axis error')
            errors.append(heartbeats[id].axis_error)

    # file write functions
    f.write(str(t) + ', positions: ' + str(pos_estimates) + '\n')
    f.write(str(t) + ', velocities: ' + str(vel_estimates) + '\n')
    f.write(str(t) + ', voltages: ' + str(voltages) + '\n')

    # check for errors and log new ones
    if (controller_error | encoder_error | motor_error | odrive_error):
        if (errors != last_errors):
            f.write(str(t) + ', errors: ' + str(errors) + '\n')
            global last_errors
            last_errors = errors

    # Update plots
    for id in axis_ids:
        """ Update each subplot with data from each motor """
        updatePlotVals(times, vels, t, round(vel_estimates[id], 2))
        drawPlot(axs[0], times, vels, motor_names[id], 'Motor Velocity (m/s)')

        updatePlotYVals(volts, round(voltages[id], 2))
        drawPlot(axs[1], times, volts, motor_names[id], 'Volts')

        updatePlotYVals(poss, round(pos_estimates[id], 2))
        drawPlot(axs[2], times, poss, motor_names[id], 'Position (rad)')

    # reset all lists for next update
    clear_lists()


if __name__ == '__main__':
    mgr = OdriveMotorManager()
    f = openFile()
    try:
        while True:
            update()
        """Run the plot animate function - includes updating motor logs"""
        anim = _animation.FuncAnimation(fig, update, interval=100)
        """Show the plot drawing"""
        plt.legend()
        plt.show()

    except KeyboardInterrupt:
        print('Shutdown signal received. Ending execution.')
