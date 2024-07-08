import matplotlib.pyplot as plt
import Control.draw_control as draw_control

class C_car:
    # vehicle config
    RF = 1  # [m] distance from rear to vehicle front end of vehicle
    RB = 0.2  # [m] distance from rear to vehicle back end of vehicle
    W = 0.8  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 0.8  # [m] Wheel base
    TR = 0.1  # [m] Tyre radius
    TW = 0.12  # [m] Tyre width

if __name__ == '__main__':
    C_car = C_car()
    draw_control.draw_car(0, 0, 0, 0, C_car, "r")
    plt.show()