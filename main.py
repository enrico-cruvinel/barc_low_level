from controller import PID
from dynamics import acceleration
from visualization import TurtleFig

def main():
    controller = PID(kp, ki, kd, dt)
    model = acceleration()
    figure = TurtleFig
    graphics = 0

if __name__ == "__main__":
    main()

