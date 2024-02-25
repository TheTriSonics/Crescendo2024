import wpilib


def is_sim() -> bool:
    return wpilib.RobotBase.isSimulation()


# Create a python decorator that captures all exceptions and returns False
# if one occurs
def safe_bool(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            return False
    return wrapper


def safe_float(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            return 0.00
    return wrapper