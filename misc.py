import wpilib


def is_sim() -> bool:
    return wpilib.RobotBase.isSimulation()


# Create a python decorator that squares a return value
def square(func):
    def wrapper(*args, **kwargs):
        return func(*args, **kwargs) * abs(func(*args, **kwargs))
    return wrapper


# Create python deorator that deadbands a value provided in the decorator
def deadband(deadband: float):
    def deadband_decorator(func):
        def wrapper(*args, **kwargs):
            value = func(*args, **kwargs)
            if abs(value) < deadband:
                return 0
            return value
        return wrapper
    return deadband_decorator

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