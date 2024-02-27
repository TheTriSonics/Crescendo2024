from misc import square, deadband


@square
def f():
    return -4

@deadband(0.01)
def g():
    return 0.02

print(g(), f())
