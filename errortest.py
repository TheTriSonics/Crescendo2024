from misc import safe, compsafe

@compsafe
def f():
    print('start')
    raise ValueError()
    print('end')

f()
