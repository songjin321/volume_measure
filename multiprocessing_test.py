from multiprocessing import pool

def f(x):
    return x*x
with pool.Pool(5) as p:
    print(p.map(f, [1, 2, 3]))