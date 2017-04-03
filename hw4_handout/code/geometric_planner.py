import numpy as np
import numpy.linalg as la
import time

def cost(p1, p2):
    return la.norm(np.array(p1)-np.array(p2))

def replace(p1, p2, eps):
    delta = cost(p1, p2)
    if delta == 0:
        return p2
        
    direc = (np.array(p2)-np.array(p1))/delta

    return p1 + direc * min(delta, eps)

def interpolate(p1, p2, a):
    return np.array(p1)*(1-a) + np.array(p2)*(a)

def generate(llim, ulim, pg, pr, check):
    coin = np.random.rand()
    if coin < pr:
        pf = pg
    else:
        collision = True
        while collision:
            llim = np.array(llim)
            ulim = np.array(ulim)
            pf = np.random.rand(1, llim.shape[0])
            pf = pf*(ulim - llim) + llim
            collision = check(pf)
    
    return np.array(pf).squeeze()


def extend(p1, p2, check, eps):
    collision = False
    p1 = np.array(p1)
    p2 = np.array(p2)
    pf = p1
    c  = cost(p1, p2)
    num = max(np.ceil(c / eps), 1)
    #print num
    v, s = np.linspace(0, 1, num=num, endpoint=False, retstep=True)
    v += s
    for alpha in v:
        p = interpolate(p1, p2, alpha)
        collision = check(p)
        if not collision:
            pf = p
        else:
            break

    return np.array(pf)

def shorten(path, extend, timeout=5.0, plot_edge=None):
    to  = time.time()
    tf  = to + timeout
    T   = 0.09
    dt  = 0.01
    a   = 0.9
    eps = 1e-3
    nul = 1e-12
    while (to + T + dt) < tf:
        scost = [cost(path[i], path[i+1]) 
                    for i in range(len(path)-1)]
        dcost = [scost[i]+scost[i+1] 
                    for i in range(len(scost)-1)]
        ncost = [cost(path[i], path[i+2])
                    for i in range(len(scost)-1)]

        delta = [dcost[i] - ncost[i] for i in range(len(dcost))]
        idx = [i[0] for i in sorted(enumerate(delta), 
                                    key=lambda x:x[1], 
                                    reverse=True)]
        #print path
        #print delta
        #print idx
        
        found = False
        for i in idx:
            if delta[i] > eps:
                start_config = path[i]
                end_config = path[i+2]
                new_config = extend(start_config, end_config)
                error = cost(new_config, end_config)
                #print 'test', i, '| error', int(error), '| pass', error < eps
                if error < eps:
                    found = True
                    break
                #else:
                #    print new_config

        if not found:
            #print 'not found'
            #IPython.embed()
            j = np.argmax(scost)
            path = list(path[:j+1] + \
                   [interpolate(path[j], path[j+1], 0.5)] + \
                   path[j+1:])

        else:
            path = list(path[:i+1] + path[i+2:])
        
        tx = time.time()
        T = T*a + (tx-to)*(1-a)
        to = tx

        if plot_edge:
            for i in range(len(path)-1):
                plot_edge(path[i], path[i+1], 'b')
        #print '***', len(path), T, tf - to

    scost = [cost(path[i], path[i+1]) 
                for i in range(len(path)-1)]
    dcost = [scost[i]+scost[i+1] 
                for i in range(len(scost)-1)]
    ncost = [cost(path[i], path[i+2])
                for i in range(len(scost)-1)]

    delta = [dcost[i] - ncost[i] for i in range(len(dcost))]

    print len(path)
    npath = [path[0]]
    npath.extend([path[i] for i in range(1, len(path)-1)
                            if delta[i-1] > nul])
    npath.append(path[-1])
    print len(npath)
    #IPython.embed()

    path = npath

    return path