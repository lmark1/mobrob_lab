from heapq import *
import numpy as np
import matplotlib.pyplot as plt

point = ()

def show_path(matrix, path=None, interesting_points=None):
    """
    matrix : 
    path : 
    interesting_points : 
    """
    if path is not None:
        for item in path:
            matrix[item[0]][item[1]]=100
        for i in range(150):
            for j in range(150):
                if matrix[i][j]==1:
                    matrix[i][j]=256 
        if interesting_points is not None:
            for item in interesting_points:
                matrix[item[0]][item[1]]=200

    fig = plt.figure()
    plt.imshow(matrix)
    cid = fig.canvas.mpl_connect('button_press_event', __onclick__)
    plt.show()


    return point


def __onclick__(click):
    global point 
    point = (click.xdata,click.ydata)

    plt.close()
    
    return point


def heuristic(a, b):
    """
    Return absolute distance.
    a, b : tuple (coordinates)
    """
    return abs(b[0] - a[0]) + abs(b[1] - a[1]) 


def astar(array, start, goal):
    """
    array : 
    start : tuple (coordinates)
    goal : tuple (coordinates)
    """
    # relativne koordinate susjeda
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}  # g part of cost function (sum of all distances till now)
    fscore = {start:heuristic(start, goal)}  # f cost function
    oheap = []  # ovo je nasa OPEN lista iz algoritma

    # heappush = Push the value (fscore[start], start) onto the heap, maintaining the heap invariant.
    heappush(oheap, (fscore[start], start)) 
    while oheap:  # sve dok postoji nesto na open listi
        

        # heappop = Pop and return the smallest item from the heap, maintaining the heap invariant.
        current = heappop(oheap)[1]  # WHY INDEX 1 ??
                                        
        if current == goal:  # ako je trenutni cvor jednak cilju
            data = []
            while current in came_from:  
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:  # za x, y koordinatu susjednih plocica
            neighbor = current[0] + i, current[1] + j  # racunanje realne (apsolutne) koordinate susjeda
            tentative_g_score = gscore[current] + heuristic(current, neighbor)  # g dio cost funkcije: pribrojavanje
                                                                                # dosadasnje izracunate g funkcije s trenutnom
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue              
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue              
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))               
    return False