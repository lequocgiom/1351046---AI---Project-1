__author__ = 'misbah'
from tkinter import *
import time

window = Tk()
window.title("A* Algorithm")

Walls = []
rows = 30
cols = 30
player = (0,0)
end = (cols-2,rows-2)

Width = 500/rows


platform = Canvas(window, width = cols*Width, height = rows*Width)
platform.pack()

def render_grid():
    global rows, cols, Width, player, end
    for i in range(cols):
        for j in range(rows):
            platform.create_rectangle(i*Width, j*Width,(i+1)*Width, (j+1)*Width, fill="white", width = 1)

    platform.create_rectangle(player[0]*Width, player[1]*Width,(player[0]+1)*Width, (player[1]+1)*Width, fill="blue", width = 1)
    platform.create_rectangle(end[0]*Width, end[1]*Width,(end[0]+1)*Width, (end[1]+1)*Width, fill="green", width = 1)

    for (i,j) in Walls:
        platform.create_rectangle(i*Width, j*Width,(i+1)*Width, (j+1)*Width, fill="black", width = 1)

render_grid()

def render_path(path):
    global Width
    for (i,j) in path:
        platform.create_rectangle(i*Width, j*Width,(i+1)*Width, (j+1)*Width, fill="blue", width = 1)
        time.sleep(0.1)

def start_game():
    window.mainloop()
