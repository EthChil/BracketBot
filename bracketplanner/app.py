import random
import math
import numpy as np
import scipy as sp
from scipy.integrate import solve_ivp
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

import matplotlib.pyplot as plt

def value_to_color(x, min_val, max_val):
    r = int((x - min_val) / (max_val - min_val) * 255)
    if r > 255:
        print(f"Incorrect color requested, {x} out of {min_val} - {max_val} range")
        return
    g = 0
    b = 255
    return '#' + '%02x%02x%02x' % (r, g, b)

# def t2s(x1, x2, xd1, xd2, y1, y2, yd1, yd2, t):
#     return sp.integrate.quad(lambda m: np.sqrt((xd1-2*t*(3*x1-3*x2+2*xd1+t*xd2)+3*t**2*(2*x1-2*x2+xd1+t*xd2)-t**2*xd2+t**3*xd2)**2+(yd1-2*t*(3*y1-3*y2+2*yd1+t*yd2)+3*t**2*(2*y1-2*y2+yd1+t*yd2)-t**2*yd2+t**3*yd2)**2), 0, t)[0]


def solve_st(x1, x2, xd1, xd2, y1, y2, yd1, yd2):
    def f(t, y):
        return ((xd1 - 2*t*(3*x1 - 3*x2 + 2*xd1 + t*xd2) + 3*t**2*(2*x1 - 2*x2 + xd1 + t*xd2) - t**2*xd2 + t**3*xd2)**2 + (yd1 - 2*t*(3*y1 - 3*y2 + 2*yd1 + t*yd2) + 3*t**2*(2*y1 - 2*y2 + yd1 + t*yd2) - t**2*yd2 + t**3*yd2)**2)**(1/2)

    sol = solve_ivp(f, (0, 1), (0,), method="RK23", dense_output=False)
    func = lambda t: np.interp(t, sol.t, sol.y[0])
    return func

def s2t(x1, x2, xd1, xd2, y1, y2, yd1, yd2, s):
    t0 = 0
    t1 = 1
    t2s = solve_st(x1, x2, xd1, xd2, y1, y2, yd1, yd2)
    while(t1-t0>0.001):
        if s > t2s(t0):
            if s > t2s((t1 + t0)/2):
                t0 = (t1 + t0)/2
            else:
                t1 = (t1 + t0)/2
    return (t0+t1)/2

# def s2t(x1, x2, xd1, xd2, y1, y2, yd1, yd2, s):
#     t0 = 0
#     t1 = 1
#     while(t1-t0>0.001):
#         if s > t2s(x1, x2, xd1, xd2, y1, y2, yd1, yd2, t0):
#             if s > t2s(x1, x2, xd1, xd2, y1, y2, yd1, yd2, (t1 + t0)/2):
#                 t0 = (t1 + t0)/2
#             else:
#                 t1 = (t1 + t0)/2
#     return (t0+t1)/2



class Zoom_Advanced(ttk.Frame):
    def __init__(self, mainframe, path):
        ttk.Frame.__init__(self, master=mainframe)
        self.master.title('Bracket Planner')
        self.master.geometry("1200x900")
        # Toolbar
        self.toolbar = tk.Frame(self.master, bg='light grey')
        self.open_button = tk.Button(self.toolbar, text="Open", font=("Helvetica", 10)).pack(side=tk.LEFT, padx = 2, pady = 2)
        self.start_button = tk.Button(self.toolbar, text="Start", font=("Helvetica", 10)).pack(side=tk.LEFT, padx = 2, pady = 2)
        self.pause_button = tk.Button(self.toolbar, text="Pause", font=("Helvetica", 10))
        self.pause_button.pack(side=tk.LEFT, padx = 2, pady = 2)
        self.skip_button = tk.Button(self.toolbar, text="Skip", font=("Helvetica", 10)).pack(side=tk.LEFT, padx = 2, pady = 2)
        self.toolbar.pack(side=tk.TOP, fill=tk.X)
        # Workspace
        self.workspace = tk.Frame(self.master, bg='white')
        # Create canvas and put image on it
        self.canvas = tk.Canvas(self.workspace, highlightthickness=0)
        # self.canvas.grid(row=0, column=0, sticky='nswe')
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.update()  # wait till canvas is created
        self.workspace.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        # Status Bar
        self.status=tk.Label(self.master,text="(x,y): ", bd=1, relief=tk.SUNKEN, anchor="w")
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
        # Make the canvas expandable
        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)
        # Bind events to the Canvas
        self.canvas.bind('<Configure>', self.show_image)  # canvas is resized
        self.canvas.bind('<Motion>', self.moved)
        self.canvas.bind('<ButtonPress-1>', self.m_press)
        self.canvas.bind('<B1-Motion>',     self.m_pos)
        self.canvas.bind('<Shift-B1-Motion>',     self.m_sq_pos)
        self.canvas.bind('<ButtonPress-2>', self.move_from)
        self.canvas.bind('<B2-Motion>',     self.move_to)
        self.canvas.bind('<MouseWheel>', self.wheel)  # with Windows and MacOS, but not Linux
        self.canvas.bind('<Button-5>',   self.wheel)  # only with Linux, wheel scroll down
        self.canvas.bind('<Button-4>',   self.wheel)  # only with Linux, wheel scroll up
        self.image = Image.open(path)  # open image
        self.width, self.height = self.image.size
        self.imscale = 1.0  # scale for the canvas image
        self.delta = 1.3  # zoom magnitude
        # Put image into container rectangle and use it to set proper coordinates to the image
        self.container = self.canvas.create_rectangle(0, 0, self.width, self.height, width=0)
        gridsz = 50
        # Lines
        self.coords = {"x": 0, "y": 0, "x2": 0, "y2": 0}
        self.lines = []
        self.splines = []
        self.angles = []
        self.hover_pos = self.canvas.create_text(self.canvas.winfo_width()-10, self.canvas.winfo_height()-10, text="", anchor="se", fill="white")

        self.draw_grid(gridsz)
        self.show_image()

    def moved(self, event):
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        self.status.configure(self.status, text="(x,y): (%r, %r)" % (x, y))

    def draw_grid(self, gridsz):
        for c in range(self.width // gridsz + 1):
            self.canvas.create_line(c*gridsz, 0, c*gridsz, self.height, fill='#ADADAD')
        for r in range(self.height // gridsz + 1):
            self.canvas.create_line(0, r*gridsz, self.width, r*gridsz, fill='#ADADAD')

    def create_spline(self, x1, y1, th1, x2, y2, th2, vel, accl=0):
        self.splines.append([])
        th1, th2 = -th1, -th2
        # Spline parameters
        k = 0.75*math.sqrt(pow(x2-x1,2)+pow(y2-y1,2)) # The 0.75 can be tuned to make it more relaxed or aggressive
        xd1 = k*math.cos(th1)
        xd2 = k*math.cos(th2)
        yd1 = k*math.sin(th1)
        yd2 = k*math.sin(th2)
        cx = xd1
        cy = yd1
        # Get length and segment length
        # spline_length = t2s(x1, x2, xd1, xd2, y1, y2, yd1, yd2, 1)
        spline_length = solve_st(x1, x2, xd1, xd2, y1, y2, yd1, yd2)(1)
        vel = spline_length/(spline_length//vel)
        min_seg = 9999
        max_seg = 0
        # Store previous point
        px, py = x1, y1
        # Acceleration
        ti = 0
        vi = 0
        # while vi < step:
        #     vi += accl
        #     ti += vi
        #     print("Position: ",ti," Velocity: ",vi)
        ss = []

        for s in np.arange(vel,spline_length+vel,vel):
            tx = s2t(x1, x2, xd1, xd2, y1, y2, yd1, yd2, s)
            ty = s2t(x1, x2, xd1, xd2, y1, y2, yd1, yd2, s)
            t = tx
            bx = -3*x1+3*x2-2*xd1-xd2*t
            by = -3*y1+3*y2-2*yd1-yd2*t
            ax = 2*x1-2*x2+xd1+xd2*t
            ay = 2*y1-2*y2+yd1+yd2*t
            dx = ax*pow(t,3)+bx*pow(t,2)+cx*t+x1
            dy = ay*pow(t,3)+by*pow(t,2)+cy*t+y1
            seg_len = math.sqrt(pow(dx-px,2)+pow(dy-py,2))
            ss.append(seg_len)
            if seg_len < min_seg: min_seg = seg_len
            elif seg_len > max_seg: max_seg = seg_len
            self.splines[-1].append(self.canvas.create_line(px, py, dx, dy, width=2, fill="white"))
            px = dx # Update previous point
            py = dy
        for l in self.splines[-1]:
            lc = self.canvas.coords(l)
            seg_len = math.sqrt(pow(lc[0]-lc[2],2)+pow(lc[1]-lc[3],2)) 
            self.canvas.itemconfig(l, fill=value_to_color(seg_len,min_seg,max_seg))

    def delete_spline(self,idx):
        for n in self.splines[idx]:
            self.canvas.delete(n)
        del self.splines[idx]

    def scroll_y(self, *args, **kwargs):
        self.canvas.yview(*args, **kwargs)  # scroll vertically
        self.show_image()  # redraw the image

    def scroll_x(self, *args, **kwargs):
        self.canvas.xview(*args, **kwargs)  # scroll horizontally
        self.show_image()  # redraw the image

    def move_from(self, event):
        self.canvas.scan_mark(event.x, event.y)

    def move_to(self, event):
        self.canvas.scan_dragto(event.x, event.y, gain=1)
        self.show_image()  # redraw the image

    def wheel(self, event):
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        bbox = self.canvas.bbox(self.container)  # get image area
        if bbox[0] < x < bbox[2] and bbox[1] < y < bbox[3]: pass  # Ok! Inside the image
        else: return  # zoom only inside image area
        scale = 1.0
        # Respond to Linux (event.num) or Windows (event.delta) wheel event
        if event.num == 5 or event.delta == -120:  # scroll down
            i = min(self.width, self.height)
            if int(i * self.imscale) < 30: return  # image is less than 30 pixels
            self.imscale /= self.delta
            scale        /= self.delta
        if event.num == 4 or event.delta == 120:  # scroll up
            i = min(self.canvas.winfo_width(), self.canvas.winfo_height())
            if i < self.imscale: return  # 1 pixel is bigger than the visible area
            self.imscale *= self.delta
            scale        *= self.delta
        self.canvas.scale('all', x, y, scale, scale)  # rescale all canvas objects
        self.show_image()

    def show_image(self, event=None):
        self.canvas.coords(self.hover_pos, self.canvas.winfo_width()-10, self.canvas.winfo_height()-10)

        bbox1 = self.canvas.bbox(self.container)  # get image area
        # Remove 1 pixel shift at the sides of the bbox1
        bbox1 = (bbox1[0] + 1, bbox1[1] + 1, bbox1[2] - 1, bbox1[3] - 1)
        bbox2 = (self.canvas.canvasx(0),  # get visible area of the canvas
                 self.canvas.canvasy(0),
                 self.canvas.canvasx(self.canvas.winfo_width()),
                 self.canvas.canvasy(self.canvas.winfo_height()))
        bbox = [min(bbox1[0], bbox2[0]), min(bbox1[1], bbox2[1]),  # get scroll region box
                max(bbox1[2], bbox2[2]), max(bbox1[3], bbox2[3])]
        if bbox[0] == bbox2[0] and bbox[2] == bbox2[2]:  # whole image in the visible area
            bbox[0] = bbox1[0]
            bbox[2] = bbox1[2]
        if bbox[1] == bbox2[1] and bbox[3] == bbox2[3]:  # whole image in the visible area
            bbox[1] = bbox1[1]
            bbox[3] = bbox1[3]
        self.canvas.configure(scrollregion=bbox)  # set scroll region
        x1 = max(bbox2[0] - bbox1[0], 0)  # get coordinates (x1,y1,x2,y2) of the image tile
        y1 = max(bbox2[1] - bbox1[1], 0)
        x2 = min(bbox2[2], bbox1[2]) - bbox1[0]
        y2 = min(bbox2[3], bbox1[3]) - bbox1[1]
        if int(x2 - x1) > 0 and int(y2 - y1) > 0:  # show image if it in the visible area
            x = min(int(x2 / self.imscale), self.width)   # sometimes it is larger on 1 pixel...
            y = min(int(y2 / self.imscale), self.height)  # ...and sometimes not
            image = self.image.crop((int(x1 / self.imscale), int(y1 / self.imscale), x, y))
            imagetk = ImageTk.PhotoImage(image.resize((int(x2 - x1), int(y2 - y1))))
            imageid = self.canvas.create_image(max(bbox2[0], bbox1[0]), max(bbox2[1], bbox1[1]),
                                               anchor='nw', image=imagetk)
            self.canvas.lower(imageid)  # set image into background
            self.canvas.imagetk = imagetk  # keep an extra reference to prevent garbage-collection

    def m_press(self,e):
        x = self.canvas.canvasx(e.x)
        y = self.canvas.canvasy(e.y)
        self.coords["x"] = x
        self.coords["y"] = y
        self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill='white', outline='black')
        self.lines.append(self.canvas.create_line(self.coords["x"], self.coords["y"], self.coords["x"], self.coords["y"], width=2, fill='white'))
        self.angles.append(0.0)
        if len(self.lines) > 1:
            v1 = self.canvas.coords(self.lines[-2])
            v2 = self.canvas.coords(self.lines[-1])
            self.create_spline(v1[0], v1[1], 0, v2[0], v2[1], 0, 10)

    def m_pos(self, e):
        r = 50
        x = self.canvas.canvasx(e.x)
        y = self.canvas.canvasy(e.y)
        self.angles[-1] = math.atan2((self.canvas.coords(self.lines[-1])[1] - y),(x - self.canvas.coords(self.lines[-1])[0]))
        x2 = r*math.cos(-self.angles[-1]) + self.coords["x"]
        y2 = r*math.sin(-self.angles[-1]) + self.coords["y"]
        self.coords["x2"] = x2
        self.coords["y2"] = y2
        #print(f"x: {x} y: {y} x2: {x2} y2: {y2} angle: {self.angles[-1]}")
        self.canvas.coords(self.lines[-1], self.coords["x"], self.coords["y"], self.coords["x2"], self.coords["y2"])

        if len(self.lines) > 1:
            self.delete_spline(-1)
            v1 = self.canvas.coords(self.lines[-2])
            v2 = self.canvas.coords(self.lines[-1])
            self.create_spline(v1[0], v1[1], self.angles[-2], v2[0], v2[1], self.angles[-1], 10)


    def m_sq_pos(self, e):
            r = 50
            x = self.canvas.canvasx(e.x)
            y = self.canvas.canvasy(e.y)
            dx = x - self.coords["x"]
            dy = y - self.coords["y"]
            if abs(dx) > abs(dy): # Snap horizontal
                if dx > 0:
                    self.coords["y2"] = self.coords["y"]
                    self.coords["x2"] = self.coords["x"] + 50
                    self.angles[-1] = 0
                else:
                    self.coords["y2"] = self.coords["y"]
                    self.coords["x2"] = self.coords["x"] - 50
                    self.angles[-1] = math.pi
            else: # Snap vertical
                if dy > 0:
                    self.coords["x2"] = self.coords["x"]
                    self.coords["y2"] = self.coords["y"] + 50
                    self.angles[-1] = -math.pi/2
                else:
                    self.coords["x2"] = self.coords["x"]
                    self.coords["y2"] = self.coords["y"] - 50
                    self.angles[-1] = math.pi/2
            x2 = self.coords["x2"] 
            y2 = self.coords["y2"] 
            #print(f"x: {x} y: {y} x2: {x2} y2: {y2} angle: {self.angles[-1]}")
            self.canvas.coords(self.lines[-1], self.coords["x"], self.coords["y"], self.coords["x2"], self.coords["y2"])

            if len(self.lines) > 1:
                self.delete_spline(-1)
                v1 = self.canvas.coords(self.lines[-2])
                v2 = self.canvas.coords(self.lines[-1])
                self.create_spline(v1[0], v1[1], self.angles[-2], v2[0], v2[1], self.angles[-1], 10)


path = 'icon map tiny.png'  # place path to your image here
root = tk.Tk()
app = Zoom_Advanced(root, path=path)
root.mainloop()