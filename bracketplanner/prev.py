import tkinter

from tkinter import *
from tkinter import filedialog as fd

import PIL.Image
from PIL import ImageTk

from warp import *

win = Tk()
# bg_image = Image.open("./test_imgs/h1_1.jpg")
filename = fd.askopenfilename()
bg_image = PIL.Image.open(filename)
img_hold = bg_image

# full screen:
# win.geometry("{0}x{1}+0+0".format(win.winfo_screenwidth(), win.winfo_screenheight()))

# quarter screen:
win.geometry("1280x720")
win.title('vpoint_tool')

# canvas
canvas = tkinter.Canvas(win, bg="white", height=win.winfo_height(), width=win.winfo_width())
canvas.pack(fill=BOTH, expand=1)
bg_image = ImageTk.PhotoImage(bg_image)
bg_img = canvas.create_image(0, 0, anchor=NW, image=bg_image)

# drawing
coords = {"x": 0, "y": 0, "x2": 0, "y2": 0}
lines = []
slopes = []
heights = []
right_slope = 0
left_slope = 0
right_count = 0
left_count = 0
p_avg = []
area_of_interest = [(0, 0), (0, 0), (0, 0), (0, 0)]


def inter(m1, b1, m2, b2):
    m1 = -m1
    m2 = -m2
    px = (b2 - b1) / (m1 - m2)
    py = m1 * px + b1
    return px, py


def m_press(e):
    coords["x"] = e.x
    coords["y"] = e.y
    lines.append(canvas.create_line(coords["x"], coords["y"], coords["x"], coords["y"], width=2, fill='red'))


def m_pos(e):
    coords["x2"] = e.x
    coords["y2"] = e.y
    canvas.coords(lines[-1], coords["x"], coords["y"], coords["x2"], coords["y2"])


def m_rel(e):
    global left_count, left_slope, right_count, right_slope, p_avg_x, p_avg_y
    t_slope = (canvas.coords(lines[-1])[1] - canvas.coords(lines[-1])[3]) / (
            canvas.coords(lines[-1])[2] - canvas.coords(lines[-1])[0])
    t_height = canvas.coords(lines[-1])[1] + t_slope * canvas.coords(lines[-1])[0]
    slopes.append(t_slope)
    heights.append(t_height)

    if t_slope > 0:
        # left
        left_count += 1
        left_slope = (left_slope + t_slope) / left_count
    else:
        # right
        right_count += 1
        right_slope = (right_slope + t_slope) / right_count

    if len(lines) > 1:
        local_avg_x = local_avg_y = 0
        for i in range(len(lines) - 1):
            px, py = inter(slopes[-1], heights[-1], slopes[i], heights[i])
            canvas.create_oval(px - 3, py - 3, px + 3, py + 3, fill="green")
            local_avg_x += px
            local_avg_y += py
        local_avg_x /= (len(lines) - 1)
        local_avg_y /= (len(lines) - 1)
        canvas.create_oval(local_avg_x - 5, local_avg_y - 5, local_avg_x + 5, local_avg_y + 5, fill="yellow")
        p_avg.append([local_avg_x, local_avg_y])


def m_right(e):
    global area_of_interest
    p_sum = [0, 0]
    point3 = [0, 0]
    point4 = [0, 0]
    for i in range(len(lines) - 1):
        p_sum[0] += p_avg[i][0]
        p_sum[1] += p_avg[i][1]
    p_sum[0] /= (len(lines) - 1)
    p_sum[1] /= (len(lines) - 1)
    canvas.create_oval(p_sum[0] - 5, p_sum[1] - 5, p_sum[0] + 5, p_sum[1] + 5, fill="blue")

    # find max perspective
    img_width, img_height = img_hold.size
    # left line
    m_l = (img_height - p_sum[1]) / p_sum[0]
    h_l = img_height

    # right line
    m_r = (p_sum[1] - img_height) / (img_width - p_sum[0])
    h_r = p_sum[1] + m_r * p_sum[0]

    point3[0], point3[1] = inter(0, (p_sum[1] + 50), m_r, h_r)
    # point3[0] = img_width + point3[0]
    # point3[0] = -point3[0]
    point4[0], point4[1] = inter(0, (p_sum[1] + 50), m_l, h_l)

    area_of_interest = [(0, img_height), (img_width, img_height), (point3[0], point3[1]), (point4[0], point4[1])]

win.bind('<Button-1>', m_press)
win.bind('<Button-3>', m_right)
win.bind('<B1-Motion>', m_pos)
win.bind('<ButtonRelease-1>', m_rel)

win.mainloop()
warp_img(area_of_interest, filename)