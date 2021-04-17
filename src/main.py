# coding=utf-8
import numpy as np
import cv2 as cv
import math
from tkinter import Tk, Label, Button, Text, filedialog, messagebox
from PIL import Image, ImageTk
import glob


def draw_map(src="../src/case01.txt"):
    global shiftX, shiftY, datas, lines
    f = open(src, "r")
    file = f.readlines()
    f.close()
    datas,lines = [],[]
    x, y = [math.inf, -math.inf], [math.inf, -math.inf]
    try:
        for l in file:
            temp = l.split(",")
            d = [int(num) for num in temp]
            if d[0] < x[0]: x[0] = d[0]
            if d[0] > x[1]: x[1] = d[0]
            if d[1] < y[0]: y[0] = d[1]
            if d[1] > y[1]: y[1] = d[1]
            datas += [tuple(d)]
        shiftX = x[1] - int((x[1] - x[0]) / 2)
        shiftY = y[1] - int((y[1] - y[0]) / 2)
        cv.rectangle(road_map, shift(datas[1]), shift(datas[2]), green, -1)  # end area
        for i in range(3, len(datas) - 1):
            cv.putText(road_map, str(datas[i]), shift(datas[i]), cv.FONT_HERSHEY_SIMPLEX, 0.5, black, 1, cv.LINE_AA)
            cv.line(road_map, shift(datas[i]), shift(datas[i + 1]), black)
            lines += [get_line(datas[i], datas[i + 1])]
    except:
        messagebox.showinfo("Choose Map", "地圖內容格式不符")
        map_change()


def draw_car():
    global current_img
    angle = car_dir
    current_img = road_map.copy()
    cv.circle(current_img, shift((car_X, car_Y)), int(car_length / 2) * scale, red, 2)
    cv.putText(current_img, "({:.2f}, {:.2f})".format(car_X, car_Y), shift((car_X - 2, car_Y - 1)),
               cv.FONT_HERSHEY_SIMPLEX, 0.5, black, 1, cv.LINE_AA)
    d_c = draw_sensor(angle)
    d_l = draw_sensor(angle + 45)
    d_r = draw_sensor(angle - 45)

    t_center.delete("1.0", "end")  # 清空;"1.0"=start index
    t_center.insert("1.0", "Center Sensor : {:10.7f}".format(d_c))
    t_left.delete("1.0", "end")
    t_left.insert("1.0", "Left Sensor : {:10.7f}".format(d_l))
    t_right.delete("1.0", "end")
    t_right.insert("1.0", "Right Sensor : {:10.7f}".format(d_r))
    t_dir.delete("1.0", "end")
    t_dir.insert("1.0", "Current Direction : {:10.7f}".format(car_dir))
    return d_c, d_l, d_r


def draw_sensor(deg):
    global car_X, car_Y, current_img
    (x, y) = (car_X, car_Y)
    l = 20
    angle = deg % 360
    r = degree_to_radian(angle)

    if angle % 90 != 0:
        x = x + l * math.cos(r)
        y = y + l * math.sin(r)
    elif angle == 0:
        x = x + l
    elif angle == 180:
        x = x - l
    elif angle == 90:
        y = y + l
    elif angle == 270:
        y = y - l
    cv.line(current_img, shift((car_X, car_Y)), shift((x, y)), blue)
    line_func = get_line((car_X, car_Y), (x, y))
    d = get_distance(line_func)
    return d


# 使圖置於畫面正中
def shift(point):
    (x, y) = point
    new_x = (x - shiftX) * scale + imgW / 2
    new_y = -1 * (y - shiftY) * scale + imgH / 2
    return round(new_x), round(new_y)


def degree_to_radian(deg):
    r = deg * math.pi / 180
    return r


def radian_to_degree(r):
    deg = r * 180 / math.pi % 360
    return deg


def get_line(p, q):  # get line function : ax+by=c
    (px, py) = p
    (qx, qy) = q
    if px == qx:  # 垂直
        func = [1, 0, px, p, q]
    elif py == qy:  # 水平
        func = [0, 1, py, p, q]
    else:  # normal
        a = (qy - py)
        b = -1 * (qx - px)
        c = a * px + b * py
        func = [a, b, c, p, q]
    return func


def get_distance(sensor):
    global lines, car_X, car_Y
    d = math.inf
    (final_x, final_y) = (0, 0)
    for border in lines:
        if (border[0], border[1]) == (sensor[0], sensor[1]):  # parallel
            pass
        else:
            A = np.array([
                [border[0], border[1]],
                [sensor[0], sensor[1]]
            ])
            B = np.array([border[2], sensor[2]]).reshape(2, 1)
            invA = np.linalg.inv(A)
            ans = invA.dot(B)
            (x, y) = (ans[0, 0], ans[1, 0])

            (px, py), (qx, qy) = border[3], border[4]
            # 判斷焦點確實在邊界的範圍內；difference=1
            if min(px, qx) - 1 <= x <= max(px, qx) + 1 and min(py, qy) - 1 <= y <= max(py, qy) + 1:
                (px, py) = sensor[4]  # sensor end
                if (x - car_X) * (px - car_X) < 0 or (y - car_Y) * (py - car_Y) < 0:  # 反向
                    # cv.circle(current_img, shift((int(x), int(y))), 3, green, 1)
                    pass
                else:
                    temp = math.sqrt((x - car_X) ** 2 + (y - car_Y) ** 2)
                    if temp < d:
                        d = temp
                        (final_x, final_y) = (x, y)
    cv.circle(current_img, shift((final_x, final_y)), 3, red, -1)  # 劃出與邊的交點
    return float(format(d, '.7f'))


def car_move(thita):
    global car_X, car_Y, car_angle, car_dir
    car_angle = degree_to_radian(car_dir)
    t = degree_to_radian(thita)  # t=方向盤變動角度 ;car_angle=目前行進方向
    car_X = car_X + math.cos(car_angle + t) + math.sin(t) * math.sin(car_angle)
    car_Y = car_Y + math.sin(car_angle + t) - math.sin(t) * math.cos(car_angle)
    car_angle = car_angle - math.asin(2 * math.sin(t) / car_length)
    car_dir = radian_to_degree(car_angle)


# control car by itself
def car_control():
    global handle
    if bool_pause:
        handle = window.after(100, car_control)  # pass
    else:
        (c, l, r) = draw_car()
        showImage()
        # 抵達終點
        if end_P[0] <= car_X <= end_Q[0] and end_Q[1] <= car_Y <= end_P[1]:
            t_result.insert("1.0", "Finish Completly")
            showImage()
            f_4D.close()
            f_6D.close()
        # 碰撞
        elif c <= (car_length / 2) or l <= (car_length / 2) or r <= (car_length / 2):
            t_result.insert("1.0", "Collision")
            showImage()
            f_4D.close()
            f_6D.close()
        else:  # fuzzy rule
            dif = l - r
            max_dif = 20
            if abs(dif) < 4:  # 還沒到彎處
                s1 = "{:10.7f} {:10.7f} {:10.7f} {:11.7f}\n".format(c, l, r, 0)
                s2 = "{:10.7f} {:10.7f} ".format(car_X, car_Y) + s1
                car_move(0)
            else:
                s = -40 / max_dif
                thita = s * dif  # 方向盤要打的角度
                if thita < -40:
                    thita = -40
                elif thita > 40:
                    thita = 40
                t_thita.delete("1.0", "end")
                t_thita.insert("1.0", "Next Thita : {:10.7f}".format(thita))
                s1 = "{:10.7f} {:10.7f} {:10.7f} {:11.7f}\n".format(c, l, r, thita)
                s2 = "{:10.7f} {:10.7f} ".format(car_X, car_Y) + s1
                car_move(thita)  # 根據運動方程式移動
            f_4D.write(s1)
            f_6D.write(s2)
            handle = window.after(200, car_control)


def start():
    global car_X, car_Y, car_dir, f_4D, f_6D, handle
    print("start")
    # stop current activity
    if handle:
        window.after_cancel(handle)
        handle = None

    t_result.delete("1.0", "end")
    (car_X, car_Y, car_dir) = datas[0]
    f_4D = open("train4D.txt", 'w')
    f_6D = open("train6D.txt", 'w')
    car_control()


def pause():
    global bool_pause
    bool_pause = not bool_pause


def map_change():
    global end_P, end_Q, road_map, bool_pause, car_X, car_Y, car_dir, bool_pause
    bool_pause = False
    file_path = filedialog.askopenfilename()
    if not file_path:
        pass
    elif file_path.split('.')[-1] != "txt":
        messagebox.showinfo("Choose Map", "請使用txt檔案")
        map_change()
    else:
        print("map change : " + file_path)
        road_map = np.full((imgH, imgW, 3), 255, np.uint8)
        draw_map(file_path)
        (end_P, end_Q) = (datas[1], datas[2])
        (car_X, car_Y, car_dir) = datas[0]
        draw_car()
        showImage()


def choose_record():
    global record, step, car_dir, bool_pause
    bool_pause,car_dir = False,90
    t_result.delete("1.0", "end")
    file_path = filedialog.askopenfilename()
    if not file_path:  pass
    elif file_path.split('.')[-1] != "txt":
        messagebox.showinfo("Choose Record", "file_path.split('.')[-1] : 請使用txt檔案")
        choose_record()
    else:
        try:
            f = open(file_path, 'r')
            data = f.readlines()
            f.close()
            for l in data:
                temp = l.split(" ")
                d = []
                for num in temp:
                    if num == '': pass
                    else:
                        d.append(float(num))
                record += [(d[0], d[1], d[5])]  # x,y,dir
            step = 0
            map_change()
            car_control_by_record()
        except:
            messagebox.showinfo("Choose Record", "路徑檔格式不符: *6D.txt")
            choose_record()


def car_control_by_record():
    global car_X, car_Y, car_dir, step, handle
    if bool_pause:
        handle = window.after(100, car_control_by_record)
    else:
        if step >= len(record):
            t_result.insert("1.0", "Record End")
            showImage()
        else:
            (car_X, car_Y, thita) = record[step]
            r, t = degree_to_radian(car_dir), degree_to_radian(thita)
            r = r - math.asin(2 * math.sin(t) / car_length)
            car_dir = radian_to_degree(r)

            t_thita.delete("1.0", "end")
            t_thita.insert("1.0", "Next Thita : {:10.7f}".format(thita))
            draw_car()
            showImage()
            step += 1
            handle = window.after(200, car_control_by_record)

# 將opencv影像顯示在tkinter視窗
def showImage():
    global img_Panel
    img = Image.fromarray(current_img)
    imgtk = ImageTk.PhotoImage(image=img)
    img_Panel.imgtk = imgtk
    img_Panel.config(image=imgtk)


# image initial parameter
(imgH, imgW) = (600, 800)
(scale, shiftX, shiftY) = (8, 300, 300)
road_map = np.full((imgH, imgW, 3), 255, np.uint8)
current_img = road_map.copy()
red, green, blue, black = (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 0, 0)
bool_pause, bool_road = False, False

# main data
car_length = 6
datas, lines, record, step = [], [], [], 0
draw_map()
(car_X, car_Y, car_dir) = datas[0]
car_angle = degree_to_radian(car_dir)
(end_P, end_Q) = (datas[1], datas[2])

# window
handle = None  # for window control
window = Tk()
window.title("HW_1")
window.geometry('1000x600')
img_Panel = Label(window)
img_Panel.place(relx=0, rely=0, relheight=1, relwidth=0.6)
t_center = Text(window, height=2)
t_center.place(relx=0.65, rely=0, relheight=0.1, relwidth=0.3)
t_left = Text(window, height=2)
t_left.place(relx=0.65, rely=0.1, relheight=0.1, relwidth=0.3)
t_right = Text(window, height=2)
t_right.place(relx=0.65, rely=0.2, relheight=0.1, relwidth=0.3)
t_dir = Text(window, height=2)
t_dir.place(relx=0.65, rely=0.3, relheight=0.1, relwidth=0.3)
t_thita = Text(window, height=2)
t_thita.insert("1.0", "Change Thita : ")
t_thita.place(relx=0.65, rely=0.4, relheight=0.1, relwidth=0.3)
btn_start = Button(window, text="Start", command=start)
btn_start.place(relx=0.65, rely=0.5, relheight=0.1, relwidth=0.3)
btn_pause = Button(window, text="Pause", command=pause)
btn_pause.place(relx=0.65, rely=0.6, relheight=0.1, relwidth=0.3)
btn_map = Button(window, text="Choose Map", command=map_change)
btn_map.place(relx=0.65, rely=0.7, relheight=0.1, relwidth=0.3)
btn_record = Button(window, text="Choose Record", command=choose_record)
btn_record.place(relx=0.65, rely=0.8, relheight=0.1, relwidth=0.3)
t_result = Text(window, height=5, fg='red', font=14)
t_result.place(relx=0.65, rely=0.9, relheight=0.1, relwidth=0.3)

draw_car()
showImage()
window.mainloop()
