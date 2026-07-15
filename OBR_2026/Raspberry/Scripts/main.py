import tkinter
from datetime import timedelta
from multiprocessing import Process, shared_memory

import customtkinter as ctk
import cv2
import psutil
from PIL import Image

from control import control_loop
from line_cam import line_cam_loop
from mp_manager import *

ctk.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme("dark-blue")  # Themes: "blue" (standard), "green", "dark-blue"

camera_width_1 = 448
camera_height_1 = 252

display_status = "none"

data_font_size = 15
label_color = "#141414"
button_color = "#141414"


class App(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Overengineering²")

        # =================== Center Form =================== #
        # Window size
        self.geometry("1024x600")

        # Turn off the resizing feature
        self.resizable(False, False)

        # Make window be always on top
        self.attributes("-topmost", True, "-fullscreen", True)

        # =================== Mainframe =================== #
        self.mainFrame = ctk.CTkFrame(master=self)
        self.mainFrame.pack(pady=8, padx=8, fill="both", expand=True)

        self.mainFrame.grid_columnconfigure(0, weight=2)
        self.mainFrame.grid_columnconfigure(1, weight=1)
        self.mainFrame.grid_columnconfigure(2, weight=1)
        self.mainFrame.grid_columnconfigure(3, weight=1)
        self.mainFrame.grid_columnconfigure(4, weight=2)

        self.mainFrame.grid_rowconfigure(0)
        self.mainFrame.grid_rowconfigure(1)
        self.mainFrame.grid_rowconfigure(2)

        # =================== Camera Stream =================== #
        self.top_cam = ctk.CTkFrame(master=self.mainFrame)
        self.top_cam.grid(column=0, row=0, columnspan=5, sticky="n", padx=8, pady=8)

        self.top_camera = ctk.CTkLabel(self.top_cam, text="")
        self.top_camera.grid(padx=6, pady=6)

        # =================== Status Indicator =================== #
        self.label_status_var = tkinter.StringVar(value=" ")
        self.label_status_font = ctk.CTkFont(family="arial", size=15)
        label_status = ctk.CTkLabel(master=self.mainFrame, corner_radius=4, textvariable=self.label_status_var, text_color="white", font=self.label_status_font, fg_color="#141414")
        label_status.grid(column=3, columnspan=2, sticky="s", row=0, padx=8, pady=17)

        # =================== Calibration Frame =================== #
        self.calibrationFrame = ctk.CTkFrame(master=self.mainFrame)
        self.calibrationFrame.configure(width=self.calibrationFrame["width"])
        self.calibrationFrame.grid_propagate(0)
        self.calibrationFrame.grid(column=4, row=1, sticky="ne", padx=8, pady=4)

        self.calibrationFrame.grid_columnconfigure(0, weight=2)
        self.calibrationFrame.grid_columnconfigure(1, weight=1)
        self.calibrationFrame.grid_columnconfigure(2, weight=1)

        self.calibrationFrame.grid_rowconfigure(0, weight=1)

        # =================== Exit Color Calibration Button =================== #
        self.exit_color_calibration_button = ctk.CTkButton(master=self.calibrationFrame, corner_radius=5, command=self.exit_calibrate_color, text="✕", text_color="white", font=("Arial", 20), width=40, height=30, fg_color=button_color, hover_color="black")
        self.exit_color_calibration_button.grid(column=2, row=0, sticky="ne", padx=53, pady=8)

        # =================== Color Calibration Button =================== #
        self.color_calibration_button_var = tkinter.StringVar(value="✎")
        self.color_calibration_button = ctk.CTkButton(master=self.calibrationFrame, corner_radius=5, command=self.set_calibrate_color_status, textvariable=self.color_calibration_button_var, text_color="white", font=("Arial", 20), width=40, height=30, fg_color="black", hover_color=button_color)
        self.color_calibration_button.grid(column=2, row=0, sticky="ne", padx=8, pady=8)

        # =================== Color Choose Button =================== #
        calibration_color.value = "l-gz"
        self.color_choose_button_var = tkinter.StringVar(value="l-gz")
        self.color_choose_button = ctk.CTkButton(master=self.calibrationFrame, corner_radius=5, command=self.choose_color, textvariable=self.color_choose_button_var, text_color="white", font=("Arial", data_font_size), width=40, height=30, fg_color=button_color, hover_color="black")
        self.color_choose_button.grid(column=2, row=0, sticky="ne", padx=8, pady=44)

        # =================== Bar =================== #
        self.barFrame = ctk.CTkFrame(master=self.mainFrame)
        self.barFrame.grid_propagate(0)
        self.barFrame.grid(column=0, row=2, columnspan=5, sticky="nwe", padx=8, pady=8)

        self.barFrame.grid_columnconfigure(0, weight=2)
        self.barFrame.grid_columnconfigure(1, weight=1)
        self.barFrame.grid_columnconfigure(2, weight=1)
        self.barFrame.grid_columnconfigure(3, weight=1)
        self.barFrame.grid_columnconfigure(4, weight=2)

        self.barFrame.grid_rowconfigure(0)
        self.barFrame.grid_rowconfigure(1)
        self.barFrame.grid_rowconfigure(2)

        logo = Image.open("../../Python/main/resources/logo/logo_white_transparent.png")
        self.logo_img = ctk.CTkImage(logo, size=(50, 50))

        self.logo_label = ctk.CTkLabel(self.barFrame, text="", image=self.logo_img)
        self.logo_label.grid(column=2, row=0, columnspan=3, rowspan=3, sticky="ne", padx=8, pady=18)

        # =================== Timer =================== #
        self.label_timer_var = tkinter.StringVar(value="--:--:--")
        self.label_timer = ctk.CTkLabel(master=self.barFrame, textvariable=self.label_timer_var, text_color="white", font=("Arial", 30), width=80, height=30)
        self.label_timer.grid(column=2, row=0, sticky="n", padx=0, pady=10)

        # =================== Top Right Bar =================== #

        # =================== Exit Button =================== #
        self.exit_button = ctk.CTkButton(master=self.mainFrame, corner_radius=10, command=self.exit, text="✕", text_color="white", font=("Arial", 20), width=30, height=30, fg_color="black", bg_color="black", hover_color=button_color)
        self.exit_button.grid(column=4, row=0, sticky="ne", padx=8, pady=4)

        # =================== Expand Button =================== #
        self.expand_button_var = tkinter.StringVar(value="-")
        self.expand_button = ctk.CTkButton(master=self.mainFrame, corner_radius=10, command=self.expand, textvariable=self.expand_button_var, text_color="white", font=("Arial", 20), width=40, height=30, fg_color="black", bg_color="black", hover_color=button_color)
        self.expand_button.grid(column=4, row=0, sticky="ne", padx=50, pady=4)

        # =================== CPU Indicator =================== #
        self.label_cpu_var = tkinter.StringVar(value="0 %")
        label_cpu = ctk.CTkLabel(master=self.mainFrame, textvariable=self.label_cpu_var, text_color="white", font=("Arial", 15), width=100, height=30, fg_color=button_color)
        label_cpu.grid(column=3, columnspan=2, row=0, sticky="nw", padx=12, pady=4)

        # =================== IPS Control Indicator =================== #
        self.label_ips_c_var = tkinter.StringVar(value="0")
        label_ips_c = ctk.CTkLabel(master=self.mainFrame, textvariable=self.label_ips_c_var, text_color="white", font=("Arial", 15), width=85, height=30, fg_color=button_color)
        label_ips_c.grid(column=3, columnspan=2, row=0, sticky="nw", padx=118, pady=4)

        # =================== Center Bar =================== #

        # =================== Hold Indicator =================== #
        self.label_hold_var = tkinter.StringVar(value="‖")
        label_hold = ctk.CTkLabel(master=self.mainFrame, corner_radius=5, textvariable=self.label_hold_var, text_color="white", font=("Arial", 15), width=40, height=30, fg_color=button_color, bg_color=button_color)
        label_hold.grid(column=2, row=0, sticky="n", padx=8, pady=4)

        # =================== Rotation Indicator =================== #
        self.label_rotation_var = tkinter.StringVar(value="-")
        label_rotation = ctk.CTkLabel(master=self.mainFrame, corner_radius=5, textvariable=self.label_rotation_var, text_color="white", font=("Arial", data_font_size), width=40, height=30, fg_color=button_color)
        label_rotation.grid(column=2, row=0, sticky="s", padx=8, pady=123)

        # =================== Gap Angle Indicator =================== #
        self.label_gap_angle_var = tkinter.StringVar(value="0")
        label_gap_angle = ctk.CTkLabel(master=self.mainFrame, corner_radius=5, textvariable=self.label_gap_angle_var, text_color="white", font=("Arial", 12), width=40, height=30, fg_color=button_color)
        label_gap_angle.grid(column=2, row=0, sticky="s", padx=8, pady=83)

        # =================== Direction Indicator =================== #
        self.label_turn_dir_var = tkinter.StringVar(value="⇧")
        label_turn_dir = ctk.CTkLabel(master=self.mainFrame, corner_radius=10, textvariable=self.label_turn_dir_var, text_color="white", font=("Arial", 25), width=40, height=30, fg_color=button_color, bg_color=button_color)
        label_turn_dir.grid(column=2, row=0, sticky="s", padx=8, pady=43)

        # =================== Angle Indicator =================== #
        self.label_angle_var = tkinter.StringVar(value="0")
        label_angle = ctk.CTkLabel(master=self.mainFrame, corner_radius=5, textvariable=self.label_angle_var, text_color="white", font=("Arial", data_font_size), width=40, height=30, fg_color=button_color)
        label_angle.grid(column=2, row=0, sticky="s", padx=8, pady=8)

    def expand(self):
        if self.expand_button_var.get() == "-":
            self.attributes("-topmost", False, "-fullscreen", False)
            self.expand_button_var.set("+")
        else:
            self.attributes("-topmost", True, "-fullscreen", True)
            self.expand_button_var.set("-")

    def exit(self):
        self.destroy()
        cam_1_stream.close()

        terminate.value = True

        time.sleep(0.5)

        for process in processes:
            process.terminate()
            time.sleep(.5)

    def exit_calibrate_color(self):
        calibrate_color_status.value = "none"
        self.color_calibration_button_var.set("✎")
        self.set_calibration_status()

    @staticmethod
    def set_calibration_status():
        if calibrate_color_status.value == "calibrate":
            calibration_type = ["none", "none", "none"]

            if calibration_color.value == "l-gz":
                calibration_type = ["green evac-point", "top", "line"]
            elif calibration_color.value == "l-rz":
                calibration_type = ["red evac-point", "top", "line"]
            elif calibration_color.value == "l-bz":
                calibration_type = ["black line", "center", "line"]
            elif calibration_color.value == "l-bn":
                calibration_type = ["black line", "top and bottom", "line"]
            elif calibration_color.value == "l-bv":
                calibration_type = ["black line", "top and bottom", "line"]
            elif calibration_color.value == "l-bvl":
                calibration_type = ["black line", "top", "line"]
            elif calibration_color.value == "l-bd":
                calibration_type = ["black line at a ramp down", "top", "line"]
            elif calibration_color.value == "l-gl":
                calibration_type = ["green marker", "center", "line"]
            elif calibration_color.value == "l-rl":
                calibration_type = ["red goal tile", "center", "line"]

            status.value = f'Please move the {calibration_type[0]} to the {calibration_type[1]} of the {calibration_type[2]} camera and press ⎚'

        elif calibrate_color_status.value == "check":
            image_type = ["none", "none"]

            if calibration_color.value == "l-gz":
                image_type = ["green evac-point", "line cam"]
            elif calibration_color.value == "l-rz":
                image_type = ["red evac-point", "line cam"]
            elif calibration_color.value == "l-bz":
                image_type = ["black zone", "line cam"]
            elif calibration_color.value == "l-bn":
                image_type = ["black normal", "line cam"]
            elif calibration_color.value == "l-bv":
                image_type = ["black silver validation (LED off)", "line cam"]
            elif calibration_color.value == "l-bvl":
                image_type = ["black silver validation (LED on)", "line cam"]
            elif calibration_color.value == "l-bd":
                image_type = ["black ramp down", "line cam"]
            elif calibration_color.value == "l-gl":
                image_type = ["green", "line cam"]
            elif calibration_color.value == "l-rl":
                image_type = ["red", "line cam"]

            status.value = f'This is the {image_type[0]} binary image ({image_type[1]}). Press ✓ to confirm'

        elif calibrate_color_status.value == "none":
            status.value = f'Stopped'

    def set_calibrate_color_status(self):
        if calibrate_color_status.value == "none":
            calibrate_color_status.value = "calibrate"
            self.color_calibration_button_var.set("⎚")
            if run_start_time.value == -1:
                run_start_time.value = time.perf_counter()

        elif calibrate_color_status.value == "calibrate":
            calibrate_color_status.value = "check"
            self.color_calibration_button_var.set("✓")

        elif calibrate_color_status.value == "check":
            calibrate_color_status.value = "none"
            self.color_calibration_button_var.set("✎")

        self.set_calibration_status()

    def choose_color(self):
        if calibrate_color_status.value == "none":
            # Line Cam Green Evacuation Point Calibration
            if calibration_color.value == "l-rl":
                calibration_color.value = "l-gz"
                self.color_choose_button_var.set("l-gz")

            # Line Cam Red Evacuation Point Calibration
            elif calibration_color.value == "l-gz":
                calibration_color.value = "l-rz"
                self.color_choose_button_var.set("l-rz")

            # Line Cam Black Zone Calibration
            elif calibration_color.value == "l-rz":
                calibration_color.value = "l-bz"
                self.color_choose_button_var.set("l-bz")

            # Line Cam Black Normal Calibration
            elif calibration_color.value == "l-bz":
                calibration_color.value = "l-bn"
                self.color_choose_button_var.set("l-bn")

            # Line Cam Black Silver Validation Calibration (LED off)
            elif calibration_color.value == "l-bn":
                calibration_color.value = "l-bv"
                self.color_choose_button_var.set("l-bv")

            # Line Cam Black Silver Validation Calibration (LED on)
            elif calibration_color.value == "l-bv":
                calibration_color.value = "l-bvl"
                self.color_choose_button_var.set("l-bvl")

            # Line Cam Black Ramp Down Calibration
            elif calibration_color.value == "l-bvl":
                calibration_color.value = "l-bd"
                self.color_choose_button_var.set("l-bd")

            # Line Cam Green Calibration
            elif calibration_color.value == "l-bd":
                calibration_color.value = "l-gl"
                self.color_choose_button_var.set("l-gl")

            # Line Cam Red Calibration
            elif calibration_color.value == "l-gl":
                calibration_color.value = "l-rl"
                self.color_choose_button_var.set("l-rl")

            self.set_calibration_status()

    def main(self, *args):
        global cam_1_stream, display_status

        self.label_angle_var.set(f"{line_angle.value} °")
        self.label_gap_angle_var.set(f"{gap_angle.value:.2f} °")
        self.label_cpu_var.set(f"CPU: {psutil.cpu_percent()} %")
        self.label_ips_c_var.set(f"IPS_C: {iterations_control.value}")

        if display_status != status.value:
            if len(status.value) > 60:
                self.label_status_font.configure(size=12)
            else:
                self.label_status_font.configure(size=15)

            self.label_status_var.set(f"... {status.value} ...")
            display_status = status.value

        if rotation_y.value == "none":
            self.label_rotation_var.set("n")
        elif rotation_y.value == "ramp_up":
            self.label_rotation_var.set("u")
        elif rotation_y.value == "ramp_down":
            self.label_rotation_var.set("d")

        if turn_dir.value == "straight" and objective.value == "follow_line":
            self.label_turn_dir_var.set("⇧")
        elif turn_dir.value == "left" and objective.value == "follow_line":
            self.label_turn_dir_var.set("⇦")
        elif turn_dir.value == "right" and objective.value == "follow_line":
            self.label_turn_dir_var.set("⇨")
        elif turn_dir.value == "turn_around" and objective.value == "follow_line":
            self.label_turn_dir_var.set("⟲")  # ⟳
        elif objective.value == "stop":
            self.label_turn_dir_var.set("II")

        if switch.value == 0:
            self.label_hold_var.set("II")
        elif switch.value == 1:
            self.label_hold_var.set("▶")

        if not run_start_time.value == -1:
            run_time = str(timedelta(seconds=time.perf_counter() - run_start_time.value)).replace(".", ":").split(":")
            try:
                self.label_timer_var.set(f'{run_time[1]}:{run_time[2]}:{run_time[3][:2]}')
            except IndexError:
                print(run_time, "Error in run_time")

        cam_1_stream = shared_memory.SharedMemory(name="shm_cam_1")
        bgr_img_arr_cam_1 = np.ndarray((252, 448, 3), dtype=np.uint8, buffer=cam_1_stream.buf)
        rgb_img_arr_cam_1 = cv2.cvtColor(bgr_img_arr_cam_1, cv2.COLOR_BGR2RGB)
        img_cam_1 = Image.fromarray(rgb_img_arr_cam_1)
        img_tks_cam_1 = ctk.CTkImage(img_cam_1, size=(camera_width_1, camera_height_1))
        self.top_camera.configure(image=img_tks_cam_1)

        delay = 100
        self.after(delay, self.main)


if __name__ == "__main__":
    program_start_time.value = time.perf_counter()

    processes = [
        Process(target=line_cam_loop, args=()),
        Process(target=control_loop, args=())
        ]

    for process in processes:
        process.start()
        print(process)
        time.sleep(0.5)

    app = App()
    app.main()
    app.mainloop()
