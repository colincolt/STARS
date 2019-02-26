from guizero import App, Text, PushButton, Window, Slider, Picture, CheckBox
import server_pi_v_15 as main_launcher
from threading import Thread, Event
import sys

shutdown_event = Event()
kill_event = Event()
send_flag = Event()

global startThread

# ##____________________GUI______________________##
class gui_app():
    def __init__(self):
        self.ballSpeed = 1
        self.difficulty = 1
        self.drillType = "No Drill"
        self.PitchYaw = True
        self.Launcher = True
        self.Evo = True

    def START(self, ballSpeed, difficulty, drillType):
        shutdown_event.clear()
        def StartProgram(ballSpeed, difficulty, drillType):
            main_launcher.startMainFile(ballSpeed, difficulty, drillType, shutdown_event, kill_event, send_flag, self.PitchYaw, self.Launcher, self.Evo)

        self.ballSpeed = ballSpeed
        self.difficulty = difficulty
        self.drillType = drillType
        startThread = Thread(target=StartProgram, args=[ballSpeed, difficulty, drillType])
        startThread.start()

    def stop_command(self):
        shutdown_event.set()
        print("This is the STOP command")

    def exit_command(self, sender):
        kill_event.set()
        print("EXITING...")
        try:
            startThread.join()
        except:
            pass
        sys.exit()

    def PitchYaw_Only_Cmd(self):
        self.Launcher = False

    def Launcher_Only_Cmd(self):
        self.PitchYaw = False

    def No_Evo(self):
        self.Evo = False

    def ball_speed_slider(self, slider_value):
        self.ballSpeed = str(slider_value)
        print("bs slider value = ", self.ballSpeed)

    def difficulty_slider(self, slider_value):
        self.difficulty = str(slider_value)
        print("diff slider value = ", self.difficulty)

    def start_command(self, ballSpeed, difficulty, drillType):
        self.START(ballSpeed, difficulty, drillType)
        # print("This is the START command")

    def staticDrill(self):
        second_message.value = "Static Passing Selected"
        self.drillType = "Static"

        window1 = Window(app, bg="darkgreen")

        Heading = Text(window1, "Static Passing Parameters", size=25, font="Times New Roman", color="black")

        Slide1 = Text(window1, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window1, command=self.ball_speed_slider, start=1, end=5)
        speed.width = 300
        speed.text_color = "white"

        Slide2 = Text(window1, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
        difficulty = Slider(window1, command=self.difficulty_slider, start=1, end=5)
        difficulty.width = 300
        difficulty.text_color = "white"

        Heading.text_color = "white"
        Slide1.text_color = "white"
        Slide2.text_color = "white"


        PitchYaw_1 = CheckBox(window1, command=self.PitchYaw_Only_Cmd, args=[], text="PitchYaw Only")
        Launcher_1 = CheckBox(window1, command=self.Launcher_Only_Cmd, args=[], text="Launcher Only")
        Evo_1 = CheckBox(window1, command=self.No_Evo, args=[], text="No Evo Lidar")
        start_1 = PushButton(window1, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType], text="Begin")
        stop_1 = PushButton(window1, command=self.stop_command, args=[], text="Stop")
        exit_1 = PushButton(window1, command=self.exit_command, args=["window"], text="Exit")
        PitchYaw_1.text_color = "white"
        Launcher_1.text_color = "white"
        Evo_1.text_color = "white"
        PitchYaw_1.width = 20
        Launcher_1.width = 20
        Evo_1.width = 20
        exit_1.width = 20
        exit_1.bg = "red"
        start_1.width = 30
        start_1.bg = "#469e00"
        stop_1.width = 30
        stop_1.bg = "#a0a000"
        exit_1.text_color = "white"
        start_1.text_color = "white"
        stop_1.text_color = "white"

    def predictiveDrill(self):
        second_message.value = "Entering Predictive Passing Mode"
        self.drillType = "Dynamic"
        window2 = Window(app, bg="darkgreen")

        Heading = Text(window2, "Predictive Passing Controls", size=25, font="Times New Roman", color="black")

        Slide1 = Text(window2, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window2, command=self.ball_speed_slider, start=1, end=5)
        speed.width = 300
        speed.text_color = "white"

        Slide2 = Text(window2, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
        difficulty = Slider(window2, command=self.difficulty_slider, start=1, end=5)
        difficulty.width = 300
        difficulty.text_color = "white"

        Heading.text_color = "white"
        Slide1.text_color = "white"
        Slide2.text_color = "white"

        PitchYaw_2 = CheckBox(window2, command=self.PitchYaw_Only_Cmd, args=[], text="PitchYaw Only")
        Launcher_2 = CheckBox(window2, command=self.Launcher_Only_Cmd, args=[], text="Launcher Only")
        Evo_2 = CheckBox(window2, command=self.No_Evo, args=[], text="No Evo Lidar")
        start_2 = PushButton(window2, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType], text="Begin")
        stop_2 = PushButton(window2, command=self.stop_command, args=[], text="stop")
        exit_2 = PushButton(window2, command=self.exit_command, args=["window"], text="Exit")
        PitchYaw_2.text_color = "white"
        Launcher_2.text_color = "white"
        Evo_2.text_color = "white"
        PitchYaw_2.width = 20
        Launcher_2.width = 20
        Evo_2.width = 20
        exit_2.width = 20
        exit_2.bg = "red"
        start_2.width = 30
        start_2.bg = "#469e00"
        stop_2.width = 30
        stop_2.bg = "#a0a000"
        exit_2.text_color = "white"
        start_2.text_color = "white"
        stop_2.text_color = "white"

    def manualDrill(self):
        second_message.value = "Entering Manual Mode"
        self.drillType = "Manual"

        window3 = Window(app, bg="darkgreen")

        Heading = Text(window3, "Manual Mode Controls", size=25, font="Times New Roman", color="black")
        Slide1 = Text(window3, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window3, command=self.ball_speed_slider, start=1, end=5)
        speed.width = 300
        speed.text_color = "white"

        Heading.text_color = "white"
        Slide1.text_color = "white"

        PitchYaw_3 = CheckBox(window3, command=self.PitchYaw_Only_Cmd, args=[], text="PitchYaw Only")
        Launcher_3 = CheckBox(window3, command=self.Launcher_Only_Cmd, args=[], text="Launcher Only")
        Evo_3 = CheckBox(window3, command=self.No_Evo, args=[], text="No Evo Lidar")
        start_3 = PushButton(window3, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType], text="Begin")
        stop_3 = PushButton(window3, command=self.stop_command, args=[], text="stop")
        exit_3 = PushButton(window3, command=self.exit_command, args=["window"], text="Exit")
        PitchYaw_3.text_color = "white"
        Launcher_3.text_color = "white"
        Evo_3.text_color = "white"
        PitchYaw_3.width = 20
        Launcher_3.width = 20
        Evo_3.width = 20
        exit_3.width = 20
        exit_3.bg = "red"
        start_3.width = 30
        start_3.bg = "#469e00"
        stop_3.width = 30
        stop_3.bg = "#a0a000"
        exit_3.text_color = "white"
        start_3.text_color = "white"
        stop_3.text_color = "white"


##_____Code that gets run:__________##
# help(STEREOMAINFILE_Server)
app = App(title="S.T.A.R.S. User Interface")

welcome_message = Text(app, "Welcome to S.T.A.R.S.", size=20, font="Times New Roman", color="red")
second_message = Text(app, "Let's play some soccer!", size=13, font="Times New Roman", color="darkgreen")
logo = Picture(app, image="logo.gif", align="left")
logo.resize(200, 200)

print('looping')

drill_1 = PushButton(app, command=gui_app().staticDrill, text="Static Passing")
drill_1.width = 30
drill_2 = PushButton(app, command=gui_app().predictiveDrill, text="Predictive Passing")
drill_2.width = 30
drill_3 = PushButton(app, command=gui_app().manualDrill, text="Manual Mode")
drill_3.width = 30
exit_1 = PushButton(app, command=gui_app().exit_command, args=["app"], text="Exit")
exit_1.width = 20
exit_1.bg = "red"

app.display()