from guizero import App, Text, PushButton, Window, Slider, Picture, TextBox
import include.main_file as main
import include.launch_test as test
import include.stereo as stereo
from threading import Thread
import multiprocessing as mp
import sys
import time

pause_event = mp.Event()
kill_event = mp.Event()
close_launch = mp.Event()
show_camera = mp.Event()


global startThread

# ##____________________GUI______________________##
class gui_app():
    def __init__(self):
        # PROGRAM OPTIONS:
        self.PitchYaw = True
        self.Launcher = True
        self.Evo = False


        self.ballSpeed = 1
        self.difficulty = 1
        self.drillType = "No Drill"
        self.motor_data = "<0,0,0,0,0,0>"


    def START(self, ballSpeed, difficulty, drillType):
        if pause_event.is_set():
            pause_event.clear()
        else:
            def StartProgram(ballSpeed, difficulty, drillType):
                main.startMainFile(ballSpeed, difficulty, drillType, pause_event, kill_event, self.PitchYaw, self.Launcher, self.Evo, show_camera)

            self.ballSpeed = ballSpeed
            self.difficulty = difficulty
            self.drillType = drillType
            self.startThread = Thread(target=StartProgram, args=[ballSpeed, difficulty, drillType])
            self.startThread.start()

    def pause_command(self, sender):
        sender = sender
        senderstr = str(sender)
        try:
            if self.startThread.isAlive():
                pause_event.set()
        except:
            print("[GUI] : Drill not started")

    def exit_command(self, appname, window):

        appname = appname
        senderstr = str(appname)
        window = window
        try:
            if self.startThread.isAlive():
                kill_event.set()
                time.sleep(10)
                print("[GUI] : EXITING...")
                thread_not_joined = False
                while thread_not_joined:
                    try:
                        self.startThread.join()
                        print("[GUI] : in try")
                        thread_not_joined = True
                    except:
                        print("[GUI] : in except")
                        continue
                window.hide()
                appname.destroy()

                print("[GUI] : closing")
                sys.exit()
        except:
            window.hide()

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

    def distance_slider(self, slider_value):
        self.distance = str(slider_value)
        self.motor_data = test.return_data(self.distance)
        self.motor_data = self.motor_data.split("_")
        print(self.motor_data)
        self.textbox.value =self.motor_data


    def start_command(self, ballSpeed, difficulty, drillType):
        self.START(ballSpeed, difficulty, drillType)
        # print("This is the START command")

    def app_exit(self,app):
        try:
            if self.startThread.isAlive():
                self.exit_command(app)
        except AttributeError as e:
            sys.exit()

    def send_data(self):
        run_launch = test.run(self.motor_data, close_launch)

    def close_test(self, window):
        close_launch.set()
        window.hide()
        # sys.exit()

    def show_cam(self):
        if show_camera.is_set():
            show_camera.clear()
        else:
            show_camera.set()
            print("[GUI] : Showing the camera")


    def staticDrill(self,appname):
        appname = appname
        second_message.value = "Static Passing Selected"
        self.drillType = "Static"

        self.window1 = Window(app, bg="darkgreen",height=320, width=480, layout="grid")
        logo = Picture(self.window1, image="include/logo.gif", align="left", grid=[0, 0])
        logo.resize(75, 75)

        Heading = Text(self.window1, "Static Passing Parameters", size=14, font="Times New Roman", color="white",grid=[1,0,2,1])

        Slide1 = Text(self.window1, "Please select a passing speed:", size=12, font="Times New Roman", color="white",grid=[1,1,2,1])
        speed = Slider(self.window1, command=self.ball_speed_slider, start=1, end=5,grid=[1,2,2,1])
        speed.width = 300
        speed.text_color="black"
        speed.bg = "grey"

        Slide2 = Text(self.window1, "Please select a difficulty:", size=12, font="Times New Roman", color="white",grid=[1,3,2,1])
        difficulty = Slider(self.window1, command=self.difficulty_slider, start=1, end=5,grid=[1,4,2,1])
        difficulty.width = 300
        difficulty.text_color="black"
        difficulty.bg = "grey"

        start = PushButton(self.window1, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType], text="Begin", grid=[1,5])
        start.width = 15
        start.bg = "#469e00"
        start.text_color = "white"

        stop = PushButton(self.window1, command=self.pause_command, args=[self.window1], text="Pause",grid=[2,5])
        stop.width = 15
        stop.bg = "#a0a000"
        stop.text_color = "white"

        exit_win = PushButton(self.window1, command=self.exit_command, args=[appname,self.window1], text="Stop",grid=[1,6,2,1])
        exit_win.width = 20
        exit_win.bg = "red"
        exit_win.text_color = "white"

        camera = PushButton(self.window1, command=self.show_cam, args=[], text="ShowCam",grid=[5,0])
        camera.width = 8
        camera.height = 3
        camera.bg = "grey"
        camera.text_color = "black"


    def predictiveDrill(self,appname):
        appname = appname
        second_message.value = "Entering Predictive Passing Mode"

        self.drillType = "Dynamic"
        self.window2 = Window(app, bg="darkgreen",height=320, width=480, layout="grid")

        logo = Picture(self.window2, image="include/logo.gif", align="left", grid=[0, 0])
        logo.resize(75, 75)

        Heading = Text(self.window2, "Predictive Passing Controls", size=14, font="Times New Roman", color="white",grid=[1,0,2,1])

        Slide1 = Text(self.window2, "Please select a passing speed:", size=12, font="Times New Roman", color="white",
                      grid=[1, 1, 2, 1])
        speed = Slider(self.window2, command=self.ball_speed_slider, start=1, end=5, grid=[1, 2, 2, 1])
        speed.width = 300
        speed.text_color = "black"
        speed.bg = "grey"

        Slide2 = Text(self.window2, "Please select a difficulty:", size=12, font="Times New Roman", color="white",
                      grid=[1, 3, 2, 1])
        difficulty = Slider(self.window2, command=self.difficulty_slider, start=1, end=5, grid=[1, 4, 2, 1])
        difficulty.width = 300
        difficulty.text_color = "black"
        difficulty.bg = "grey"

        start = PushButton(self.window2, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType],
                           text="Begin", grid=[1, 5])
        start.width = 15
        start.bg = "#469e00"
        start.text_color = "white"

        stop = PushButton(self.window2, command=self.pause_command, args=[self.window2], text="Pause", grid=[2, 5])
        stop.width = 15
        stop.bg = "#a0a000"
        stop.text_color = "white"

        exit_win = PushButton(self.window2, command=self.exit_command, args=[appname, self.window2], text="Stop", grid=[1, 6, 2, 1])
        exit_win.width = 20
        exit_win.bg = "red"
        exit_win.text_color = "white"

        camera = PushButton(self.window2, command=self.show_cam, args=[], text="ShowCam", grid=[5, 0])
        camera.width = 8
        camera.height = 3
        camera.bg = "grey"
        camera.text_color = "black"



    def manualDrill(self,appname):
        second_message.value = "Entering Manual Mode"
        self.drillType = "Manual"

        self.window3 = Window(app, bg="darkgreen",height=320, width=480, layout="grid")
        logo = Picture(self.window3, image="include/logo.gif", align="left", grid=[0, 0])
        logo.resize(75, 75)
        Heading = Text(self.window3, "Manual Mode Controls", size=25, font="Times New Roman", color="white",grid=[1,0,2,1])
        Slide1 = Text(self.window3, "Please select a passing speed:", size=12, font="Times New Roman", color="white", grid=[1, 1, 2, 1])
        speed = Slider(self.window3, command=self.ball_speed_slider, start=1, end=5, grid=[1, 2, 2, 1])
        speed.width = 300
        speed.text_color = "black"
        speed.bg = "grey"


        start = PushButton(self.window3, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType],
                           text="Begin", grid=[1, 5])
        start.width = 15
        start.bg = "#469e00"
        start.text_color = "white"

        stop = PushButton(self.window3, command=self.pause_command, args=[self.window3], text="Pause", grid=[2, 5])
        stop.width = 15
        stop.bg = "#a0a000"
        stop.text_color = "white"

        exit_win = PushButton(self.window3, command=self.exit_command, args=[appname, self.window3], text="Stop", grid=[1, 6, 2, 1])
        exit_win.width = 20
        exit_win.bg = "red"
        exit_win.text_color = "white"

        camera = PushButton(self.window3, command=self.show_cam, args=[], text="ShowCam", grid=[5, 0])
        camera.width = 8
        camera.height = 3
        camera.bg = "grey"
        camera.text_color = "black"

    def user_input(self, appname):
        appname = appname
        # self.textbox.value = "<0,0,0,0,0,0>"
        second_message.value = "User Input Selected"

        self.window4 = Window(app, bg="darkgreen", height=320, width=480)

        Text(self.window4, "User Input Mode", size=14, font="Times New Roman", color="white")

        Text(self.window4, "Please select a distance:", size=12, font="Times New Roman", color="white")
        distance = Slider(self.window4, command=self.distance_slider, start=5, end=25)
        self.textbox = TextBox(self.window4)
        self.textbox.width = 30

        send = PushButton(self.window4, command=self.send_data, args=[], text="LAUNCH")
        send.width = 30

        exit = PushButton(self.window4, command=self.close_test, args=[self.window4], text="Close")
        exit.bg = "red"



##_____Code that gets run:__________##
# help(STEREOMAINFILE_Server)
app = App(title="S.T.A.R.S. User Interface", layout="grid", height=320, width=480)

welcome_message = Text(app, "Welcome to S.T.A.R.S.", size=20, font="Times New Roman", color="red", grid=[1,0,2,1])
second_message = Text(app, "Let's play some soccer!", size=13, font="Times New Roman", color="darkgreen",grid=[1,1,2,1])
logo = Picture(app, image="include/logo.gif", align="left", grid=[0,0])
logo.resize(75, 75)

print('looping')

drill_1 = PushButton(app, command=gui_app().staticDrill, args=[app], text="Static Passing", grid=[1,2],width=17)
# drill_1.width = 30
drill_2 = PushButton(app, command=gui_app().predictiveDrill,args=[app], text="Predictive Passing",grid=[2,2],width=17)
# drill_2.width = 30
drill_3 = PushButton(app, command=gui_app().manualDrill,args=[app], text="Manual Mode",grid=[1,3],width=17)
# drill_3.width = 30
input_mode = PushButton(app, command=gui_app().user_input, args=[app], text="User Input",grid=[2,3],width=17)

exit_main = PushButton(app, command=gui_app().app_exit, args=[app], text="Exit",grid=[1,4,2,4])
exit_main.width = 20
exit_main.bg = "red"

app.display()
