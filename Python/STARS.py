#!/bin/python
from guizero import App, Text, PushButton, Window, Slider, Picture, TextBox, Box
import include.main_file as main
import include.launch_test as test
#import include.stereo_pool as stereo
from threading import Thread
import multiprocessing as mp
import sys
import time

pause_event = mp.Event()
kill_event = mp.Event()
close_launch = mp.Event()
show_camera = mp.Event()
voice_control = mp.Event()


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
        self.motor_data = "<0,0,0,0,0,0,0>"


    def START(self, ballSpeed, difficulty, drillType):
        if pause_event.is_set():
            pause_event.clear()
        if kill_event.is_set():
            kill_event.clear()
        else:
            def StartProgram(ballSpeed, difficulty, drillType):
                main.startMainFile(ballSpeed, difficulty, drillType, pause_event, kill_event, self.PitchYaw, self.Launcher, self.Evo, show_camera,voice_control).run()

            self.ballSpeed = ballSpeed
            self.difficulty = difficulty
            self.drillType = drillType
            self.startThread = Thread(target=StartProgram, args=[ballSpeed, difficulty, drillType])
            if not self.startThread.isAlive():
                self.startThread.start()

    def pause_command(self, sender):
        sender = sender
        senderstr = str(sender)
        try:
            if self.startThread.isAlive():
                if pause_event.is_set():
                    pause_event.clear()
                else:
                    pause_event.set()
        except:
            print("[GUI] : Drill not started")

    def exit_command(self, appname, window):
        if pause_event.is_set():
            pause_event.clear()
        appname = appname
        senderstr = str(appname)
        window = window
        try:
            if self.startThread.isAlive():
                kill_event.set()
                time.sleep(1)
                print("[GUI] : Closing Drill Window...")
                thread_not_joined = False
                while thread_not_joined:
                    try:
                        self.startThread.join()
                        thread_not_joined = True
                    except:
                        continue

        except:
            print("Leaving before starting drill?")
        finally:
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
        print("")
        print("----------------------------")
        print("Application Closing...")
        print("----------------------------")

        try:
            if self.startThread.isAlive():
                self.exit_command(app)
        except AttributeError as e:
            sys.exit()

    def send_data(self):
        def Start_Launch(motor_data, close_launch):
            test.run(motor_data, close_launch)

        self.StartL = Thread(target=Start_Launch(self.motor_data, close_launch))
        self.StartL.start()

    def close_test(self, window):
        close_launch.set()
        try:
            self.StartL.join()
            window.hide()
        except:
            window.hide()

    def show_cam(self):
        if show_camera.is_set():
            show_camera.clear()
            print("[GUI] : Hiding the camera")
        else:
            show_camera.set()
            print("[GUI] : Showing the camera")

    def enable_voice(self):
        if voice_control.is_set():
            voice_control.clear()
            print("[GUI] : Voice Control disabled")
        else:
            voice_control.set()
            print("[GUI] : Voice Control enabled")


    def staticDrill(self,appname):
        from guizero import Box
        appname = appname
        second_message.value = "Static Passing Selected"
        self.drillType = "Static"

        self.window1 = Window(app, bg="#424242",height=320, width=480, layout="grid")
        # logo = Picture(self.window1, image="include/logo.gif", align="left", grid=[0, 0])
        # logo.resize(40, 40)

        Heading = Text(self.window1, "Basic Tracking", size=18, font="Calibri Bold", color="white",grid=[1,0,3,1])

        Slide1 = Text(self.window1, "Ball Speed:", size=14, font="Calibri Bold", color="white",
                      grid=[1, 1])  # , 2, 1])
        speed = Slider(self.window1, command=self.ball_speed_slider, start=1, end=5, grid=[1, 2])
        speed.width = 150
        speed.text_color = "black"
        speed.bg = "white"
        speed.text_size=14

        Slide2 = Text(self.window1, "Pass Difficulty:", size=14, font="Calibri Bold", color="white",
                      grid=[3, 1])
        difficulty = Slider(self.window1, command=self.difficulty_slider, start=1, end=5, grid=[3, 2])
        difficulty.width = 150
        difficulty.text_color = "black"
        difficulty.bg = "white"
        difficulty.text_size=14

        start = PushButton(self.window1, command=self.start_command,
                           args=[self.ballSpeed, self.difficulty, self.drillType],
                           image="include/images/startbut.png", grid=[1, 5])
        start.bg = "#37f100"
        start.text_color = "white"

        stop = PushButton(self.window1, command=self.pause_command, args=[self.window1],
                          image="include/images/pausebut.png", grid=[3, 5])
        stop.bg = "#ffb000"
        stop.text_color = "white"

        exit_win = PushButton(self.window1, command=self.exit_command, args=[appname, self.window1],
                              image="include/images/stopbut.png", grid=[1, 6])
        exit_win.bg = "#e9002a"
        exit_win.text_color = "white"

        camera = PushButton(self.window1, command=self.show_cam, args=[], image="include/images/camerabut.png",
                            grid=[3, 6])
        camera.bg = "#002ff5"
        camera.text_color = "white"

        Center = Box(self.window1, width=50, height=165, grid=[2, 1, 1, 7])
        Left = Box(self.window1, width=60, height=200, grid=[0, 0, 1, 7])
        Right = Box(self.window1, width=20, height=200, grid=[4, 0, 1, 7])

        VC = PushButton(Center, command=self.enable_voice, args=[],
                      image="include/images/micbut.png", align="bottom")

    def predictiveDrill(self,appname):
        appname = appname
        second_message.value = "Entering Predictive Passing Mode"
        from guizero import Box

        self.drillType = "Dynamic"
        self.window2 = Window(app, bg="#424242",height=320, width=480, layout="grid")


        # logo = Picture(self.window2, image="include/logo.gif", align="left", grid=[0, 0])
        # logo.resize(75, 75)

        Heading = Text(self.window2, "Predictive Tracking", size=18, font="Calibri Bold", color="white",grid=[1,0,3,1])

        Slide1 = Text(self.window2, "Ball Speed:", size=14, font="Calibri Bold", color="white",
                      grid=[1, 1]) #, 2, 1])
        speed = Slider(self.window2, command=self.ball_speed_slider, start=1, end=5, grid=[1, 2])
        speed.width = 150
        speed.text_color = "black"
        speed.bg = "white"
        speed.text_size=14

        Slide2 = Text(self.window2, "Pass Difficulty:", size=14, font="Calibri Bold", color="white", grid=[3, 1])
        difficulty = Slider(self.window2, command=self.difficulty_slider, start=1, end=5, grid=[3, 2]) #, 2, 1])
        difficulty.width = 150
        difficulty.text_color = "black"
        difficulty.bg = "white"
        difficulty.text_size=14

        start = PushButton(self.window2, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType],
                           image="include/images/startbut.png", grid=[1, 5])
        start.bg = "#37f100"
        start.text_color = "white"

        stop = PushButton(self.window2, command=self.pause_command, args=[self.window2], image="include/images/pausebut.png", grid=[3, 5])
        stop.bg = "#ffb000"
        stop.text_color = "white"

        exit_win = PushButton(self.window2, command=self.exit_command, args=[appname, self.window2], image="include/images/stopbut.png", grid=[1, 6])
        exit_win.bg = "#e9002a"
        exit_win.text_color = "white"

        camera = PushButton(self.window2, command=self.show_cam, args=[], image="include/images/camerabut.png", grid=[3, 6])
        camera.bg = "#002ff5"
        camera.text_color = "white"

        Center = Box(self.window2,width=50,height=165,grid=[2,1,1,7])
        Left = Box(self.window2,width=60,height=200,grid=[0,0,1,7])
        Right = Box(self.window2,width=20,height=200,grid=[4,0,1,7])

        VC = PushButton(Center, command=self.enable_voice, args=[],
                        image="include/images/micbut.png", align="bottom")



    def manualDrill(self,appname):
        from guizero import Box
        second_message.value = "Entering Manual Mode"
        self.drillType = "Manual"

        self.window3 = Window(app, bg="#424242",height=320, width=480, layout="grid")
        # logo = Picture(self.window3, image="include/logo.gif", align="left", grid=[0, 0])
        # logo.resize(75, 75)
        Heading = Text(self.window3, "Voice Activated Launch", size=18, font="Calibri Bold", color="white",grid=[1,0,3,1])

        Slide1 = Text(self.window3, "Ball Speed:", size=14, font="Calibri Bold", color="white",
                      grid=[1, 1])  # , 2, 1])
        speed = Slider(self.window3, command=self.ball_speed_slider, start=1, end=5, grid=[1, 2])
        speed.width = 150
        speed.text_color = "black"
        speed.bg = "white"
        speed.text_size=14

        Slide2 = Text(self.window3, "Pass Difficulty:", size=14, font="Calibri Bold", color="white",
                      grid=[3, 1])
        difficulty = Slider(self.window3, command=self.difficulty_slider, start=1, end=5, grid=[3, 2])  # , 2, 1])
        difficulty.width = 150
        difficulty.text_color = "black"
        difficulty.bg = "white"
        difficulty.text_size=14

        start = PushButton(self.window3, command=self.start_command,
                           args=[self.ballSpeed, self.difficulty, self.drillType],
                           image="include/images/startbut.png", grid=[1, 5])
        start.bg = "#37f100"
        start.text_color = "white"

        stop = PushButton(self.window3, command=self.pause_command, args=[self.window3],
                          image="include/images/pausebut.png", grid=[3, 5])
        stop.bg = "#ffb000"
        stop.text_color = "white"

        exit_win = PushButton(self.window3, command=self.exit_command, args=[appname, self.window3],
                              image="include/images/stopbut.png", grid=[1, 6])
        exit_win.bg = "#e9002a"
        exit_win.text_color = "white"

        camera = PushButton(self.window3, command=self.show_cam, args=[], image="include/images/camerabut.png",
                            grid=[3, 6])
        camera.bg = "#002ff5"
        camera.text_color = "white"

        Center = Box(self.window3, width=50, height=200, grid=[2, 1, 1, 7])
        Left = Box(self.window3, width=60, height=200, grid=[0, 0, 1, 7])
        Right = Box(self.window3, width=20, height=200, grid=[4, 0, 1, 7])

    def user_input(self, appname):
        from guizero import Box
        appname = appname
        second_message.value = "User Input Selected"

        self.window4 = Window(app, bg="#424242", height=320, width=480)

        Text(self.window4, "User Input Mode", size=18, font="Calibri Bold", color="white")

        Text(self.window4, "Please select a distance:", size=14, font="Calibri Bold", color="white")
        distance = Slider(self.window4, command=self.distance_slider, start=5, end=25)
        distance.text_size=14
        Box(self.window4,width=100,height=10)
        distance.bg="white"
        self.textbox = TextBox(self.window4)
        self.textbox.width = 36
        self.textbox.bg = "white"
        self.textbox.text_size=14
        Box(self.window4,width=100,height=10)

        send = PushButton(self.window4, command=self.send_data, args=[], image="include/images/startbut.png")
        # send.width = 30
        send.bg="#37f100"

        exit = PushButton(self.window4, command=self.close_test, args=[self.window4], image="include/images/stopbut.png",)
        exit.bg = "#e9002a"


##_____Code that gets run:__________##
# help(STEREOMAINFILE_Server)
app = App(title="S.T.A.R.S. User Interface", layout="grid", height=320, width=480, bg="white",visible=True)
# MainBox = Box(app,grid=[0,0])

welcome_message = Text(app, "Welcome to S.T.A.R.S.", size=20, font="Calibri Bold", color="red", grid=[1,0,2,1])
second_message = Text(app, "Please select a drill: ", size=14, font="Calibri Bold", color="green",grid=[1,1,2,1])
logo = Picture(app, image="include/images/logo.gif", align="left", grid=[0,0])
logo.resize(75, 75)
logoright = Picture(app, image="include/images/logo.gif", align="left", grid=[3,0])
logoright.resize(75, 75)

print('looping')
drill_1 = PushButton(app, command=gui_app().staticDrill, args=[app] ,text="Basic Tracking", grid=[1,2],width=17)
# drill_1.width = 30
drill_1.font="Calibri Bold"
drill_1.bg="blue"
drill_1.text_color="white"
# drill_1.text_size=12

drill_2 = PushButton(app, command=gui_app().predictiveDrill,args=[app], text="Predictive Tracking",grid=[2,2],width=17)
# drill_2.width = 30
drill_2.font="Calibri Bold"
drill_2.bg="blue"
drill_2.text_color="white"
# drill_2.text_size=12

drill_3 = PushButton(app, command=gui_app().manualDrill,args=[app], text="VC w/Basic Tracking",grid=[1,3],width=17)
# drill_3.width = 30
drill_3.font="Calibri Bold"
drill_3.bg="blue"
drill_3.text_color="white"
# drill_3.text_size=12

input_mode = PushButton(app, command=gui_app().user_input, args=[app], text="User Input",grid=[2,3],width=17)
input_mode.bg="blue"
input_mode.text_color="white"
# input_mode.text_size=12

exit_main = PushButton(app, command=gui_app().app_exit, args=[app], image="include/images/stopbut.png",grid=[1,4,2,4])
# exit_main.width = 20
exit_main.bg = "#e9002a"

app.display()
