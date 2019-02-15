from guizero import App, Text, PushButton, Window, Slider, Picture
import server_pi_v_08 as main_launcher
from threading import Thread, Event

shutdown_event = Event()
kill_event = Event()
# ##____________________GUI______________________##
class gui_app():
    def __init__(self):
        self.ballSpeed = 1
        self.difficulty = 1
        self.drillType = "No Drill"

    def START(self, ballSpeed, difficulty, drillType):
        def StartProgram(ballSpeed, difficulty, drillType):
            main_launcher.startMainFile(ballSpeed, difficulty, drillType, shutdown_event, kill_event)

        self.ballSpeed = ballSpeed
        self.difficulty = difficulty
        self.drillType = drillType
        global startThread
        startThread = Thread(target=StartProgram, args=[ballSpeed, difficulty, drillType])
        startThread.start()

    def stop_command(self):
        #startThread.join()
        shutdown_event.set()
        print("This is the STOP command")

    def exit_command(self):
        kill_event.set()
        print("This is the EXIT command")

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

        Text(window1, "Static Passing Parameters", size=25, font="Times New Roman", color="black")

        Text(window1, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window1, command=self.ball_speed_slider, start=1, end=5)
        speed.width = 300

        Text(window1, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
        difficulty = Slider(window1, command=self.difficulty_slider, start=1, end=5)
        difficulty.width = 300

        start_1 = PushButton(window1, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType], text="Begin")
        stop_1 = PushButton(window1, command=self.stop_command, args=[], text="stop")
        exit_1 = PushButton(window1, command=self.exit_command, args=[], text="stop")
        exit_1.width = 30
        exit_1.bg = "red"
        start_1.width = 30
        start_1.bg = "green"
        stop_1.width = 30
        stop_1.bg = "blue"

    def predictiveDrill(self):
        second_message.value = "Entering Predictive Passing Mode"
        self.drillType = "Dynamic"
        window2 = Window(app, bg="darkgreen")

        Text(window2, "Predictive Passing Controls", size=25, font="Times New Roman", color="black")

        Text(window2, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window2, command=self.ball_speed_slider, start=1, end=5)
        speed.width = 300

        Text(window2, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
        difficulty = Slider(window2, command=self.difficulty_slider, start=1, end=5)
        difficulty.width = 300

        start_2 = PushButton(window2, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType], text="Begin")
        stop_2 = PushButton(window2, command=self.stop_command, args=[], text="stop")
        start_2.width = 30
        start_2.bg = "gray"
        stop_2.width = 30
        stop_2.bg = "gray"

    def manualDrill(self):
        second_message.value = "Entering Manual Mode"
        gui_app.drillType = "Manual"

        window3 = Window(app, bg="darkgreen")

        Text(window3, "Manual Mode Controls", size=25, font="Times New Roman", color="black")

        Text(window3, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window3, command=self.ball_speed_slider, start=1, end=5)
        speed.width = 300

        start_3 = PushButton(window3, command=self.start_command, args=[self.ballSpeed, self.difficulty, self.drillType], text="Begin")
        stop_3 = PushButton(window3, command=self.stop_command, args=[], text="stop")
        start_3.width = 30
        start_3.bg = "gray"
        stop_3.width = 30
        stop_3.bg = "gray"


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

app.display()