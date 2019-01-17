from guizero import App, Text, PushButton, Window, Slider, Picture, TextBox
import start_program as main
import time
##########___________GUI______________##################
bs = 1
diff = 1
drillType = "None"
args = 1
trigger = 0

def stop_command():
    print("This is the STOP command")

def ball_speed_slider(slider_value):
    bs = slider_value
    textbox.value = slider_value
    return bs

def difficulty_slider(slider_value):
    diff = slider_value
    textbox.value = slider_value
    return diff
    
def sendData(bs, diff, drill,arg):
    main.startMainFile(bs, diff, drill, arg)
        
def staticDrill():

    second_message.value = "Static Passing Selected"
    drillType = "static"
    
    window1 = Window(app, bg="darkgreen")

    welcome_message2 = Text(window1, "Static Passing Parameters", size=25, font="Times New Roman", color="black")

    third_message1 = Text(window1, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
    speed = Slider(window1, command=ball_speed_slider, start=1, end=5)
    speed.width = 300
    
    final_message1 = Text(window1, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
    difficulty = Slider(window1, command=difficulty_slider, start=1, end=5)
    difficulty.width = 300

    args = 1
    
    start_1 = PushButton(window1, command=sendData(), text="Begin")
    stop_1 = PushButton(window1, command=stop_command, text="stop")
    start_1.width = 30
    start_1.bg = "gray"
    stop_1.width = 30
    stop_1.bg = "gray"
    
    return drillType, args
    
  
def predictiveDrill():
    second_message.value = "Entering Predictive Passing Mode"
    drillType = "dynamic"
    window2 = Window(app, bg="darkgreen")

    welcome_message2 = Text(window2, "Predictive Passing Controls", size=25, font="Times New Roman", color="black")

    third_message2 = Text(window2, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
    speed = Slider(window2, command=ball_speed_slider, start=1, end=5)
    speed.width = 300
    
    final_message1 = Text(window2, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
    difficulty = Slider(window2, command=difficulty_slider, start=1, end=5)
    difficulty.width = 300
    
    args = 1
    
    start_2 = PushButton(window2, command=main.startMainFile(speed, difficulty, drillType, args), text="Begin")
    stop_2 = PushButton(window2, command=stop_command, text="stop")
    start_2.width = 30
    start_2.bg = "gray"
    stop_2.width = 30
    stop_2.bg = "gray"
      

def manualDrill():
    second_message.value = "Entering Manual Mode"
    drillType = "manual"

    window3 = Window(app, bg="darkgreen")

    welcome_message3 = Text(window3, "Manual Mode Controls", size=25, font="Times New Roman", color="black")

    third_message3 = Text(window3, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
    speed = Slider(window3, command=ball_speed_slider, start=1, end=5)
    speed.width = 300

    difficulty = 0
    args = 1
    
    start_3 = PushButton(window3, command=sendData(),args = [bs,diff,drillType,args], text="Begin")
    stop_3 = PushButton(window3, command=stop_command, text="stop")
    start_3.width = 30
    start_3.bg = "gray"
    stop_3.width = 30
    stop_3.bg = "gray"
    
    
##_____Code that gets run:__________##
#help(STEREOMAINFILE_Server)
        
app = App(title="S.T.A.R.S. User Interface")
textbox = TextBox(app)

welcome_message = Text(app, "Welcome to S.T.A.R.S.", size=20, font="Times New Roman", color="red")
second_message = Text(app, "Let's play some soccer!", size=13, font="Times New Roman", color="darkgreen")
logo = Picture(app, image="logo.gif", align = "left")
logo.resize(200,200)

speed = 0
difficulty = 0
drillType = 0
args = 0

drill_1 = PushButton(app, command=staticDrill, text="Static Passing")
drill_1.width = 30
drill_2 = PushButton(app, command=predictiveDrill, text="Predictive Passing")
drill_2.width = 30
drill_3 = PushButton(app, command=manualDrill, text="Manual Mode")
drill_3.width = 30


app.display()
