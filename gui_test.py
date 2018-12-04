def drill_1_selection():
    second_message.value = "Target Drill Selected"

    window1 = Window(app, bg="darkgreen")

    welcome_message1 = Text(window1, "Target Drill Parameters", size=25, font="Times New Roman", color="black")

    start_1 = PushButton(window1, command=drill_1_selection, text="Start")
    pause_1 = PushButton(window1, command=drill_2_selection, text="Pause")
    start_1.width = 30
    start_1.bg = "gray"
    pause_1.width = 30
    pause_1.bg = "gray"
    
    second_message1 = Text(window1, "How many targets would you like to use?", size=15, font="Times New Roman", color="black")
    targets = Slider(window1, command=print("Okay"), start=1, end=4)
    third_message1 = Text(window1, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
    ball_speed = Slider(window1, command=print("Okay"), start=1, end=10)
    ball_speed.width = 300


def drill_2_selection():
    second_message.value = "Static Passing Selected"
    
    window2 = Window(app, bg="darkgreen")

    welcome_message2 = Text(window2, "Static Passing Parameters", size=25, font="Times New Roman", color="black")

    start_2 = PushButton(window2, command=powerON, text="Start")
    pause_2 = PushButton(window2, command=powerON, text="Pause")
    start_2.width = 30
    start_2.bg = "gray"
    pause_2.width = 30
    pause_2.bg = "gray"
    
    #second_message2 = Text(window2, "How many targets would you like to use?", size=15, font="Times New Roman", color="black")
    #targets = Slider(window1, command=print("Okay"), start=1, end=4)
    third_message2 = Text(window2, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
    ball_speed2 = Slider(window2, command=print("Okay"), start=1, end=10)
    ball_speed2.width = 300

def drill_3_selection():
    second_message.value = "Keeper Practice Selected"
    
    window3 = Window(app, bg="darkgreen")

    welcome_message3 = Text(window3, "Keeper Practice Parameters", size=25, font="Times New Roman", color="black")

    start_3 = PushButton(window3, command=powerON, text="Start")
    pause_3 = PushButton(window3, command=powerON, text="Pause")
    start_3.width = 30
    start_3.bg = "gray"
    pause_3.width = 30
    pause_3.bg = "gray"
    
    #second_message3 = Text(window3, "How many targets would you like to use?", size=15, font="Times New Roman", color="black")
    #targets = Slider(window3, command=print("Okay"), start=1, end=4)
    third_message3 = Text(window3, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
    ball_speed3 = Slider(window3, command=print("Okay"), start=1, end=10)
    ball_speed3.width = 300


def drill_4_selection():
    second_message.value = "Predictive Passing Selected"
    
    window4 = Window(app, bg="darkgreen")

    welcome_message4 = Text(window4, "Predictive Passing Parameters", size=25, font="Times New Roman", color="black")

    start_4 = PushButton(window4, command=powerON, text="Start")
    pause_4 = PushButton(window4, command=powerON, text="Pause")
    start_4.width = 30
    start_4.bg = "gray"
    pause_4.width = 30
    pause_4.bg = "gray"
    
    #second_message4 = Text(window4, "How many targets would you like to use?", size=15, font="Times New Roman", color="black")
    #targets = Slider(window4, command=print("Okay"), start=1, end=4)
    third_message4 = Text(window4, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
    ball_speed4 = Slider(window4, command=print("Okay"), start=1, end=10)
    ball_speed4.width = 300

def powerON():
    print("Drill initiated.")
    if GPIO.input(40):
        GPIO.output(40,GPIO.LOW)
        start_1["text"] = "Start"
    else:
        GPIO.output(40,GPIO.HIGH)
        start_1["text"] = "Pause"

from guizero import App, Text, PushButton, Window, Slider, Picture
#from tkinter import *
#import RPi.GPIO as GPIO

app = App(title="S.T.A.R.S. User Interface")

welcome_message = Text(app, "Welcome to S.T.A.R.S.", size=20, font="Times New Roman", color="red")
second_message = Text(app, "Let's play some soccer!", size=13, font="Times New Roman", color="darkgreen")
logo = Picture(app, image="logo.gif", align = "left")
logo.resize(50,45)

drill_1 = PushButton(app, command=drill_1_selection, text="Target Drill")
drill_1.width = 30
drill_2 = PushButton(app, command=drill_2_selection, text="Static Passing")
drill_2.width = 30
drill_3 = PushButton(app, command=drill_3_selection, text="Keeper Practice")
drill_3.width = 30
drill_4 = PushButton(app, command=drill_4_selection, text="Predictive Passing")
drill_4.width = 30


app.display()
