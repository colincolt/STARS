
bs = 1
diff = 1

def begin_command():
    print("3")

def pause_command():
    print("2")

def ball_speed_slider(slider_value):
    bs = slider_value
    print ("bs slider value = ", bs)

def difficulty_slider(slider_value):
    diff = slider_value
    print("diff slider value = ", diff)


def drill_2_selection():
    second_message.value = "Static Passing Selected"
    
    window2 = Window(app, bg="darkgreen")

    welcome_message2 = Text(window2, "Static Passing Parameters", size=25, font="Times New Roman", color="black")

    start_2 = PushButton(window2, command=begin_command, text="Begin")
    pause_2 = PushButton(window2, command=pause_command, text="Pause")
    start_2.width = 30
    start_2.bg = "gray"
    pause_2.width = 30
    pause_2.bg = "gray"
    
    third_message2 = Text(window2, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
    ball_speed2 = Slider(window2, command=ball_speed_slider, start=1, end=5)
    ball_speed2.width = 300
    
    final_message2 = Text(window2, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
    difficulty2 = Slider(window2, command=difficulty_slider, start=1, end=5)
    difficulty2.width = 300

def drill_3_selection():
    second_message.value = "Entering Manual Mode"
    
    window3 = Window(app, bg="darkgreen")

    welcome_message3 = Text(window3, "Manual Mode Controls", size=25, font="Times New Roman", color="black")

    start_3 = PushButton(window3, command=begin_command, text="Begin")
    pause_3 = PushButton(window3, command=pause_command, text="Pause")
    start_3.width = 30
    start_3.bg = "gray"
    pause_3.width = 30
    pause_3.bg = "gray"
    
    third_message3 = Text(window3, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
    ball_speed3 = Slider(window3, command=ball_speed_slider, start=1, end=5)
    ball_speed3.width = 300

def drill_4_selection():
    second_message.value = "Predictive Passing Selected"
    
    window4 = Window(app, bg="darkgreen")

    welcome_message4 = Text(window4, "Predictive Passing Parameters", size=25, font="Times New Roman", color="black")

    start_4 = PushButton(window4, command=begin_command, text="Begin")
    pause_4 = PushButton(window4, command=pause_command, text="Pause")
    start_4.width = 30
    start_4.bg = "gray"
    pause_4.width = 30
    pause_4.bg = "gray"
    
    third_message4 = Text(window4, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
    ball_speed4 = Slider(window4, command=ball_speed_slider, start=1, end=5)
    ball_speed4.width = 300


from guizero import App, Text, PushButton, Window, Slider, Picture

app = App(title="S.T.A.R.S. User Interface")

welcome_message = Text(app, "Welcome to S.T.A.R.S.", size=20, font="Times New Roman", color="red")
second_message = Text(app, "Let's play some soccer!", size=13, font="Times New Roman", color="darkgreen")
logo = Picture(app, image="logo.gif", align = "left")
logo.resize(200,200)

drill_2 = PushButton(app, command=drill_2_selection, text="Static Passing")
drill_2.width = 30
drill_4 = PushButton(app, command=drill_4_selection, text="Predictive Passing")
drill_4.width = 30

#### Manual mode:
drill_3 = PushButton(app, command=drill_3_selection, text="Manual Mode")
drill_3.width = 30

app.display()
