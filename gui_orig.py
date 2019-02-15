from guizero import App, Text, PushButton, Window, Slider, Picture
import start_program as main
##########___________GUI______________##################
class STARS_aPP(object):
    ballSpeed = 1
    difficulty = 1
    drillType = "No Drill"
            
    def stop_command():
        print("This is the STOP command")

    def ball_speed_slider(slider_value):
        STARS_aPP.ballSpeed = str(slider_value)
        print ("bs slider value = ", STARS_aPP.ballSpeed)

    def difficulty_slider(slider_value):
        STARS_aPP.difficulty = str(slider_value)
        print("diff slider value = ", STARS_aPP.difficulty)
        
    def start_command(ballSpeed,difficulty,drillType):
        print("This is the START command")
#        print(STARS_aPP.ballSpeed)
#        print(STARS_aPP.difficulty)
#        print(STARS_aPP.drillType)
        main.startMainFile(STARS_aPP.ballSpeed, STARS_aPP.difficulty, STARS_aPP.drillType)
           
    def staticDrill():
        second_message.value = "Static Passing Selected"
        STARS_aPP.drillType = "static"
        
        window1 = Window(app, bg="darkgreen")

        welcome_message2 = Text(window1, "Static Passing Parameters", size=25, font="Times New Roman", color="black")

        third_message1 = Text(window1, "Please select a passing speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window1, command=STARS_aPP.ball_speed_slider, start=1, end=5)
        speed.width = 300
        
       
        final_message1 = Text(window1, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
        difficulty = Slider(window1, command=STARS_aPP.difficulty_slider, start=1, end=5)
        difficulty.width = 300
                
        start_1 = PushButton(window1, command=STARS_aPP.start_command, args = [STARS_aPP.ballSpeed , STARS_aPP.difficulty, STARS_aPP.drillType], text="Begin")
        stop_1 = PushButton(window1, command=STARS_aPP.stop_command, text="stop")
        start_1.width = 30
        start_1.bg = "gray"
        stop_1.width = 30
        stop_1.bg = "gray"
        
    def predictiveDrill():
        second_message.value = "Entering Predictive Passing Mode"
        STARS_aPP.drillType = "dynamic"
        window2 = Window(app, bg="darkgreen")

        welcome_message2 = Text(window2, "Predictive Passing Controls", size=25, font="Times New Roman", color="black")

        third_message2 = Text(window2, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window2, command=STARS_aPP.ball_speed_slider, start=1, end=5)
        speed.width = 300
        
        final_message1 = Text(window2, "Please select a difficulty:", size=15, font="Times New Roman", color="black")
        difficulty = Slider(window2, command=STARS_aPP.difficulty_slider, start=1, end=5)
        difficulty.width = 300
        
        args = 1
        
        start_2 = PushButton(window2, command=STARS_aPP.start_command, args = [STARS_aPP.ballSpeed , STARS_aPP.difficulty, STARS_aPP.drillType], text="Begin")
        stop_2 = PushButton(window2, command=STARS_aPP.stop_command, text="stop")
        start_2.width = 30
        start_2.bg = "gray"
        stop_2.width = 30
        stop_2.bg = "gray"
          
    def manualDrill():
        second_message.value = "Entering Manual Mode"
        STARS_aPP.drillType = "manual"

        window3 = Window(app, bg="darkgreen")

        welcome_message3 = Text(window3, "Manual Mode Controls", size=25, font="Times New Roman", color="black")

        third_message3 = Text(window3, "Please select a ball speed:", size=15, font="Times New Roman", color="black")
        speed = Slider(window3, command=STARS_aPP.ball_speed_slider, start=1, end=5)
        speed.width = 300
                
        start_3 = PushButton(window3, command=STARS_aPP.start_command, args = [STARS_aPP.ballSpeed , STARS_aPP.difficulty, STARS_aPP.drillType], text="Begin")
        stop_3 = PushButton(window3, command=STARS_aPP.stop_command, text="stop")
        start_3.width = 30
        start_3.bg = "gray"
        stop_3.width = 30
        stop_3.bg = "gray"
    
##_____Code that gets run:__________##
#help(STEREOMAINFILE_Server)
app = App(title="S.T.A.R.S. User Interface")

welcome_message = Text(app, "Welcome to S.T.A.R.S.", size=20, font="Times New Roman", color="red")
second_message = Text(app, "Let's play some soccer!", size=13, font="Times New Roman", color="darkgreen")
logo = Picture(app, image="logo.gif", align = "left")
logo.resize(200,200)

drill_1 = PushButton(app, command=STARS_aPP.staticDrill, text="Static Passing")
drill_1.width = 30
drill_2 = PushButton(app, command=STARS_aPP.predictiveDrill, text="Predictive Passing")
drill_2.width = 30
drill_3 = PushButton(app, command=STARS_aPP.manualDrill, text="Manual Mode")
drill_3.width = 30

app.display()


