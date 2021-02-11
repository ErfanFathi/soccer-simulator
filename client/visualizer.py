from tkinter import *
import math
from cfg import *
from client import Provider
import time

# a subclass of Canvas for dealing with resizing of windows
class ResizingCanvas(Canvas):
    def __init__(self,parent,**kwargs):
        Canvas.__init__(self,parent,**kwargs)
        self.bind("<Configure>", self.on_resize)
        self.height = self.winfo_reqheight()
        self.width = self.winfo_reqwidth()

    def on_resize(self,event):
        # determine the ratio of old width/height to new width/height
        self.wscale = float(event.width)/self.width
        self.hscale = float(event.height)/self.height
        self.width = event.width
        self.height = event.height
        # resize the canvas 
        self.config(width=self.width, height=self.height)
        # rescale all the objects tagged with the "all" tag
        self.scale("all",0,0,self.wscale,self.hscale)

class visualizer:
    def __init__(self):

        self.obj = []
        self.prov = Provider()

        root = Tk()
        myframe = Frame(root)
        myframe.master.title("soccer-visualizer")
        myframe.pack(fill=BOTH, expand=YES)
        self.mycanvas = ResizingCanvas(myframe,width=670, height=520, bg="green", highlightthickness=0)
        self.mycanvas.pack(fill=BOTH, expand=YES)

        # Field Boundaries
        self.mycanvas.create_line(35, 35, 635, 35, fill="white")
        self.mycanvas.create_line(635, 35, 635, 485, fill="white")
        self.mycanvas.create_line(635, 485, 35, 485, fill="white")
        self.mycanvas.create_line(35, 485, 35, 35, fill="white")

        # Center Lines
        self.mycanvas.create_line(35, 260, 635, 260, fill="white")
        self.mycanvas.create_line(335, 35, 335, 485, fill="white")

        # Left Penalty
        self.mycanvas.create_line(35, 200, 95, 200, fill="white")
        self.mycanvas.create_line(95, 200, 95, 320, fill="white")
        self.mycanvas.create_line(95, 320, 35, 320, fill="white")

        # Right Penalty
        self.mycanvas.create_line(635, 200, 575, 200, fill="white")
        self.mycanvas.create_line(575, 200, 575, 320, fill="white")
        self.mycanvas.create_line(575, 320, 635, 320, fill="white")

        # Middle Circle
        self.mycanvas.create_oval(310, 235, 360, 285, outline="white", width=1)

        # Left Goal
        self.mycanvas.create_line(35, 230, 25, 230, fill="red")
        self.mycanvas.create_line(25, 230, 25, 290, fill="red")
        self.mycanvas.create_line(25, 290, 35, 290, fill="red")

        # Right Goal
        self.mycanvas.create_line(635, 230, 645, 230, fill="red")
        self.mycanvas.create_line(645, 230, 645, 290, fill="red")
        self.mycanvas.create_line(645, 290, 635, 290, fill="red")

        # Update positions
        root.after(0, self.show)

        self.mycanvas.addtag_all("all")
        root.mainloop()


    def draw_robot(self, x, y, dire, color, num):
        # robot drawing with canvas

        #calculate mid position for show robots id
        mid_x = (((x*50+330)*self.mycanvas.width/670) + ((x*50+340)*self.mycanvas.width/670))/2
        mid_y = (((-y*50+255)*self.mycanvas.height/520) + ((-y*50+265)*self.mycanvas.height/520))/2

        # draw robot
        self.obj.append(self.mycanvas.create_arc((x*50+330)*self.mycanvas.width/670,
                                                (-y*50+255)*self.mycanvas.height/520,
                                                (x*50+340)*self.mycanvas.width/670,
                                                (-y*50+265)*self.mycanvas.height/520,
                                                start=dire+45,
                                                extent=270,
                                                outline="white",
                                                style="chord",
                                                fill=color))

        # draw text
        self.obj.append(self.mycanvas.create_text(mid_x, mid_y, text=num, fill="black"))

    def draw_ball(self, x, y):
        # draw ball
        self.obj.append(self.mycanvas.create_oval((x*50+332.5)*self.mycanvas.width/670,
                                            (-y*50+257.5)*self.mycanvas.height/520,
                                            (x*50+337.5)*self.mycanvas.width/670,
                                            (-y*50+262.5)*self.mycanvas.height/520,
                                            outline="purple",
                                            fill="orange",
                                            width=3))

    def show_frame_number(self, fn):
        # show frame number every frame
        self.obj.append(self.mycanvas.create_text(70*self.mycanvas.width/670, 25*self.mycanvas.height/670, text="Frame : "+str(fn), fill="black"))

    def delete_objects(self):
        # delete robots and ball from canvas
        [self.mycanvas.delete(x) for x in self.obj]
        self.obj.clear()

    def show(self):
        # Get data from simulator and show after that
        while True:
            pack, no = self.sample_packet()

            self.prov.send(pack, no, False)
            data = self.prov.receive()

            for i in range(robot_count):
                if data["robot_yellow_team_"+str(i)]:
                    color = "yellow"
                else:
                    color = "blue"
                
                if color == "yellow":
                    num = i - int(robot_count/2)
                else:
                    num = i
                self.draw_robot(data["robot_x_"+str(i)], 
                                data["robot_y_"+str(i)],
                                data["robot_orientation_"+str(i)]*180/math.pi,
                                color,
                                str(num))
            
            self.draw_ball(data["ball_x"], data["ball_y"])

            self.show_frame_number(data["frame_number"])

            self.mycanvas.update()

            time.sleep(0.016666667)

            self.delete_objects()

    def sample_packet(self):
        # this is sample packet for send to the soccer-simulator
        commands = {}
        commands['reset_frame_number'] = False
        commands['robot_id_0'] = 0
        commands['robot_yellow_team_0'] = False
        commands['robot_vx_0'] = 0
        commands['robot_vy_0'] = 0
        commands['robot_vw_0'] = 0
        commands['robot_kickspeedx_0'] = False
        commands['robot_kickspeedz_0'] = False
        commands['robot_spinner_0'] = False

        # commands['robot_id_1'] = 1
        # commands['robot_yellow_team_1'] = False
        # commands['robot_vx_1'] = 0.01
        # commands['robot_vy_1'] = 0.01
        # commands['robot_vw_1'] = 0.001
        # commands['robot_kickspeedx_1'] = False
        # commands['robot_kickspeedz_1'] = False
        # commands['robot_spinner_1'] = False

        no = 1 # command for 1 robot

        return commands, no

def main():
    print("Soccer-Visualizer")
    visualizer()

if __name__ == "__main__":
    main()