import cv2
import numpy as np
import open3d as o3d
from o3d_utils import create_segment, create_grid
import tkinter



# LINES_BODY is used when drawing the skeleton onto the source image. 
# Each variable is a list of continuous lines.
# Each line is a list of keypoints as defined at https://google.github.io/mediapipe/solutions/pose.html#pose-landmark-model-blazepose-ghum-3d
#LINES_BODY = [[28,30,32,28,26,24,12,11,23,25,27,29,31,27],
#                [23,24],
#                [22,16,18,20,16,14,12],
#                [21,15,17,19,15,13,11],
#                [8,6,5,4,0,1,2,3,7],
#                [10,9],
#                ]
LINES_BODY = [#[28,30,32,28,26,24,12,11,23,25,27,29,31,27],
                #[24,12],#torso
                #[23,11],#torso
                #[23,24],#torso
                #[23,24],
                [11,12],
                [20,22],
                [21,19],
                [22,18,20],#right hand
                [18,16,22],#right hand
                [16,14,12],#right arm
                [21,17,19],#left hand
                [21,15,17],#left hand
                [15,13,11],#left arm
                #[8,6,5,4,0,1,2,3,7],
                #[10,9],
                ]

# LINE_MESH_BODY and COLORS_BODY are used when drawing the skeleton in 3D. 
rgb = {"right":(0,1,0), "left":(1,0,0), "middle":(1,1,0)}
LINE_MESH_BODY = [[9,10],[4,6],[1,3],
                    [12,14],[14,16],[16,20],[20,18],[18,16],
                    [12,11],[11,23],[23,24],[24,12],
                    [11,13],[13,15],[15,19],[19,17],[17,15],
                    [24,26],[26,28],[32,30],
                    [23,25],[25,27],[29,31]]


COLORS_BODY = ["middle","right","left",
                    "right","right","right","right","right",
                    "middle","middle","middle","middle",
                    "left","left","left","left","left",
                    "right","right","right","left","left","left"]
COLORS_BODY = [rgb[x] for x in COLORS_BODY]


SELECTED_LANDMARKS = [11,12,13,14,15,16,17,18,19,20,21,22]


class BlazeposeRenderer:
    def __init__(self,
                pose,
                show_3d=False,
                output=None):
        self.pose = pose
        self.show_3d = show_3d

        # Rendering flags
        self.show_rot_rect = False
        self.show_mirror = False
        self.show_landmarks = True
        self.show_score = False
        self.show_fps = False
        cv2.namedWindow("corpusLabe", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("corpusLabe", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.screen_w = tkinter.Tk().winfo_screenwidth()
        self.screen_h = tkinter.Tk().winfo_screenheight()


        if self.show_3d:

            self.vis3d = o3d.visualization.Visualizer()
            self.vis3d.create_window() 
            opt = self.vis3d.get_render_option()
            opt.background_color = np.asarray([0, 0, 0])
            z = min(pose.img_h, pose.img_w)/3
            self.grid_floor = create_grid([0,pose.img_h,-z],[pose.img_w,pose.img_h,-z],[pose.img_w,pose.img_h,z],[0,pose.img_h,z],5,2, color=(1,1,1))
            self.grid_wall = create_grid([0,0,z],[pose.img_w,0,z],[pose.img_w,pose.img_h,z],[0,pose.img_h,z],5,2, color=(1,1,1))
            self.vis3d.add_geometry(self.grid_floor)
            self.vis3d.add_geometry(self.grid_wall)
            view_control = self.vis3d.get_view_control()
            view_control.set_up(np.array([0,-1,0]))
            view_control.set_front(np.array([0,0,-1]))

        if output is None:
            self.output = None
        else:
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self.output = cv2.VideoWriter(output,fourcc,pose.video_fps,(pose.img_w, pose.img_h)) 

    def draw_landmarks(self, body):
        if self.show_rot_rect:
            if self.show_mirror:
                cv2.polylines(self.frame, [np.array(body.rect_points)], True, (0,255,255), 2, cv2.LINE_AA)
            else:
                cv2.polylines(self.fake_frame, [np.array(body.rect_points)], True, (0, 255, 255), 2, cv2.LINE_AA)
        if self.show_landmarks:
            list_connections = LINES_BODY
            lines = [np.array([body.landmarks[point,:2] for point in line]) for line in list_connections]
            # lines = [np.array([body.landmarks_padded[point,:2] for point in line]) for line in list_connections]
            if self.show_mirror:
                cv2.polylines(self.frame, lines, False, (211, 211, 211), 5, cv2.LINE_AA)
                #(255, 180, 90)
            else:
                cv2.polylines(self.fake_frame, lines, False, (211, 211, 211), 5, cv2.LINE_AA)
                #(255, 180, 90)
            
            # for i,x_y in enumerate(body.landmarks_padded[:,:2]):
            for i,x_y in enumerate(body.landmarks[:self.pose.nb_kps,:2]):
                if i in SELECTED_LANDMARKS:
                    if i > 14:
                        color = (0,128,0) if i%2==0 else (211,0,0)
                    elif i == 0:
                        color = (0,255,255)
                    elif i in [4,5,6,8,10]:
                        color = (0,255,0)
                    else:
                        color = (211,211,211)
                    if self.show_mirror:
                        cv2.circle(self.frame, (x_y[0], x_y[1]), 4, color, -11)
                    else:
                        cv2.circle(self.fake_frame, (x_y[0], x_y[1]), 4, color, -11)

            if self.show_3d:
                self.vis3d.clear_geometries()
                self.vis3d.add_geometry(self.grid_floor, reset_bounding_box=False)
                self.vis3d.add_geometry(self.grid_wall, reset_bounding_box=False)
                points = body.landmarks
                lines = LINE_MESH_BODY
                colors = COLORS_BODY
                for i,a_b in enumerate(lines):
                    a, b = a_b
                    #print(points[a])
                    line = create_segment(points[a], points[b], radius=5, color=colors[i])
                    if line: self.vis3d.add_geometry(line, reset_bounding_box=False)
                self.vis3d.poll_events()
                self.vis3d.update_renderer()
                

        if self.show_score:
            h, w = self.frame.shape[:2]
            cv2.putText(self.frame, f"Landmark score: {body.lm_score:.2f}", 
                        (20, h-60), 
                        cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)


    def draw(self, frame, body):
        self.frame = frame
        h, w = self.frame.shape[:2]
        self.fake_frame = np.ones((h, w, 3), np.uint8) * 168
        if body:
            self.draw_landmarks(body)

        if self.show_mirror:
            return self.frame
        else:
            return self.fake_frame
    
    def exit(self):
        if self.output:
            self.output.release()

    def waitKey(self, inFrame, delay=1):
        if self.show_fps:
            self.pose.fps.draw(inFrame, orig=(50,50), size=1, color=(240,180,100))
        
        dim = (self.screen_w, self.screen_h)
  
        # resize image
        if (False):
            resized = cv2.resize(inFrame, dim, interpolation = cv2.INTER_AREA)
            cv2.imshow("corpusLabe", cv2.flip(resized, 1))
        else:
            cv2.imshow("corpusLabe", cv2.flip(inFrame, 1))

        if self.output:
            self.output.write(self.frame)
        key = cv2.waitKey(delay) 
        if key == 32:
            # Pause on space bar
            cv2.waitKey(0)
        elif key == ord('b'):
            self.show_rot_rect = not self.show_rot_rect
        elif key == ord('l'):
            self.show_landmarks = not self.show_landmarks
        elif key == ord('p'):
            self.show_score = not self.show_score
        elif key == ord('f'):
            self.show_fps = not self.show_fps
        elif key == ord('m'):
            self.show_mirror = not self.show_mirror
        return key
        
            
