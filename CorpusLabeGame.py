import cv2
import numpy as np
from sympy import Point3D, Plane #Line3D
import csv
from os import listdir
from os.path import isfile, join


class CorpusLabe:
    def __init__(self, pose):
        self.pose = pose
        self.flashing_lights = True
        self.current_exercise = 0
        self.shoulder_indexes = [11, 12]
        self.target_angle_f = [30, 30]
        self.target_angle_i = [160, 160]
        self.hips_indexes = [23, 24]
        self.hand_indexes = [21, 22]
        self.elbow_indexes = [13, 14]
        self.wrist_indexes = [15, 16]
        self.circle_angles = [[270, 440], [100, 270]]
        self.circle_colors = [(211, 0, 0), (0, 128, 0)]
        self.angle_adjustment = [450, 90]
        self.arm_length = 0
        self.current_target_pos = (0, 0)
        self.show_angle = True
        self.play_game = False
        self.current_angle = [0, 0]
        self.current_max_angle = [0, 0]
        self.exercise_status = 0
        self.exercise_ref = 0
        self.exercise_counter = [0, 0]
        self.exercise_goal = [5, 5]
        self.gray_color = (199, 199, 199)
        self.goal_reached = 0
        self.pos_side_vis = [[-1, -1], [-1, -1]] #todo
        self.show_help = False
        self.predefined_folder = './testFolder/'

    def save_data(self):
        # asses current max values
        # current_max_angle
        # get current user id
        current_user_id = self.get_current_id()
        # write stuff
        out_name = str(current_user_id) + '.csv'

        with open(self.predefined_folder + out_name, mode='w') as csvfile:
            csvfile = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            new_line_out = [str(self.current_max_angle[0]), str(self.current_max_angle[1])]
            csvfile.writerow(new_line_out)

    def get_current_id(self):
        # check folder exists

        # explore predefined_folder
        onlyfiles = [f for f in listdir(self.predefined_folder) if isfile(join(self.predefined_folder, f))]
        if len(onlyfiles) == 0:
            current_user_id = 0
            return current_user_id
        else:
            x = (onlyfiles[-1]).split(".", 1)
            return int(x[0]) + 1
        # determine max user id
        # assign current_user_id

# output file
    def draw_help(self):
        #h, w = self.frame.shape[:2]
        self.frame = cv2.flip(self.frame, 1)
        delta_X = 150
        cv2.putText(self.frame, "help menu",
                    (delta_X, 40),
                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
        cv2.putText(self.frame, "h - show/hide help menu",
                    (delta_X, 90),
                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
        cv2.putText(self.frame, "g - alternates between game and angle assesment app",
                    (delta_X, 140),
                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
        cv2.putText(self.frame, "r - reset maximum angle values stored",
                    (delta_X, 190),
                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
        cv2.putText(self.frame, "s - save data",
                    (delta_X, 240),
                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
        cv2.putText(self.frame, "q - quit",
                    (delta_X, 290),
                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
        self.frame = cv2.flip(self.frame, 1)
        # g - alternates between game and angle assesment app
        # q - quit
        # z - reset maximum angle values stored
        # s - save data



    def draw(self, frame, body):
        self.frame = frame
        if body:
            #self.current_exercise = 0

            control_check = self.check_if_too_close(body)

            if control_check == -1:
                if self.play_game:
                    self.draw_arena(body)
                    self.draw_play(body)
                    self.draw_target(body)
                    self.check_target(body)
                    if self.goal_reached == 0:
                        self.draw_score_counter()
                    else:
                        self.frame = cv2.flip(self.frame, 1)
                        h, w = self.frame.shape[:2]
                        cv2.putText(self.frame, "exercise goal reached",
                                    (int(w / 2) - 220, 40),
                                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                        cv2.putText(self.frame, "well done",
                                    (int(w / 2) - 120, 80),
                                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                        cv2.putText(self.frame, "[FREE PRACTICE]",
                                    (int(w / 2) - 170, 120),
                                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                        self.frame = cv2.flip(self.frame, 1)
                else:
                    self.current_angle[self.current_exercise] = self.get_current_angle(body)
                    if self.current_angle[self.current_exercise] > self.current_max_angle[self.current_exercise]:
                        self.current_max_angle[self.current_exercise] = self.current_angle[self.current_exercise]

                if (self.show_angle):
                    self.write_angle()
                    self.draw_arena(body)
                    self.current_exercise = abs(self.current_exercise - 1)
                    self.write_angle()
                    self.draw_arena(body)
                    #self.draw_arm_selected() #todo v0.x
                if self.show_help:
                    self.draw_help()
            elif control_check == 2:
                self.frame = cv2.flip(self.frame, 1)
                h, w = self.frame.shape[:2]
                cv2.putText(self.frame, "move a bit further back",
                            (int(w / 2) - 120, int(h / 2)),
                            cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                self.frame = cv2.flip(self.frame, 1)
            elif control_check == 3:
                self.frame = cv2.flip(self.frame, 1)
                h, w = self.frame.shape[:2]
                cv2.putText(self.frame, "move a bit to your right",
                            (int(w / 2) - 120, int(h / 2)),
                            cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                self.frame = cv2.flip(self.frame, 1)
            elif control_check == 4:
                self.frame = cv2.flip(self.frame, 1)
                h, w = self.frame.shape[:2]
                cv2.putText(self.frame, "move a bit to your left",
                            (int(w / 2) - 120, int(h / 2)),
                            cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                self.frame = cv2.flip(self.frame, 1)

            #self.current_exercise = 1
            #self.draw_arena(body)
            #self.draw_play(body)
            #self.draw_target(body)
        else:
            if self.play_game:
                self.frame = cv2.flip(self.frame, 1)
                h, w = self.frame.shape[:2]
                cv2.putText(self.frame, "ready to move?",
                            (int(w/2)-120, int(h/2)),
                            cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                cv2.putText(self.frame, "stand in front and try this",
                            (int(w/2)-220, int(h/2)+100),
                            cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)
                self.frame = cv2.flip(self.frame, 1)
                self.goal_reached = 0
                self.exercise_counter = [0, 0]
        return self.frame

    def draw_score_counter(self):
        h, w = self.frame.shape[:2]

        score_delta = [(20, 50), (w-100, 50)]



        self.frame = cv2.flip(self.frame, 1)

        cv2.putText(self.frame, f"{self.exercise_counter[self.current_exercise]}""/"f"{self.exercise_goal[self.current_exercise]}",
                    score_delta[self.current_exercise],
                    cv2.FONT_HERSHEY_PLAIN, 2, self.circle_colors[self.current_exercise], 2)

        cv2.putText(self.frame,
                    f"{self.exercise_counter[abs(self.current_exercise - 1)]}""/"f"{self.exercise_goal[abs(self.current_exercise - 1)]}",
                    score_delta[abs(self.current_exercise - 1)],
                    cv2.FONT_HERSHEY_PLAIN, 2, self.gray_color, 2)


        self.frame = cv2.flip(self.frame, 1)


    def write_angle(self):

        self.frame = cv2.flip(self.frame, 1)
        h, w = self.frame.shape[:2]
        if self.current_exercise == 0:
            cv2.putText(self.frame, f"{self.current_max_angle[self.current_exercise]:.2f}",
                        (20, h - 60),
                        cv2.FONT_HERSHEY_PLAIN, 3, self.circle_colors[self.current_exercise], 2)
        else:
            cv2.putText(self.frame, f"{self.current_max_angle[self.current_exercise]:.2f}",
                        (w-200, h - 60),
                        cv2.FONT_HERSHEY_PLAIN, 3, self.circle_colors[self.current_exercise], 2)
        self.frame = cv2.flip(self.frame, 1)

    def check_point_in_image(self, point):
        h, w = self.frame.shape[:2]
        if (0 < point[0] < w) & (0 < point[1] < h):
            return True
        else:
            return False

    def update_counter(self):
        self.exercise_counter[self.current_exercise] = self.exercise_counter[self.current_exercise] + 1
        if (self.exercise_counter[self.current_exercise] == self.exercise_goal[self.current_exercise]) & (self.exercise_counter[abs(self.current_exercise - 1)] == self.exercise_goal[abs(self.current_exercise - 1)]):
            self.exercise_counter[self.current_exercise] = 0
            self.exercise_counter[abs(self.current_exercise - 1)] = 0
            self.goal_reached = 1

    def check_target(self, body):
        pt1 = body.landmarks[:self.pose.nb_kps, :2][self.hand_indexes[self.current_exercise]]
        pt2 = self.current_target_pos

        if (self.check_point_in_image(pt1)) & (self.check_point_in_image(pt2)):

            length = ((((pt2[0] - pt1[0]) ** 2) + ((pt2[1] - pt1[1]) ** 2)) ** 0.5)

            if length < 25:
                if self.exercise_ref == 0:
                    self.exercise_ref = 1
                    #self.draw_target(body)
                else:
                    if self.goal_reached == 0:
                        self.update_counter()
                    self.current_exercise = abs(self.current_exercise - 1)
                    self.exercise_ref = 0
                    #self.draw_target(body)


    def check_if_too_close(self, body):

        outcode = -1
        if True:

            # 2 - too close
            # 3 - too much to the left
            # 4 - too much to the right

            h, w = self.frame.shape[:2]

            # left
            t11 = 180
            t12 = 0
            p1 = body.landmarks[:self.pose.nb_kps, :2][self.shoulder_indexes[0]]
            # right
            t21 = 180
            t22 = 0
            p2 = body.landmarks[:self.pose.nb_kps, :2][self.shoulder_indexes[0]]

            self.arm_length = int(self.get_arm_length(body))
            l = self.arm_length


            # bottom left

            tr11 = np.radians(t11)
            a11 = np.sin(tr11) * l
            b11 = np.cos(tr11) * l
            t_x11 = int(p1[0] + a11)
            t_y11 = int(p2[1] - b11)

            # top left

            tr12 = np.radians(t12)
            a12 = np.sin(tr12) * l
            b12 = np.cos(tr12) * l
            t_x12 = int(p1[0] + a12)
            t_y12 = int(p2[1] - b12)

            # bottom right

            tr21 = np.radians(t21)
            a21 = np.sin(tr21) * l
            b21 = np.cos(tr21) * l
            t_x21 = int(p2[0] - a21)
            t_y21 = int(p2[1] - b21)

            # top right

            tr22 = np.radians(t22)
            a22 = np.sin(tr22) * l
            b22 = np.cos(tr22) * l
            t_x22 = int(p2[0] - a22)
            t_y22 = int(p2[1] - b22)


            if (t_y12 < 0) | (t_y22 < 0) | (t_y11 > h) | (t_y21 > h):
                outcode = 2

            if (t_x11 < 0) | (t_x12 < 0):
                outcode = 3

            if (t_x21 > w) | (t_x22 > w):
                outcode = 4

        return outcode






    def get_target(self, body):
        l = self.arm_length
        if self.exercise_ref == 0:
            t = self.target_angle_i[self.current_exercise]
        else:
            t = self.target_angle_f[self.current_exercise]
        p = body.landmarks[:self.pose.nb_kps, :2][self.shoulder_indexes[self.current_exercise]]


        tr = np.radians(t)
        #
        a = np.sin(tr) * l
        b = np.cos(tr) * l
        # else:

        if self.current_exercise == 0:
            t_x = int(p[0] + a)
            t_y = int(p[1] - b)
        else:
            t_x = int(p[0] - a)
            t_y = int(p[1] - b)

        self.current_target_pos = (t_x, t_y)


    def draw_target(self, body):
        self.get_target(body)
        (t_x, t_y) = self.current_target_pos
        h, w, c = self.frame.shape

        #print(w, t_x)
        #print(h, t_y)
        if (0 < t_x < w) & (0 < t_y < h):

            #print(t_x, t_y)
            #if self.flashing_lights:
            #cv2.circle(self.frame, (t_x, t_y), 10, self.circle_colors[self.current_exercise], -1)
            #self.flashing_lights = not self.flashing_lights
            #else:
            cv2.circle(self.frame, (t_x, t_y), 20, self.circle_colors[self.current_exercise], -1)
            self.flashing_lights = not self.flashing_lights
            text = ""
            delta_0 = 0
            if self.exercise_ref == 0:
                text = "START HERE"
                delta_0 = -25
            else:
                text = "CATCH ME"
            if self.current_exercise == 0:
                delta = -200 + delta_0
            else:
                delta = 30
            self.frame = cv2.flip(self.frame, 1)
            cv2.putText(self.frame, text,
                        (w-t_x+delta, t_y+10),
                        cv2.FONT_HERSHEY_PLAIN, 2, self.circle_colors[self.current_exercise], 2)
            self.frame = cv2.flip(self.frame, 1)

    def draw_arm_selected(self):
        cv2.circle(self.frame,
                   (self.pos_side_vis[self.current_exercise][0], self.pos_side_vis[self.current_exercise][1]),
                   20,
                   self.circle_colors[self.current_exercise],
                   -1)


    def draw_arena(self, body):
        radius = int(self.get_arm_length(body))
        color = self.gray_color
        thickness = 5
        center = body.landmarks[:self.pose.nb_kps, :2][self.shoulder_indexes[self.current_exercise]]
        axes = (radius, radius)
        angle = 0
        circle = self.circle_angles[self.current_exercise]
        startAngle = int(circle[0])
        endAngle = int(circle[1])
        #self.frame = (self.frame).astype(np.uint8)
        self.frame = cv2.ellipse(self.frame, (int(center[0]),int(center[1])), axes, angle, startAngle, endAngle, color, thickness)
        #return image

    def get_current_angle(self, body):
        pt1 = body.landmarks[:self.pose.nb_kps, :2][self.shoulder_indexes[self.current_exercise]]
        pt2 = body.landmarks[:self.pose.nb_kps, :2][self.elbow_indexes[self.current_exercise]]
        pt3 = np.array([pt1[0], pt1[1]+1])

        ba = pt2 - pt1
        bc = pt3 - pt1

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        return np.degrees(angle)

    def is_arm_aligned(self, body):

        p1 = body.landmarks[self.shoulder_indexes[abs(self.current_exercise - 1)]]
        p2 = body.landmarks[self.shoulder_indexes[self.current_exercise]]
        p3 = body.landmarks[self.hand_indexes[self.current_exercise]]

        pt1 = np.array([p1[0], p1[2]])
        pt2 = np.array([p2[0], p2[2]])
        pt3 = np.array([p3[0], p3[2]])

        ba = pt2 - pt1
        bc = pt3 - pt1

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        grad_angle = np.degrees(angle)
        print(grad_angle)

        if grad_angle < 9:
            return True
        else:
            return False

    def is_arm_outstretched(self, body):
        pt2 = body.landmarks[:self.pose.nb_kps, :2][self.shoulder_indexes[self.current_exercise]]
        pt3 = body.landmarks[:self.pose.nb_kps, :2][self.hand_indexes[self.current_exercise]]
        pt1 = body.landmarks[:self.pose.nb_kps, :2][self.elbow_indexes[self.current_exercise]]

        ba = pt2 - pt1
        bc = pt3 - pt1

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        if 120 < np.degrees(angle) < 190:
            return True
        else:
            return False

    def draw_play(self, body):
        c_angle = self.get_current_angle(body)
        self.current_angle[self.current_exercise] = c_angle
        #print(c_angle)

        self.arm_length = int(self.get_arm_length(body))
        color = self.circle_colors[self.current_exercise]
        thickness = 5
        center = body.landmarks[:self.pose.nb_kps, :2][self.shoulder_indexes[self.current_exercise]]
        axes = (self.arm_length, self.arm_length)
        angle = 0
        circle = self.circle_angles[self.current_exercise]

        probe_angle = 0
        if self.current_exercise == 1:
            startAngle = circle[0]
            probe_angle = self.angle_adjustment [self.current_exercise] + c_angle
            endAngle = probe_angle
        elif self.current_exercise == 0:
            probe_angle = self.angle_adjustment[self.current_exercise] - c_angle
            startAngle = probe_angle
            endAngle = circle[1]

        #& (self.is_arm_aligned(body)
        #self.is_arm_aligned(body)
        if self.is_arm_outstretched(body) & (self.circle_angles[self.current_exercise][0] < probe_angle < self.circle_angles[self.current_exercise][1]) & (self.exercise_ref == 1):
            self.frame = cv2.ellipse(self.frame, (int(center[0]),int(center[1])), axes, angle, startAngle, endAngle, color, thickness)
            self.exercise_status = 1
        else:
            self.exercise_status = 0
            self.exercise_ref = 0
        #self.draw_target(body)


    def get_arm_length(self, body):
        right_arm = [16, 14, 12]
        left_arm = [15, 13, 11]
        #[11, 12] #shoulders
        #[23, 24],  # torso

        length1 = self.get_length(body, right_arm)
        length2 = self.get_length(body, left_arm)

        # get torso hight
        pt1 = body.landmarks[:self.pose.nb_kps, :2][11]
        pt2 = body.landmarks[:self.pose.nb_kps, :2][23]

        length3 = ((((pt2[0] - pt1[0]) ** 2) + ((pt2[1] - pt1[1]) ** 2)) ** 0.5)

        return max(length1, length2, length3)

    def get_length(self, body, arm_points):
        length = 0
        for i in range(len(arm_points)-1):
            pt1 = body.landmarks[:self.pose.nb_kps, :2][arm_points[i]]
            pt2 = body.landmarks[:self.pose.nb_kps, :2][arm_points[i+1]]

            length = length + ((((pt2[0] - pt1[0]) ** 2) + ((pt2[1] - pt1[1]) ** 2)) ** 0.5)
        return length

