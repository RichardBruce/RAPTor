#!/usr/bin/python

'''
Created on Nov 13, 2009

@author: erikolsson
'''

import sys
import os
import math

a             = {}
a["rot_x"]    = 0.0
a["rot_y"]    = 0.0
a["width"]    = 320
a["height"]   = 240
cam_r         = 0.0, 0.0, 0.0
lights        = []
start_frame   = 0
end_frame     = -1
fps_render    = 60
fps_video     = 60
work_dir      = os.getcwd()
output_dir    = work_dir + "/output"
cam_mode      = "both"
anti_aliasing = ""
remove_pics_after_merge = False
#anti_aliasing = "-anti_alias 2 2 "
out_vid_name  = "output.avi"
skip_render   = False
silent        = False
merge         = True
if cam_mode == "both":
    left_cam  = "OUTPUT_DIR/%d_left_0"
    right_cam = "OUTPUT_DIR/%d_right_0"
else:
    left_cam  = "OUTPUT_DIR/%d"
    right_cam = "OUTPUT_DIR/%d"

for i in range(0, len(sys.argv)):
    arg = sys.argv[i]
    if arg == "--model_type":
        a["model_type"] = sys.argv[i+1]
        i = i + 1
    elif arg == "--model_path":
        a["model_path"] = sys.argv[i+1]
        i = i + 1
    elif arg == "--output_dir":
        output_dir = sys.argv[i+1]
        i = i + 1
    elif arg == "--cam_right":
        cam_r = float(sys.argv[i+1]), float(sys.argv[i+2]), float(sys.argv[i+3])
        a["cam_right"]  = sys.argv[i+1] + " " + sys.argv[i+2] + " " + sys.argv[i+3]
        i = i + 3
    elif arg == "--cam_up":
        a["cam_up"]  = sys.argv[i+1] + " " + sys.argv[i+2] + " " + sys.argv[i+3]
        i = i + 3
    elif arg == "--cam_fwd":
        a["cam_fwd"]  = sys.argv[i+1] + " " + sys.argv[i+2] + " " + sys.argv[i+3]
        i = i + 3
    elif arg == "--light":
        tmp_str = ""
        for j in range(0, 8):
            tmp_str = tmp_str + sys.argv[i+1+j] + " "
        lights = lights + [tmp_str]
        i = i + 8
    elif arg == "--waypoints":
        waypoint_file = open(sys.argv[i+1])
        i = i + 1
    elif arg == "--fps":
        fps_render = float(sys.argv[i+1])
        fps_video  = fps_render
        i = i + 1
    elif arg == "--fps_render":
        fps_render = float(sys.argv[i+1])
        i = i+1
    elif arg == "--fps_video":
        fps_video = float(sys.argv[i+1])
        i = i+1
    elif arg == "--x_res":
        a["width"] = int(sys.argv[i+1])
        i = i + 1
    elif arg == "--y_res":
        a["height"] = int(sys.argv[i+1])
        i = i + 1
    elif arg == "--start_at":
        start_frame = int(sys.argv[i+1])
        i = i + 1
    elif arg == "--silent":
        silent = True
    elif arg == "--end_at":
        end_frame = int(sys.argv[i+1])
        i = i + 1
    elif arg == "--cam_mode":
        cam_mode = sys.argv[i+1]
        i = i + 1
    elif arg == "--left_cam":
        left_cam = sys.argv[i+1]
        i = i + 1
    elif arg == "--right_cam":
        right_cam = sys.argv[i+1]
        i = i + 1
    elif arg == "--out_vid_name":
        out_vid_name = sys.argv[i+1]
        i = i+1
    elif arg == "--skip_render":
        skip_render = True;
    elif arg == "--dont_merge":
        merge = False
    elif arg == "--remove_pics_after_merge":
        remove_pics_after_merge = True
    elif arg == "--help" or len(sys.argv)==1:
        print("Usage: rt_anim.py [options]\n")
        print("Please refer to the README file in the 'animation' folder for a detailed")
        print("description of the options. You may also refer to 'animation/anim/sponza/sponza_args'")
        print("for an example of option formats.\n")
        print("To run rt_anim.py with arguments from e.g. 'anim/sponza/sponza_args':")
        print("    cd anim/sponza")
        print("    ../../src/rt_anim.py `cat sponza_args`\n")
        print("Waypoints can be edited as illustrated in the file 'anim/sponza/sponza_waypoints'.\n")
        sys.exit(0)

left_cam = left_cam.replace("OUTPUT_DIR", output_dir)
right_cam = right_cam.replace("OUTPUT_DIR", output_dir)

if not skip_render:
    for i in range(0, len(lights)):
        a["light" + str(i)] = lights[i]
        if not silent:
            print("light" + str(i) + " = " + lights[i])
    
    if not silent:
        print("rendering with raytracer at: " + os.environ["RAYTRACER_HOME"])
        print("model_type  = " + a["model_type"])
        print("model_path  = " + a["model_path"])
        print("cam_right   = " + a["cam_right"])
        print("cam_up      = " + a["cam_up"])
        print("cam_fwd     = " + a["cam_fwd"])
        print("waypoints   = " + repr(waypoint_file))
        print("fps_render  = " + str(fps_render))
        print("fps_video   = " + str(fps_video))
        print("x_res       = " + str(a["width"]))
        print("y_res       = " + str(a["height"]))
        print("skip to     = " + str(start_frame))
        print("cam_mode    = " + cam_mode)

    try:
        os.mkdir(output_dir)
    except:
        if not silent:
            print "Warning: directory '" + output_dir + "' already exists. Overwriting contents in directory..."

total_num_rendered_frames = 0

if not skip_render:
    waypoints = waypoint_file.readlines()
    waypoints_done = False

    for j in range(1, len(waypoints)):
        if waypoints_done:
            break
        if waypoints[j-1][0] == "#" or waypoints[j][0] == "#":
            continue

        beg_wp = waypoints[j-1].split()
        end_wp = waypoints[j  ].split()

        num_seconds    = float(beg_wp[3])
        num_frames     = int(fps_render*num_seconds) # number of frames between two waypoints
        focus_dist     = float(beg_wp[4])
        eye_separation = float(beg_wp[5])
        eye_angle      = (math.atan(eye_separation/focus_dist)/(2*math.pi))*360

        start_l = [float(beg_wp[0]),
                   float(beg_wp[1]),
                   float(beg_wp[2])]
        start_r = [start_l[0] + float(cam_r[0])*eye_separation,
                   start_l[1] + float(cam_r[1])*eye_separation,
                   start_l[2] + float(cam_r[2])*eye_separation]
        end_l   = [float(end_wp[0]),
                   float(end_wp[1]),
                   float(end_wp[2])]
        end_r   = [end_l[0] + float(cam_r[0])*eye_separation,
                   end_l[1] + float(cam_r[1])*eye_separation,
                   end_l[2] + float(cam_r[2])*eye_separation] 

        if not silent:
            print("Rendering " + str(num_frames) + " frames between waypoints:")
            print("   " + repr(start_l) + " --> " + repr(end_l))
            print("   (eye_angle = " + str(eye_angle) + " degrees)")

        dist_l  = [end_l[0] - start_l[0], end_l[1] - start_l[1], end_l[2] - start_l[2]]
        dist_r  = [end_r[0] - start_r[0], end_r[1] - start_r[1], end_r[2] - start_r[2]]

        os.chdir(os.environ["RAYTRACER_HOME"])

        for i in range(0, num_frames):
            if total_num_rendered_frames==end_frame:
                waypoints_done = True   
                break
            if total_num_rendered_frames < start_frame:
                total_num_rendered_frames = total_num_rendered_frames + 1
                continue

            snap_left = ""
            snap_right = ""

            if cam_mode in ["left", "both"]:
                snap_left     = left_cam.replace("%d", str(total_num_rendered_frames))
                a["rot_y"]    = eye_angle 
                a["cam_pos"]  = str(start_l[0]+i*dist_l[0]/num_frames) + " " + str(start_l[1]+i*dist_l[1]/num_frames) + " " + str(start_l[2]+i*dist_l[2]/num_frames)
                cmd = "raytracer -" + a["model_type"] + " " + a["model_path"] + " " + \
                      "-cam " + a["cam_pos"] + " " + \
                      "-jpg " + snap_left + " 100 " + \
                      "-dx " + a["cam_right"] + " -dy " + a["cam_up"] + " -dz " + a["cam_fwd"] + " " + \
                      anti_aliasing + " " \
                      "-res " + str(a["width"]) + " " + str(a["height"]) + " " + \
                      "-rx " + str(a["rot_x"]) + " " + \
                      "-ry " + str(a["rot_y"])
                for l in range(0, len(lights)):
                    cmd = cmd + " -light " + a["light" + str(l)]
                os.system(cmd)

            if cam_mode in ["right", "both"]:
                snap_right    = right_cam.replace("%d", str(total_num_rendered_frames))
                a["rot_y"]    = -eye_angle
                a["cam_pos"]  = str(start_r[0]+i*dist_r[0]/num_frames) + " " + str(start_r[1]+i*dist_r[1]/num_frames) + " " + str(start_r[2]+i*dist_r[2]/num_frames)
                cmd = "raytracer -" + a["model_type"] + " " + a["model_path"] + " " + \
                      "-cam " + a["cam_pos"] + " " + \
                      "-jpg " + snap_right + " 100 " + \
                      "-dx " + a["cam_right"] + " -dy " + a["cam_up"] + " -dz " + a["cam_fwd"] + " " + \
                      anti_aliasing + " " + \
                      "-res " + str(a["width"]) + " " + str(a["height"]) + " " + \
                      "-rx " + str(a["rot_x"]) + " " + \
                      "-ry " + str(a["rot_y"])
                for l in range(0, len(lights)):
                    cmd = cmd + " -light " + a["light" + str(l)]
                os.system(cmd)

            total_num_rendered_frames = total_num_rendered_frames + 1

    # append the suffix that the raytracer adds to the snapshots
    left_cam = left_cam + "_0.jpg"
    right_cam = right_cam + "_0.jpg"
else:
    left_cam = left_cam + ".jpg"
    right_cam = right_cam + ".jpg"
    if not silent:
        print("Skipping render.")

if cam_mode == "both":
    i = 0
    while (i < end_frame) or (end_frame < 0):
        left_cam_file         = left_cam.replace("%d", str(i)) 
        right_cam_file        = right_cam.replace("%d", str(i))
        left_cam_file_exists  = os.path.isfile(left_cam_file)
        right_cam_file_exists = os.path.isfile(right_cam_file)
        merged_file           = output_dir + "/" + str(i) + ".jpg"
        merged_file_exists    = os.path.isfile(merged_file)

        if (not left_cam_file_exists) or (not right_cam_file_exists):
            if not silent:
                print(left_cam_file + " exists: " + str(left_cam_file_exists))
                print(right_cam_file + " exists: " + str(right_cam_file_exists))
            break
      
        if merge or not merged_file_exists:
            if not silent:
                print("Merging pictures for 3D video frame #" + str(i))
                print("convert " + left_cam_file + " " + right_cam_file + " +append -quality 100 '" + merged_file + "'")
            os.system("convert " + left_cam_file + " " + right_cam_file + " +append -quality 100 '" + merged_file + "'")
        if remove_pics_after_merge:
            os.system("rm -f "   + left_cam_file)
            os.system("rm -f "   + right_cam_file)

        i = i + 1

if cam_mode == "left":
    snapshot_filename = left_cam 
elif cam_mode == "right":
    snapshot_filename = right_cam 
elif cam_mode == "both":
    snapshot_filename = output_dir + "/%d.jpg" 

os.system("$RAPTOR_LIBS/ffmpeg/ffmpeg -g " + str(fps_video) + " -flags +bitexact -flags2 +wpred -qscale 1 -i " + snapshot_filename + " -y " + out_vid_name) 
