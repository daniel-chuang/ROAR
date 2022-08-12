with open("major_center_waypoints_1LAPONLY_pitch_mod_optimize.txt", "r") as f:
    filelines = f.readlines()
    data = filelines[0].split(",")
    data[4] = "1.234567890"
    s = ','.join(data)
    print(s)
'''
with open("major_center_waypoints_1LAPONLY_pitch_mod_optimize.txt", "w") as f:
    f.writelines(filelines)
'''