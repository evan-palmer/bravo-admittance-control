from ast import literal_eval
import re
import matplotlib.pyplot as plt
import itertools
#!!! Important: Delete the first line "timestamp, etc"
#!!! Change [infile] below
infile = r"C:\Users\nnamd\Downloads\2023-06-05-07-29-00.log"

#Converted/Stored data
time = []
joint_position = []
joint_velocity = []
forces = []
desired_velocity = []

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Functions
def getIndexAsArray(array, index):
    output = []
    for i in array:
        output.append(i[index])
    return output

def convert(str_array):
    "Converts string formatted as array to a real array of floats"
    str_array = literal_eval(str_array)
    return [float(i) for i in str_array]

def read_lines():
    "Reads lines of text file and outputs result"
    lines = []
    with open(infile) as f:
        lines = f.readlines()
    return lines 

def combine_lines(log_lines):
    "Combines pairs of lines, outputs combined lines in array"
    combined_lines = []
    i = 0
    while i < len(log_lines):
        line1 = log_lines[i].strip()
        line2 = log_lines[i + 1].strip() if i < len(log_lines) - 1 else None
        if line1 != line2:
            combined_lines.append(line1 + ' ' + (line2 or ''))
        i += 2
    return combined_lines


def extract_data(combined_lines):
    "Takes array of strings of data, cleans and stores data"
    for line in combined_lines:
        #Remove weird 0 entries and split into arrays
        zero_line = line.replace("0. ", "0 ").replace("[ ","[")
        data = zero_line.split(',')
       
        if (len(data) != 5):
            continue
        else:
            pass

        #Clean spaces in data
        data[1] = re.sub(' +', ' ', data[1])
        data[2] = re.sub(' +', ' ', data[2])
        data[3] = re.sub(' +', ' ', data[3])
        data[4] = re.sub(' +', ' ', data[4])

        #Add commas between numbers and some corrections
        j_pos = data[1].replace(" ", ",")
        j_vel = data[2].replace(" ", ",")
        force = data[3].replace(" ", ",")
        d_vel = data[4].replace(" ", ",") 
        d_vel = d_vel.replace(",,", ",")
        force = force.replace(",,", ",")
        timestamp = float(data[0])

        #Convert to floats and append to lists
        joint_position.append(convert(j_pos))
        joint_velocity.append(convert(j_vel))
        forces.append(convert(force))
        desired_velocity.append(convert(d_vel))
        time.append(timestamp-1685975340.38633)
        
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Main

log_lines = read_lines()
combined_lines = combine_lines(log_lines)
extract_data(combined_lines)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Output/Graphs
#Forces
forces_x = getIndexAsArray(forces, 0)
forces_y = getIndexAsArray(forces,1)
forces_z = getIndexAsArray(forces,2)

#Desired Velocity
desired_v1 = getIndexAsArray(desired_velocity, 0)
desired_v2 = getIndexAsArray(desired_velocity, 1)
desired_v3 = getIndexAsArray(desired_velocity, 2)
desired_v4 = getIndexAsArray(desired_velocity, 3)
desired_v5 = getIndexAsArray(desired_velocity, 4)
desired_v6 = getIndexAsArray(desired_velocity, 5)
desired_v7 = getIndexAsArray(desired_velocity, 6)


#Measured Joint Velocity
joint_v1 = getIndexAsArray(joint_velocity, 0)
joint_v2 = getIndexAsArray(joint_velocity, 1)
joint_v3 = getIndexAsArray(joint_velocity, 2)
joint_v4 = getIndexAsArray(joint_velocity, 3)
joint_v5 = getIndexAsArray(joint_velocity, 4)
joint_v6 = getIndexAsArray(joint_velocity, 5)
joint_v7 = getIndexAsArray(joint_velocity, 6)

#Measured Joint Position
joint_pos1 = getIndexAsArray(joint_position, 0)
joint_pos2 = getIndexAsArray(joint_position, 1)
joint_pos3 = getIndexAsArray(joint_position, 2)
joint_pos4 = getIndexAsArray(joint_position, 3)
joint_pos5 = getIndexAsArray(joint_position, 4)
joint_pos6 = getIndexAsArray(joint_position, 5)
joint_pos7 = getIndexAsArray(joint_position, 6)

"""Plotting External Forces"""
plt.plot(time, forces_x, label = 'X Forces')
#plt.plot(time, forces_y, label = 'Y Forces')
#plt.plot(time, forces_z, label = 'Z Forces')
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.legend()
plt.show()

"""Plotting Joint Positions"""
#plt.plot(time,joint_pos1, label = 'Joint 1 Position')
#plt.plot(time,joint_pos2, label = 'Joint 2 Position')
#plt.plot(time,joint_pos3, label = 'Joint 3 Position')
#plt.plot(time,joint_pos4, label = 'Joint 4 Position')
#plt.plot(time,joint_pos5, label = 'Joint 5 Position')
#plt.plot(time,joint_pos6, label = 'Joint 6 Position')
#plt.plot(time,joint_pos7, label = 'Joint 7 Position')
#plt.ylabel('Radians')
#plt.xlabel('Time (s)')
#plt.legend()
#plt.show()


"""Plotting Velocities (Measured + Desired)"""

#plt.plot(time,joint_v1,label = 'Measured Joint 1 Velocity')
#plt.plot(time,joint_v2, label = 'Measured Joint 2 velocity')
plt.plot(time,joint_v3, label = 'Measured Joint 3 Velocity')
#plt.plot(time,joint_v4, label = 'Measured Joint 4 velocity')
#plt.plot(time,joint_v5, label = 'Measured Joint 5 velocity')
#plt.plot(time,joint_v6, label = 'Measured Joint 6 velocity')
#plt.plot(time,joint_v7, label = 'Measured Joint 7 velocity')

#plt.plot(time, desired_v1, label='Desired Joint 1 Velocity')
#plt.plot(time, desired_v2,  label='Desired Joint 2 Velocity')
plt.plot(time, desired_v3,  label='Desired Joint 3 Velocity')
#plt.plot(time, desired_v4,  label='Desired Joint 4 Velocity')
#plt.plot(time, desired_v5,  label='Desired Joint 5 Velocity')
#plt.plot(time, desired_v6,  label='Desired Joint 6 Velocity')
#plt.plot(time, desired_v7,  label='Desired Joint 7 Velocity')
plt.ylabel("Velocity (m/s)")
plt.xlabel("Time (s)")
plt.legend()
plt.show()

