# This code is used to extract the coordinates of the joints of the 3D model in Blender. It is executed in the "Scripting" tab in Blender. 
# Questions or support: srodrigo@mit.edu 


import bpy
import csv


armature_name = "Armature"  # Armature name in Blender
joint_list = ["joint40", "joint44", "joint57", "joint39"]  # List of joints to track (40,44,57 and 39 are the foot in our 3D animation)
start_frame = 0             
end_frame = 51              # (walking animation)

# We ensure the armature is in Pose mode
armature = bpy.data.objects[armature_name]
bpy.context.view_layer.objects.active = armature
bpy.ops.object.mode_set(mode='POSE')

bone_coords = []

for frame in range(start_frame, end_frame + 1):
    # We select the current frame
    bpy.context.scene.frame_set(frame)
    
    # Loop through each joint in our tracking list
    for joint_name in joint_list:
        
        bone = armature.pose.bones[joint_name]        
        # Get the bone's location in world space
        bone_matrix = armature.matrix_world @ bone.matrix
        bone_location = bone_matrix.translation  # Extract world space location
        
        # We append the coords
        bone_coords.append((frame, joint_name, bone_location.x, bone_location.y, bone_location.z))


output_file = "A:/Yo/2-Universidad/MIT/PANDA/3D/coords_animation/all_legs_0to51.csv"

# Results to a CSV file
with open(output_file, mode='w', newline='') as f:
    csv_writer = csv.writer(f)
    csv_writer.writerow(["FRAME", "JOINT_NUM", "X", "Y", "Z"])
       
    for coord in bone_coords:
        csv_writer.writerow(coord)

print(f"Coordinates saved to {output_file}")