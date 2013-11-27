import bpy

#filepath = '/home/cvssp/misc/m04701/Downloads/Dog_Modelling_Blender/Dog_modelling_head_root.blend'
#lib = bpy.data.libraries.load(filepath)

#scn = bpy.data.scenes[0]                       # get current scene

#for attr in dir(lib):
#    setattr(scn, attr, getattr(lib, attr))

current_skel = bpy.context.scene.objects["Armature"]
#current_skel_arm = bpy.data.armatures["Armature"]
new_skel = bpy.context.scene.objects["out2"]

#For each bone in the rigged skeleton add a copy transformation
#for the new skeleton
for bone in list(current_skel.pose.bones):
	bone.constraints.new(type='COPY_TRANSFORMS')
	constraint = bone.constraints[0]
	constraint.target = new_skel
	constraint.subtarget = bone.name
