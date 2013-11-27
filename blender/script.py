import bpy
import io_anim_bvh.import_bvh
from bpy_extras.io_utils import (axis_conversion)

new_file_name = '/home/cvssp/misc/m04701/workspace/data/bvh/out2.bvh'
global_scale=0.3
global_matrix = axis_conversion('-Z',
                               'Y',
                                ).to_4x4()
                                         
io_anim_bvh.import_bvh.load(None, bpy.context, new_file_name,
	'ARMATURE', 'NATIVE', global_scale, False, 1, global_matrix )

current_skel = bpy.context.scene.objects["Armature"]
new_skel = bpy.context.scene.objects["out2"]

#Do not show the new skeleton
new_skel.hide = True

#For each bone in the rigged skeleton add a copy transformation
#for the new skeleton
for bone in list(current_skel.pose.bones):
	#If we do not want translations then uncomment this line and comment the next
	#bone.constraints.new(type='COPY_ROTATION')
	bone.constraints.new(type='COPY_TRANSFORMS')
	constraint = bone.constraints[0]
	constraint.target = new_skel
	constraint.subtarget = bone.name
