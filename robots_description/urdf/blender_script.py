import bpy

selected = bpy.context.selected_objects

for obj in selected:
    # ensure origin is centered on bounding box center
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
    # create a cylinder for the bounding box
    bpy.ops.mesh.primitive_cylinder_add()
    # our new cylinder is now the active object, so we can keep track of it in a variable:
    bound_box = bpy.context.active_object 

    # print('obj.dimensions = {}'.format(obj.dimensions))
    xy_max = max(obj.dimensions.x, obj.dimensions.y)
    # print('xy_max = {}'.format(xy_max))
    # copy transforms
    bound_box.dimensions = (xy_max, xy_max, obj.dimensions.z)
    # print('bound_box.dimensions = {}'.format(bound_box.dimensions))
    bound_box.location = obj.location
    bound_box.rotation_euler = obj.rotation_euler
    