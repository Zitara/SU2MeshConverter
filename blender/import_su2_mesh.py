#Notes:
#--------------------------
#Element Type:	Identifier
#--------------------------
#Line	           3
#Triangle	       5
#Quadrilateral     9
#Tetrahedral      10
#Hexahedral	      12
#Wedge	          13
#Pyramid	      14
#--------------------------

import bpy
import bmesh

def read_su2_file(context, filepath, use_some_setting):
    print("------------------------")
    print("running read_su2_file...")
    f = open(filepath, 'r', encoding='utf-8')
    #data = f.read()
    
    # Looking for the number of dimentions:
    while(1):
        s = f.readline()
        if s.find('NDIME') >-1:
            NDIME = int(s.split('=')[1])
            print("Number of dimentions:", NDIME)
            break
            
    # Looking for the number of elements for face construction:
    while(1):
        s = f.readline()
        if s.find('NELEM') >-1:
           NELEM = int(s.split('=')[1])#.split('>')[1].strip()
           print("Number of elements:", NELEM)
           break
    
    face = []
    # note: last dgit is the vert number?
    # serching for the number identifyer
    for e in range(NELEM):
        s = f.readline()
        vert = s.split()#.strip()
#        if (vert[0].strip()) == '3':  #then its a line
#            face.append(( 
#                   int(vert[1].strip()),
#                   int(vert[2].strip()),
#                   #int(vert[3].strip()),
#                   #int(vert[4].strip()),  
#                   ))
#        if (vert[0].strip()) == '5':  #then its a triangle
#            face.append(( 
#                   int(vert[1].strip()),
#                   int(vert[2].strip()),
#                   int(vert[3].strip()),
#                   #int(vert[4].strip()),   # trinagle needs only 3 verts
#                   ))
#        if (vert[0].strip()) == '9':  #then its a Quadrilateral
#            face.append(( 
#                   int(vert[1].strip()),
#                   int(vert[2].strip()),
#                   int(vert[3].strip()),
#                   int(vert[4].strip()),
#                   ))
#        if (vert[0].strip()) == '10':  #then its a Tetrahedral
#            face.append(( 
#                   int(vert[1].strip()),
#                   int(vert[2].strip()),
#                   int(vert[3].strip()),
#                   #int(vert[4].strip()),   # trinagle needs only 3 verts
#                   ))
#        if (vert[0].strip()) == '12':  #then its a Hexahedral
#            face.append(( 
#                   int(vert[1].strip()),
#                   int(vert[2].strip()),
#                   int(vert[3].strip()),
#                   #int(vert[4].strip()),   # trinagle needs only 3 verts
#                   ))
#        if (vert[0].strip()) == '13':  #then its a Wedge
#            face.append(( 
#                   int(vert[1].strip()),
#                   int(vert[2].strip()),
#                   int(vert[3].strip()),
#                   #int(vert[4].strip()),   # trinagle needs only 3 verts
#                   ))
#        if (vert[0].strip()) == '14':  #then its a Pyramid
#            face.append(( 
#                   int(vert[1].strip()),
#                   int(vert[2].strip()),
#                   int(vert[3].strip()),
#                   #int(vert[4].strip()),   # trinagle needs only 3 verts
#                   ))
        #print("s: ", s)
        #print("vert: ", vert)
        list = ()
        for i in range(len(vert)):
                list += (int(vert[i].strip()), )
                
        face.append((list))
        
        #else:
        #   print("Identifyer number not found..")
        
        #print(vert)
    #print("Face: ",face)
    # Looking for the verticaes locatin:
    while(1):
        s = f.readline()
        if s.find('NPOIN') >-1:
           NPOIN = int(s.split('=')[1].split('\t')[0])
           print("Number of points:", NPOIN)
           break
       
    verts = []
    for i in range(NPOIN):
        s = f.readline()
        #print("verts s: ",s)
        #loc = s.split('\t')
        loc = s.split()
        #print("loc: ",loc)
        #if loc[0] == '':
        #loc.remove('')
            
        if (NDIME) == 2:  #then its a 2D, (x,y,0)
            #print("2D")
            verts.append(( 
                   float(loc[0].strip()),
                   float(loc[1].strip()),
                   float(0)
                   ))
        if (NDIME) == 3:
            #print("3D")
            verts.append((
                   float(loc[0].strip()),
                   float(loc[1].strip()),
                   float(loc[2].strip()),
                   ))
    #print("vers: " , verts)
        
    f.close()

    # would normally load the data here
    #print(data)
    
    
    #while(1)
    print("face[3]= ", face[3])
    print("Finished importing SU2 mesh...")
    return verts, face
    #return {'FINISHED'}

#def add_SU2_mesh(data):
#    print(data[1])

#class AddSU2(bpy.types.Operator):
#    bl_idname = "mesh.su2_add"
#    bl_label = "Add SU2"
#    bl_options = {'REGISTER', 'UNDO'}
#    
#    def execute(self, context):
#        verts_loc, faces = add_SU2_mesh(data)





# ImportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty, FloatVectorProperty
from bpy.types import Operator


class ImportSU2MeshData(Operator, ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_su2.mesh_data"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Import SU2 Mesh Data"

    # ImportHelper mixin class uses this
    filename_ext = ".su2"

    filter_glob = StringProperty(
            default="*.su2",
            options={'HIDDEN'},
            )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    use_setting = BoolProperty(
            name="Example Boolean",
            description="Example Tooltip",
            default=True,
            )

    type = EnumProperty(
            name="Example Enum",
            description="Choose between two items",
            items=(('OPT_A', "First Option", "Description one"),
                   ('OPT_B', "Second Option", "Description two")),
            default='OPT_A',
            )
    
    # generic transform props
    view_align = BoolProperty(
            name="Align to View",
            default=False,
            )
    location = FloatVectorProperty(
            name="Location",
            subtype='TRANSLATION',
            )
    rotation = FloatVectorProperty(
            name="Rotation",
            subtype='EULER',
            )

    def execute(self, context):
        print("executing...")
        # drawing the su2 mesh:
        
        verts_loc, faces = read_su2_file(context, self.filepath, self.use_setting)
        #print("Verts_loc:", verts_loc)
        #print("faces: ", faces)
        
        
        mesh = bpy.data.meshes.new("SU2")
        
        bm = bmesh.new()
        
        for v_co in verts_loc:
            bm.verts.new(v_co)
        
        bm.verts.ensure_lookup_table()
        
        print("Constructing faces...")
        #print("face constr: ", faces)
        for f_idx in faces:
            #bm.faces.new([bm.verts[i] for i in f_idx])
#--------------------------
#Line	           3
#Triangle	       5
#Quadrilateral     9
#Tetrahedral      10
#Hexahedral	      12
#Wedge	          13
#Pyramid	      14
#--------------------------
            print("f_idx= ", f_idx)
            if f_idx[0] == 3:
                bm.faces.new((bm.verts[f_idx[1]], 
                              bm.verts[f_idx[2]]))
            if f_idx[0] == 5:
                bm.faces.new((bm.verts[f_idx[1]], 
                              bm.verts[f_idx[2]], 
                              bm.verts[f_idx[3]]))
            if f_idx[0] == 9:
                bm.faces.new((bm.verts[f_idx[1]], 
                              bm.verts[f_idx[2]], 
                              bm.verts[f_idx[3]], 
                              bm.verts[f_idx[4]],))
            if f_idx[0] == 10:
                bm.faces.new((bm.verts[f_idx[1]], 
                              bm.verts[f_idx[2]], 
                              bm.verts[f_idx[3]],
                              bm.verts[f_idx[4]],))
            if f_idx[0] == 12:
                bm.faces.new((bm.verts[f_idx[1]], 
                              bm.verts[f_idx[2]], 
                              bm.verts[f_idx[3]],
                              bm.verts[f_idx[4]],))
            if f_idx[0] == 13:
                bm.faces.new((bm.verts[f_idx[1]], 
                              bm.verts[f_idx[2]], 
                              bm.verts[f_idx[3]],
                              bm.verts[f_idx[4]],))
            if f_idx[0] == 14:
                bm.faces.new((bm.verts[f_idx[1]],
                              bm.verts[f_idx[2]], 
                              bm.verts[f_idx[3]],
                              bm.verts[f_idx[4]],))
            
#            #print("[bm.verts[i] for i in f_idx]", [bm.verts[i] for i in f_idx])
#            print("f_idx: ", f_idx)
#            
#            if f_idx[0] == 3:#Line	           3
#                bm.faces.new(
#                            bm.verts[1],
#                            bm.verts[2],
#                            #bm.verts[3],
#                            )
#            if f_idx[0] == 5:#Triangle	       5
#                bm.faces.new((bm.verts[f_idx[1]], bm.verts[f_idx[2]], bm.verts[f_idx[3]]))
##                            bm.verts[f_idx[1]],
##                            bm.verts[f_idx[2]],
##                            bm.verts[f_idx[3]],
##                            )
#            if f_idx[0] == 9:#Quadrilateral     9
#                bm.faces.new(
#                            bm.verts[1],
#                            bm.verts[2],
#                            bm.verts[3],
#                            bm.verts[4],
#                            )
#            if f_idx[0] == 10:#Tetrahedral      10
#                bm.faces.new(
#                            bm.verts[1],
#                            bm.verts[2],
#                            bm.verts[3],
#                            bm.verts[4],
#                            )
#            if f_idx[0] == 12:#Hexahedral	      12
#                bm.faces.new(
#                            bm.verts[1],
#                            bm.verts[2],
#                            bm.verts[3],
#                            bm.verts[4],
#                            )
#            if f_idx[0] == 13:#Wedge	          13
#                bm.faces.new(
#                            bm.verts[1],
#                            bm.verts[2],
#                            bm.verts[3],
#                            bm.verts[4],
#                            )
#            if f_idx[0] == 14:#Pyramid	      14
#                bm.faces.new(
#                            bm.verts[1],
#                            bm.verts[2],
#                            bm.verts[3],
#                            bm.verts[4],
#                            )
        
        bm.to_mesh(mesh)
        mesh.update()
        

        
        # add the mesh as an object into the scene with this utility module
        from bpy_extras import object_utils
        object_utils.object_data_add(context, mesh, operator=self)
        
        return {'FINISHED'}
        #return read_su2_file(context, self.filepath, self.use_setting)

##---------------------------------------------------
#class AddSU2Mesh(bpy.types.Operator):
#    """Add the SU2 mesh"""
#    bl_idname = "mesh.su2_mesh_add"
#    bl_label = "Add SU2"
#    bl_options = {'REGISTER', 'UNDO'}
    
    
# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(ImportSU2MeshData.bl_idname, text="SU2 Import Operator")


def register():
    bpy.utils.register_class(ImportSU2MeshData)
    bpy.types.INFO_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ImportSU2MeshData)
    bpy.types.INFO_MT_file_import.remove(menu_func_import)


if __name__ == "__main__":
    register()

    # test call
    bpy.ops.import_su2.mesh_data('INVOKE_DEFAULT')
