# SU2MeshConverter
Using Blender and BRL-CAD to import/export SU2 mesh files. 
Hopefully one day SU2 can be integrated to work within. 

BRL-CAD:
You would need to update the CMakeList.txt in src/conv directory in order for cmake to pick it up, and add it to the building process.
However, using the program is as simple as adding -o <output-file-name-desierd.su2> <db.g> <object-withen-g-file>

Blender:
Add your file as a python script, it is based on the template scripts, so it should act the same.
