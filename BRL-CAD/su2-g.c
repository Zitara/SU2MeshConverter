/*                           S U 2 - G . C
 */
/** @file su2-g.c
 *
 * Converting su2 mesh format into BRL-CAD .g binary format
 * Based on stl-g.c file
 *
 */

#include "common.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include "bnetwork.h"
#include "bio.h"

#include "bu/cv.h"
#include "bu/getopt.h"
#include "bu/units.h"
#include "vmath.h"
#include "rt/geom.h"
#include "raytrace.h"
#include "wdb.h"

static struct vert_root  *tree_root;
static struct wmember all_head;
static struct bn_tol tol;	/* tolerance structure */
static struct rt_wdb *fd_out;	/* resulting BRL-CAD file*/

static char *input_file;	/* name of the input file */
static char *brlcad_file;	/* name of the output file */
static char *forced_name=NULL;	/* name specified on command line */
static FILE *fd_in;			/* input file */

//static int solid_count=0;	/* count of solids converted */
static int id_no=1000;      /* ident numbers */
//static int const_id=-1;	/* constant ident number (assigned to all regions if non-negative) */
static int mat_code=1;	/* default material code */
//static int debug=0;		/* debug flag */
static int *bot_faces=NULL;	/* array of ints (indices into tree_root->the_array array) three per face */
static int bot_fsize=0;	/* current size of the bot_faces array */
static int bot_fcurr=0;	/* current bot face */
//static unsigned int nELEM=0, elemNumber, nPOIN=0, pointNumber;
static int const_id=-1;	/* Constatn ident number (assigned to all regions if non-negative */

static unsigned int obj_count=0;	/* count of parts converted for "su2-g" conversions */
static float conv_factor=1.0;	/* conversion factor from model units to mm */


static double **pointBag=NULL,
    loc[3]={0,0,0};
static int face_count=0;
static unsigned int nELEM,
             nMARK=0,
             nELEM=0,
             nPOIN=0,
             nDIM=0,
             ELEMS=0,
             code[10]={0};
//             **elemBag=NULL;
static char TAG[20];

static unsigned char color[3]={128, 128, 128};

//struct bu_vls solid_name = BU_VLS_INIT_ZERO;
//struct bu_vls region_name = BU_VLS_INIT_ZERO;
struct wmember head;

/* soze of blocks of faces to malloc */
#define BOT_FBLOCK 128
#define MAX_LINE_SIZE 512

static void
usage(const char *argv0)
{
    bu_log("Usage: %s [-db] [-t tolerance] [-N forced_name] [-i initial_ident] [-I constant_ident] [-m material_code] [-c units_su2] [-x rt_debug_flag] input.su2 output.g\n", argv0);
    bu_log("	where input.su2 is a mesh file format for CFD SU2 suit\n");
    bu_log("	and output.g is the name of a BRL-CAD database file to receive the conversion.\n");
    bu_log("	The -d option prints additional debugging information.\n");
    bu_log("	The -t option specifies the minimum distance between two distinct vertices (mm).\n");
    bu_log("	The -N option specifies a name to use for the object.\n");
    bu_log("	The -i option sets the initial region ident number (default is 1000).\n");
    bu_log("	The -I option sets the ident number that will be assigned to all regions (conflicts with -i).\n");
    bu_log("	The -m option sets the integer material code for all the parts (default is 1).\n");
    bu_log("	The -c option specifies the units used in the SU2 file (units_str may be \"in\", \"ft\", ... default is \"mm\"\n");
    bu_log("	The -x option specifies an RT debug flags (see raytrace.h).\n");
}

void
Add_face(int face[3])
{
    if (!bot_faces){
        bot_faces = (int *)bu_malloc(3 * BOT_FBLOCK * sizeof(int), "bot_faces");
        bot_fsize = BOT_FBLOCK;
        bot_fcurr = 0;
    } else if (bot_fcurr >= bot_fsize){
        bot_fsize += BOT_FBLOCK;
        bot_faces = (int *)bu_realloc((void *)bot_faces, 3 * bot_fsize * sizeof(int), "bot_faces increase");
    }
    VMOVE(&bot_faces[3*bot_fcurr], face);
    bot_fcurr++;
}

void
mk_unique_brlcad_name(struct bu_vls *name)
{
    char *c;
    int count=0;
    size_t len;

    c = bu_vls_addr(name);

    while (*c != '\0'){
        if (*c == '/' || !isprint((int)*c)){
            *c = '_';
        }
        c++;
    }

    len = bu_vls_strlen(name);
    while (db_lookup(fd_out->dbip, bu_vls_addr(name), LOOKUP_QUIET) != RT_DIR_NULL){
        char suff[10];

        bu_vls_trunc(name, len);
        count++;
        sprintf(suff, "_%d", count);
        bu_vls_strcat(name, suff);
    }
}

void
solidifying(char *tags){
    struct bu_vls solid_name = BU_VLS_INIT_ZERO;

    bu_vls_strcpy(&solid_name, tags);
//    bu_vls_vlscat(&solid_name, &region_name);
    mk_unique_brlcad_name(&solid_name);

    bu_log("\tUsing solid name: %s\n", bu_vls_addr(&solid_name));

    /* check if has any solid parts */
    if (face_count == 0){
        bu_log("\t%s has no solid parts, ignoring\n", bu_vls_addr(&solid_name));
        bu_vls_free(&solid_name);

        return;
    }

    mk_bot(fd_out, bu_vls_addr(&solid_name), RT_BOT_SOLID, RT_BOT_UNORIENTED,
           0, tree_root->curr_vert, bot_fcurr, tree_root->the_array, bot_faces,
           NULL, NULL);
    clean_vert_tree(tree_root);

    if (face_count){
        (void)mk_addmember(bu_vls_addr(&solid_name), &head.l, NULL, WMOP_UNION);
    }
    face_count = 0;

//    if (bot_faces) {
////        bu_free(bot_faces, "bot_faces");
//        while (bot_faces != NULL) {
//            bot_faces = NULL;
//            bot_faces++;
//        }
//    }
    if (bot_faces) {
        bot_faces = NULL;
    }

//    memset(bot_faces, NULL, sizeof(bot_faces));

    bu_vls_free(&solid_name);
}

static void
constructing(char *line)
{
    /* Name VTK code    |id	|#points
     * --------------------------
     * Line	           	3	2
     * Triangle			5	3
     * Quadrilateral   	9	4
     * Tetrahedral    	10	4
     * Hexahedral	  	12	8
     * Wedge	  		13	6
     * Pyramid	  		14	5
     * --------------------------
     */

    int i=0;
    int vtkType=0;
    int start;
    int tmp_face[3] = {0, 0, 0};

    start = 0;
    sscanf(&line[start], "%d", &code[0]);
    start++;

    switch (code[0]){
    case 3:
        sscanf(&line[start], "%d%d",
               &code[1], &code[1]);

        for (vtkType=1; vtkType <= 2; vtkType++){
            tmp_face[vtkType-1] = Add_vert(
                        pointBag[code[vtkType]][0],
                        pointBag[code[vtkType]][1],
                        pointBag[code[vtkType]][2],
                        tree_root, tol.dist_sq);
            //bu_log("\tvertex #%d: (%g %g %g)\n", tmp_face[vtkType],
            //V3ARGS(&tree_root->the_array[3 *tmp_face[vtkType]]));
        }
        break;
    case 5:
        sscanf(&line[start], "%d%d%d",
               &code[1], &code[2], &code[3]);

        for (vtkType=1; vtkType <= 3; vtkType++){
            tmp_face[vtkType-1] = Add_vert(
                        pointBag[code[vtkType]][0],
                        pointBag[code[vtkType]][1],
                        pointBag[code[vtkType]][2],
                        tree_root, tol.dist_sq);
        }
        break;
    case 9:
        sscanf(&line[start], "%d%d %d%d",
               &code[1], &code[2], &code[3], &code[4]);

        for (vtkType=1; vtkType <= 4; vtkType++){
            tmp_face[vtkType-1] = Add_vert(
                        pointBag[code[vtkType]][0],
                        pointBag[code[vtkType]][1],
                        pointBag[code[vtkType]][2],
                        tree_root, tol.dist_sq);
        }
        break;
    case 10:
        sscanf(&line[start], "%d%d %d%d",
               &code[1], &code[2], &code[3], &code[4]);

        for (vtkType=1; vtkType <= 4; vtkType++){
            tmp_face[vtkType-1] = Add_vert(
                        pointBag[code[vtkType]][0],
                        pointBag[code[vtkType]][1],
                        pointBag[code[vtkType]][2],
                        tree_root, tol.dist_sq);
        }
        break;
    case 12:
        sscanf(&line[start], "%d%d %d%d %d%d %d%d" ,
               &code[1], &code[2], &code[3], &code[4],
               &code[5], &code[6], &code[7], &code[8]);

        for (vtkType=1; vtkType <= 8; vtkType++){
            tmp_face[vtkType-1] = Add_vert(
                        pointBag[code[vtkType]][0],
                        pointBag[code[vtkType]][1],
                        pointBag[code[vtkType]][2],
                        tree_root, tol.dist_sq);
        }
        break;
    case 13:
        sscanf(&line[start], "%d%d%d %d%d%d" ,
               &code[1], &code[2], &code[3],
               &code[4], &code[5], &code[6]);

        for (vtkType=1; vtkType <= 6; vtkType++){
            tmp_face[vtkType-1] = Add_vert(
                        pointBag[code[vtkType]][0],
                        pointBag[code[vtkType]][1],
                        pointBag[code[vtkType]][2],
                        tree_root, tol.dist_sq);
        }
        break;
    case 14:
        sscanf(&line[start], "%d%d %d%d %d" ,
               &code[1], &code[2], &code[3],
               &code[4], &code[5]);

        for (vtkType=1; vtkType <= 5; vtkType++){
            tmp_face[vtkType-1] = Add_vert(
                        pointBag[code[vtkType]][0],
                        pointBag[code[vtkType]][1],
                        pointBag[code[vtkType]][2],
                        tree_root, tol.dist_sq);
        }
        break;
    default:
        bu_log("VTK type ERROR! code %d\n", code[0]);
        break;
    }

    for (i=0; i<10; i++){
        code[i]=0;
    }

//    for (i=0; i<3; i++){
//        bu_log("\tvertex #%d: (%g %g %g)\n", tmp_face[i],
//               V3ARGS(&tree_root->the_array[3 *tmp_face[i]]));
//    }

    Add_face(tmp_face);
    face_count++;
}

static void
reading_vertices()
{
    int start;
    char line[MAX_LINE_SIZE];

    while (bu_fgets(line, MAX_LINE_SIZE, fd_in) != NULL){
        start = (-1);
        // ignoring spaces
        while (isspace((int)line[++start]));
        // ignoring %
        while (!bu_strncmp(&line[start], "%", 1)){
            ++start;
        }
        // search for the number of dimentions (2D or 3D)
        if (!bu_strncmp(&line[start], "NDIME", 5)){
            //int nDIM;
            start += 6;
            sscanf(&line[start], "%d", &nDIM);
            bu_log("NDIME = %d\n", nDIM);
            break;
        }
    }

    start = (-1);
    while (bu_fgets(line, MAX_LINE_SIZE, fd_in) != NULL){
        start = (-1);
        // ignoring spaces
        while (isspace((int)line[++start]));
        // ignoring %
        while (!bu_strncmp(&line[start], "%", 1)){
            ++start;
        }

        // Number of points
        if (!bu_strncmp(&line[start], "NPOIN", 5)){
            start += 6;
            sscanf(&line[start], "%d", &nPOIN);
            bu_log("NPOIN = %d\n", nPOIN);
            pointBag = (double **)bu_malloc(nPOIN * sizeof(double *), "pointBag");
            // storing points
        } else if (nPOIN !=0){
            int index;

            if (nDIM == 2) {
                sscanf(&line[start], "%lf%lf %d", &loc[0], &loc[1], &index);
            } else if (nDIM == 3) {
                sscanf(&line[start], "%lf%lf%lf %d", &loc[0], &loc[1], &loc[2], &index);
            } else {
                bu_log("NDIM not found ...");
                break;
            }
                loc[0] *= conv_factor;
                loc[1] *= conv_factor;
                loc[2] *= conv_factor;

            pointBag[index] = (double *)bu_malloc(3 * sizeof(double), "pointBag");
            pointBag[index][0] = loc[0];
            pointBag[index][1] = loc[1];
            pointBag[index][2] = loc[2];

//            bu_log("pointBag: %f %f %f\n",
//                   (*pointBag)[index][0],
//                   (*pointBag)[index][1],
//                   (*pointBag)[index][2]);

            nPOIN--;
         }
    }
}

static void
reading_mesh()
{
    char line[MAX_LINE_SIZE];
    int start;
    bot_fcurr = 0;

    if (RT_G_DEBUG & DEBUG_MEM || RT_G_DEBUG & DEBUG_MEM_FULL)
        bu_prmem("At start of convert_part()");

    while (bu_fgets(line, MAX_LINE_SIZE, fd_in) != NULL){
        start = (-1);

        while (isspace((int)line[++start]) && line[start] != '\0');

        // Number of elements
        if (!bu_strncmp(&line[start], "NELEM", 5)){
            start += 6;
            sscanf(&line[start], "%d", &nELEM);
            bu_log("NELEM = %d\n", nELEM);

        // storing elements
        } else if (nELEM != 0){
              constructing(line);

            nELEM--;
            if (nELEM == 0){
                bu_log("Elements constracted. ");
                solidifying("s.MESH");
            }

        // Number of Markers
        } else if (!bu_strncmp(&line[start], "NMARK", 5)){
            //int nMARK;
            start += 6;
            sscanf(&line[start], "%d", &nMARK);
            bu_log("NMARK = %d\n", nMARK);
        // Tagging markers
        } else if (!bu_strncmp(&line[start], "MARKER_TAG", 10)){
            //char TAG[20];
            start += 11;
            sscanf(&line[start], "%s", TAG);
//            bu_log("MARKER_TAG = %s. ", TAG);
        // Marker elements
        } else if (!bu_strncmp(&line[start], "MARKER_ELEMS", 12)){
            ELEMS = 0;
            start += 13;
            sscanf(&line[start], "%d", &ELEMS);
            bu_log("MARKER_ELEMS = %d. ", ELEMS);
        } else if (ELEMS != 0){

            constructing(line);

            ELEMS--;
            if (ELEMS == 0){
//                bu_log("MARKER_TAG. ");
                solidifying(TAG);
            }
        }
    }
}

static void
convert_input(){
    struct bu_vls region_name = BU_VLS_INIT_ZERO;

    if (RT_G_DEBUG & DEBUG_MEM_FULL){
        bu_prmem("At start of convert_part():\n");
        bu_log("Barrier check cat start of convert_part:\n");
        if (bu_mem_barriercheck())
            bu_exit(EXIT_FAILURE, "Barrier check failed!\n");
    }

    //-----------------------------------------------------------
    BU_LIST_INIT(&head.l);

    if (forced_name){
        bu_vls_strcpy(&region_name, forced_name);
    }else {
        /* build a name from the file name */
        char *ptr;
        int base_len;

        obj_count++;

        /* copy the file name into our work space (skip over path) */
        ptr = strrchr(input_file, '/');
        if (ptr){
            ptr++;
            bu_vls_strcpy(&region_name, ptr);
        } else {
            bu_vls_strcpy(&region_name, input_file);
        }

        /*  eliminate a trailing ".su2" */
        ptr = strstr(bu_vls_addr(&region_name), ".su2");
        if ((base_len=(ptr - bu_vls_addr(&region_name))) > 0){
            bu_vls_trunc(&region_name, base_len);
        }
    }

    mk_unique_brlcad_name(&region_name);
    bu_log("Converting Part: %s\n", bu_vls_addr(&region_name));

    /* getting vertices location */
    reading_vertices();

    /* constructing solids */
    fd_in=fopen(input_file, "rb");
    reading_mesh();

    /* Making region: */
    bu_log("\tMaking region (%s)\n", bu_vls_addr(&region_name));

    if (const_id >= 0){
        mk_lrcomb(fd_out, bu_vls_addr(&region_name), &head, 1, (char *)NULL,
                  (char *)NULL, color, const_id, 0, mat_code, 100, 0);
        if (face_count){
            (void)mk_addmember(bu_vls_addr(&region_name), &all_head.l, NULL,
                               WMOP_UNION);
        }
    } else {
        mk_lrcomb(fd_out, bu_vls_addr(&region_name), &head, 1, (char *)NULL,
                  (char *)NULL, color, id_no, 0, mat_code, 100, 0);
        if (face_count)
            (void)mk_addmember(bu_vls_addr(&region_name), &all_head.l, NULL,
                               WMOP_UNION);
    id_no++;
    }

    if (RT_G_DEBUG & DEBUG_MEM_FULL){
        bu_log("Barrier check at end of Convert_part:\n");
        if (bu_mem_barriercheck())
            bu_exit(EXIT_FAILURE, "Barrier chek failed!\n");
    }

    bu_vls_free(&region_name);

    return;
}

int
main(int argc, char *argv[]){
    int c;
    tol.magic = BN_TOL_MAGIC;
    tol.dist = 0.005;
    tol.dist_sq = tol.dist * tol.dist;
    tol.perp = 1e-9;
    tol.para = 1 - tol.perp;

    forced_name = NULL;

    conv_factor = 1.0;

    if (argc < 2){
        usage(argv[0]);
        bu_exit(1, NULL);
    }

    /* Get command line arguments. */
    while ((c = bu_getopt(argc, argv, "bt:i:I:m:dx:N:c:")) != -1){
        double tmp;

        switch (c){
            case 't': tmp = atof(bu_optarg);
                      if (tmp <= 0.0){
                          bu_log("Tolerance must be > 0, using default (&g)\n", tol.dist);
                          break;
                      }
                      tol.dist = tmp;
                      tol.dist_sq = tmp * tmp;
                      break;
            case 'c': conv_factor = bu_units_conversion(bu_optarg);
                      if (ZERO(conv_factor)) {
                          bu_log("Illegal units: (%s)\n", bu_optarg);
                          bu_exit(EXIT_FAILURE, "Illegal units!\n");
                      } else
                          bu_log("Converting units from %s to mm (conversion factor is %g)\n", bu_optarg, conv_factor);
                      break;
            case 'N': forced_name = bu_optarg;
                      break;
            case 'i': id_no = atoi(bu_optarg);
                      break;
            case 'm': mat_code = atoi(bu_optarg);
                      break;
            default:
                      usage(argv[0]);
                      bu_exit(1, NULL);
                      break;
        }
    }

    input_file = argv[bu_optind];
    if ((fd_in=fopen(input_file, "rb")) == NULL){
        bu_log("Cannot open input file (%s)\n", input_file);
        perror(argv[0]);
        bu_exit(1, NULL);
    }
    bu_optind++;
    brlcad_file = argv[bu_optind];
    if ((fd_out=wdb_fopen(brlcad_file)) == NULL){
        bu_log("Cannot open BRL-CAD file (%s)\n", brlcad_file);
        perror(argv[0]);
        bu_exit(1, NULL);
    }

    mk_id_units(fd_out, "Conversion from SU2 mesh format", "mm");

    BU_LIST_INIT(&all_head.l);

    /* create a tree structure to hold the input vertices */
    tree_root = create_vert_tree();
    convert_input();

    /* make a top level group */
    mk_lcomb(fd_out, "all", &all_head, 0, (char *)NULL, (char *)NULL, (unsigned char *)NULL, 0);
    fclose(fd_in);
    wdb_close(fd_out);

    return 0;
}

