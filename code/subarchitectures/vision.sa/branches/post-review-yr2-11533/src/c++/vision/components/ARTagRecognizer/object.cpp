
/* 
** ARToolKit object parsing function 
**   - reads in object data from object file in Data/object_data
**
** Format:
** <obj_num>
**
** <obj_name>
** <obj_pattern filename>
** <marker_width>
** <centerX centerY>
**
** ...
**
**	eg
**
**	#pattern 1
**	Hiro
**	Data/hiroPatt 
**	80.0
**	0.0 0.0
**
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <AR/ar.h>
#include <string>
#include <assert.h>
#include "object.h"

namespace ar_object
{

  static char *get_buff (char *buf, int n, FILE * fp);

  ObjectData_T *read_ObjData (char *name, int *objectnum)
  {
    FILE *fp;
    ObjectData_T *object;
    char buf[256], buf1[256];
    int i;
     // std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
    std::string package_path = "";
      printf("Opening Data File %s", name);

    if ((fp = fopen (name, "r")) == NULL)
    {
      printf ("Can't find the file - quitting");
      exit(0);
      }

    get_buff (buf, 256, fp);
    if (sscanf (buf, "%d", objectnum) != 1)
    {
      fclose (fp);
    }

    printf ("About to load %d Models", *objectnum);

    object = (ObjectData_T *) malloc (sizeof (ObjectData_T) * *objectnum);
    if (object == NULL){
    	printf("object is null ,your argument is invalid. \n");
    	 exit(0);
    }

    for (i = 0; i < *objectnum; i++)
    {
      object[i].visible = 0;

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%s", object[i].name) != 1)
      {
        fclose (fp);
        free (object);
      }

      printf ("Read in No.%d", i + 1);

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%s", buf1) != 1)
      {
        fclose (fp);
        free (object);
      }

      char path[1024];
      size_t size;
      getcwd(path, size);
   sprintf (buf, "%s/%s", path, buf1);
      if ((object[i].id = arLoadPatt (buf)) < 0)
      {
        fclose (fp);
        free (object);
      }

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%lf", &object[i].marker_width) != 1)
      {
        fclose (fp);
        free (object);
      }

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%lf %lf", &object[i].marker_center[0], &object[i].marker_center[1]) != 2)
      {
        fclose (fp);
        free (object);
      }
    }

    fclose (fp);

    return (object);
  }

  static char *get_buff (char *buf, int n, FILE * fp)
  {
    char *ret;

    for (;;)
    {
      ret = fgets (buf, n, fp);
      if (ret == NULL)
        return (NULL);
      if (buf[0] != '\n' && buf[0] != '#')
        return (ret);
    }
  }
}
