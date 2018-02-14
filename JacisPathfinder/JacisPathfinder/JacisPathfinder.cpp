//----------------------------------------------------------------------------
//
//  $Workfile: JacisPathfinder.cpp$
//
//  $Revision: X$
//
//  Project:    JacisPathfinders
//
//                            Copyright (c) 2018
//                               Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
#include "pathfinder.h"
#include <stdio.h>
#include <string.h>

//----------------------------------------------------------------------------
//  Defines
//----------------------------------------------------------------------------
#define MAX_LINE 200

//----------------------------------------------------------------------------
//  Constants
//----------------------------------------------------------------------------
const double TIME_SLICE =    0.02;
const double MAX_VEL    =  144.0;
const double MAX_ACCEL  =  600.0;
const double MAX_JERK   = 1000.0;

//----------------------------------------------------------------------------
//  Local Functions
//----------------------------------------------------------------------------
void createPaths(char** fields);

//----------------------------------------------------------------------------
//  Purpose:
//      Split the string into seperate chunks
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
char** str_split(char* a_str, const char a_delim)
{
  char** result = 0;
  size_t count = 0;
  char* tmp = a_str;
  char* last_comma = 0;
  char delim[2];
  delim[0] = a_delim;
  delim[1] = 0;

  /* Count how many elements will be extracted. */
  while (*tmp)
  {
    if (a_delim == *tmp)
    {
      count++;
      last_comma = tmp;
    }
    tmp++;
  }

  /* Add space for trailing token. */
  count += last_comma < (a_str + strlen(a_str) - 1);

  /* Add space for terminating null string so caller
  knows where the list of returned strings ends. */
  count++;

  result = (char**)malloc(sizeof(char*) * count);

  if (result)
  {
    size_t idx = 0;
    char* token = strtok(a_str, delim);

    while (token)
    {
      *(result + idx++) = _strdup(token);
      token = strtok(0, delim);
    }
    *(result + idx) = 0;
  }

  return result;
}

//----------------------------------------------------------------------------
//  Purpose:
//      Main, read things in put things out
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
int main()
{
  // Get the file
  FILE* csvFile = fopen("paths.csv","r");
  char line[MAX_LINE];

  if (NULL == csvFile)
  {
    printf("Can't find the paths.csv file!");
    fclose(csvFile);
    return -1;
  }

  fgets(line, MAX_LINE, csvFile);

  while (!feof(csvFile))
  {
    fgets(line, MAX_LINE, csvFile);

    if (strlen(line) > 0)
    {
      printf("%s", line);

      char** tokens = str_split(line, ',');

      if (tokens)
      {
        createPaths(tokens);
        free(tokens);
      }
    }
  }

  return 0;
}


//----------------------------------------------------------------------------
//  Purpose:
//      Make the path file
//
//  Notes:
//      None
//
//----------------------------------------------------------------------------
void createPaths(char** fields)
{
  char filename[MAX_LINE];

  sprintf(filename, "%sPath.java", fields[0]);

  FILE* javaFile = fopen(filename,"w");

  if (NULL == javaFile)
  {
    printf("Can't open:%s\n", filename);
    return;
  }

  fprintf(javaFile, "/* %s %s */\n", fields[0], fields[1]);


  int pointLength = atoi(fields[2]);

  Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * pointLength);

  for (int i = 0; i < pointLength; i++)
  {
    points[i].x = atof(fields[3 + (i * 3)]);
    points[i].y = atof(fields[3 + (i * 3)+1]);
    points[i].angle = d2r(atof(fields[3 + (i * 3)+2]));
  }

  TrajectoryCandidate candidate;
  pathfinder_prepare(points, 
    pointLength, 
    FIT_HERMITE_CUBIC, 
    PATHFINDER_SAMPLES_HIGH, 
    TIME_SLICE,
    MAX_VEL,
    MAX_ACCEL,
    MAX_JERK,
    &candidate);

  int length = candidate.length;
  Segment *trajectory =(Segment *)malloc(length * sizeof(Segment));

  pathfinder_generate(&candidate, trajectory);

  Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
  Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

  double wheelbase_width = 14;  //24

  pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

  fprintf(javaFile, "//----------------------------------------------------------------------------");
  fprintf(javaFile, "//");
  fprintf(javaFile, "//  $Workfile: %s$", filename);
  fprintf(javaFile, "//");
  fprintf(javaFile, "//  $Revision: X$");
  fprintf(javaFile, "//");
  fprintf(javaFile, "//  Project:    Paths");
  fprintf(javaFile, "//");
  fprintf(javaFile, "//                            Copyright (c) 2018");
  fprintf(javaFile, "//                               Cedarcrest High School");
  fprintf(javaFile, "//                            All Rights Reserved");
  fprintf(javaFile, "//");
  fprintf(javaFile, "//  Modification History:");
  fprintf(javaFile, "//  $Log:");
  fprintf(javaFile, "//  $");
  fprintf(javaFile, "//");
  fprintf(javaFile, "//----------------------------------------------------------------------------");
  fprintf(javaFile, "//");
  fprintf(javaFile, "//");
  fprintf(javaFile, "//----------------------------------------------------------------------------");
  fprintf(javaFile, "//    Parameters Used");
  fprintf(javaFile, "//----------------------------------------------------------------------------");
  fprintf(javaFile, "//   Time Slice= %f", TIME_SLICE);
  fprintf(javaFile, "//   Max Vel   = %f", MAX_VEL);
  fprintf(javaFile, "//   Max Accel = %f", MAX_ACCEL);
  fprintf(javaFile, "//   Max Jerk  = %f", MAX_JERK);

  fprintf(javaFile, "package org.usfirst.frc4089.paths;\n");
  fprintf(javaFile, "public class %sPath extends Path {\n", fields[0]);
  fprintf(javaFile, "    public %sPath() {\n", fields[0]);
  fprintf(javaFile, "       kNumPoints =%d;\n", length);
  fprintf(javaFile, "       kPoints = new double[][]{\n");

  for (int i = 0; i < length; i++)
  {
    fprintf(javaFile, "  {%f, %f, %f, %f, %f, %f, %f}",
      leftTrajectory[i].velocity,
      rightTrajectory[i].velocity,
      r2d(leftTrajectory[i].heading),
      leftTrajectory[i].x,
      leftTrajectory[i].y,
      rightTrajectory[i].x,
      rightTrajectory[i].y);

    if (i + 1 == length)
    {
      fprintf(javaFile, "};\n");
    }
    else
    {
      fprintf(javaFile, ",\n");
    }
  }
  fprintf(javaFile, "}}\n");

  fclose(javaFile);
  free(trajectory);
  free(points);
  return;
}
