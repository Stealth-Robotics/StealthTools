using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using JimsMath;

namespace FRC2018PathMapper
{
  public partial class FRC2018PathMapper : Form
  {
    FalconPathPlanner mPath;
    DrawingSurface mSurface = new DrawingSurface(0,0);

    public FRC2018PathMapper()
    {
      InitializeComponent();

      //long start = System.currentTimeMillis();
      //System.setProperty("java.awt.headless", "true"); //enable this to true to emulate roboRio environment


      //create waypoint path
      double[][] waypoints = new double[4][];

      waypoints[0] = new double[2];
      waypoints[0][0] = 0.0; waypoints[0][1] = 65.0;
      waypoints[1] = new double[2];
      waypoints[1][0] = 240.0; waypoints[1][1] = 65.0;
      waypoints[2] = new double[2];
      waypoints[2][0] = 240.0; waypoints[2][1] = 240.0;
      waypoints[3] = new double[2];
      waypoints[3][0] = 290.0; waypoints[3][1] = 240.0;
      /*
                        waypoints[2] = new double[2];
                        waypoints[2][0] = 5.0; waypoints[2][1] = 12.0;
                        waypoints[3] = new double[2];
                        waypoints[3][0] = 12.0; waypoints[3][1] = 12.0;
                        waypoints[4] = new double[2];
                        waypoints[4][0] = 15.0; waypoints[4][1] = 4.0;
                        waypoints[5] = new double[2];
                        waypoints[5][0] = 19.0; waypoints[5][1] = 12.0;
                        waypoints[6] = new double[2];
                        waypoints[6][0] = 0.0; waypoints[6][1] = 0.0;

                        for (int i = 0; i < waypoints.Length; i++)
                        {
                          waypoints[i][0] *= 30;
                          waypoints[i][1] *= 30;
                        }
                  */
      double totalTime = 5; //seconds
      double timeStep = 0.02; //period of control loop on Rio, seconds
      double robotTrackWidth = 22; //distance between left and right wheels, feet

      mPath = new FalconPathPlanner(waypoints);
      mPath.calculate(totalTime, timeStep, robotTrackWidth);

      for (int i = 0; i < waypoints.Length - 1; i++)
      {
        mSurface.Add(new JimsMath.Line2D(new Point2D(waypoints[i][0], waypoints[i][1]),
          new Point2D(waypoints[i + 1][0], waypoints[i + 1][1])));
      }

      for (int i = 0; i < mPath.leftPath.Length; i++)
      {
        mSurface.Add(new Point2D(Pens.Blue,mPath.leftPath[i][0], mPath.leftPath[i][1]));
      }

      for (int i = 0; i < mPath.rightPath.Length; i++)
      {
        mSurface.Add(new Point2D(Pens.Orange, mPath.rightPath[i][0], mPath.rightPath[i][1]));
      }
/*
      for (int i = 0; i < mPath.heading.Length; i++)
      {
        mSurface.Add(new Point2D(Pens.Red, mPath.heading[i][0], mPath.heading[i][1]));
      }
*/
      //System.out.println("Time in ms: " + (System.currentTimeMillis() - start));
    }

    private void FRC2018PathMapper_Paint(object sender, PaintEventArgs e)
    {

    }

    private void bDraw_Click(object sender, EventArgs e)
    {
      Graphics graph = pdField.CreateGraphics();

      mSurface.Draw(graph);

      lbSteps.Items.Clear();

      for (int i = 0; i < mPath.leftPath.Length; i++)
      {
        lbSteps.Items.Add(i.ToString()+", "+
          mPath.leftPath[i][0].ToString("F1") + ", " +
          mPath.leftPath[i][1].ToString("F1") + ", " +
          mPath.rightPath[i][0].ToString("F1") + ", " +
          mPath.rightPath[i][1].ToString("F1"));
      }


    }
  }
}
