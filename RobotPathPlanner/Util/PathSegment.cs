using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace RobotPathPlanner.Util
{
    public class PathSegment
    {
        private Point from;
        private Point to;
        public Point From
        {
            get { return from; }
            set { from = value; }
        }
        public Point To
        {
            get { return to; }
            set { to = value; }
        }
    }
}
