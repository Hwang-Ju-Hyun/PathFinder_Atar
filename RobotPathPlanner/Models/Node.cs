using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;


namespace RobotPathPlanner.Models
{
    public enum NodeType
    {
        EMPTY,
        OBSTACLE,
        START,
        GOAL,
        PATH,
        Open,
        CLOSE
    }
    public class Node :INotifyPropertyChanged
    {
        private Brush color = Brushes.LightGray;
        private NodeType type;
        public NodeType Type
        {
            get { return type; }
            set { type = value; OnPropertyChanged(nameof(Type));OnPropertyChanged(nameof(Color)); }
        }
        public int COL {  get; set; }
        public int ROW { get; set; }
        
        private Point pos;        
        public Point Pos
        {
            get { return pos; }
            set { pos = value; }
        }
        public float X {  get; set; }
        public float Y {  get; set; }
                
        Node parent;
        public Node Parent
        {
            get { return parent; }
            set { parent = value; OnPropertyChanged(nameof(Parent));}
        }

        public static bool operator == (Node n1,Node n2)
        {            
            if (n1 is null || n2 is null)
                return false;
            return (n1.ROW==n2.ROW)&&(n1.COL==n2.COL);
        }
        public static bool operator !=(Node n1, Node n2)
        {
            if (n1 is null || n2 is null)
                return false;
            return (n1.ROW != n2.ROW) || (n1.COL != n2.COL);
        }
        public override bool Equals(object obj)
        {
            if(obj is Node other)
            {
                return this.ROW==other.ROW&& this.COL==other.COL;
            }
            return false;
        }        

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string name)
        {
            if(PropertyChanged!=null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(name));
            }
        }

        private float f;
        private float h;
        private float g;
        private float w;
        private bool close;
        private bool open;
        public float Fn
        {
            get { return f; }
            set { f= value; OnPropertyChanged(nameof(Fn));}
        }
        public float Hn
        {
            get { return h; }
            set { h=value; OnPropertyChanged(nameof(Hn));}
        }
        public float Gn
        {
            get { return g; }
            set { g=value; OnPropertyChanged(nameof(Gn));}
        }

        public float Weight
        {
            get { return w; }
            set { w=value; OnPropertyChanged(nameof(Weight));}
        }

        public bool Close
        {
            get { return close; }
            set { close=value; OnPropertyChanged(nameof(Close));}
        }

        public bool Open
        {
            get { return open; }
            set { open=value; OnPropertyChanged(nameof(Open));}
        }

        public Brush Color 
        {
            get
            {
                switch(type)
                {                    
                    case NodeType.OBSTACLE:
                        color = Brushes.Red;
                        break;
                    case NodeType.START:
                        color= Brushes.Green;
                        break;
                    case NodeType.GOAL:
                        color = Brushes.Blue;
                        break;
                    case NodeType.PATH:
                        color= Brushes.Yellow;
                        break;
                    case NodeType.Open:
                        color = Brushes.Aqua;
                        break;
                    case NodeType.CLOSE:
                        color= Brushes.Magenta;
                        break;
                    default:
                        color = Brushes.LightGray;
                        break;
                }
                return color; 
            }
            set{ color = value; }
        }

    }
}
