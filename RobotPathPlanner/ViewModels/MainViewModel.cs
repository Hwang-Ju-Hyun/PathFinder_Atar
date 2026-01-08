using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Collections.ObjectModel;
using RobotPathPlanner.Models;
using System.Windows.Media;
using RobotPathPlanner.Util;
using System.Diagnostics;
using System.Numerics;
using RobotPathPlanner;
using System.Windows.Input;
using System.Windows.Documents;
using System.Xml.Linq;
using System.Windows.Media.Animation;

namespace RobotPathPlanner.ViewModels
{
    public class MainViewModel:INotifyPropertyChanged
    {
        private int grid_size = 30;
        Node[,] grid;

        const int Straight_Cost = 10;
        const int Diagonal_Cost = 14;

        public int GridSize
        {
            get { return grid_size; }
            set { grid_size = value; OnPropertyChanged(nameof(GridSize)); }
        }

        bool found = false;
        public bool Found
        {
            get { return found; }
            set { found = value; OnPropertyChanged(nameof(Found));}
        }
        public ICommand RunAStarCommand { get;}
        public ICommand Initialize { get; }
        public event PropertyChangedEventHandler PropertyChanged;
        public MainViewModel()
        {
            nodes = new ObservableCollection<Node>();
            CreateGrids();
            RunAStarCommand = new RelayCommand(FindPath_Astar);
            Initialize = new RelayCommand(Init);
        }
        private ObservableCollection<Node> nodes;
        public ObservableCollection<Node> Nodes { get { return nodes; } }
        protected void OnPropertyChanged(string propertyName)
        {
            if(PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }        

        delegate float Heuristic((float X, float Y) a, (float X, float Y) b);
        delegate int Heuristic_int((int X, int Y) a, (int X, int Y) b);
        static float GetDist((float X, float Y) a,(float X, float Y) b, Heuristic hueristic)
        {
            return hueristic(a, b);
        }
        static int GetDist((int X, int Y) a, (int X, int Y) b,  Heuristic_int hueristic)
        {
            return hueristic(a, b);
        }

        static public int Manha((int X, int Y) a, (int X, int Y) b)
        {            
            return (Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y));
        }

        static public int Octile((int X, int Y) a, (int X, int Y) b)
        {
            int dx = Math.Abs(a.X - b.X);
            int dy = Math.Abs(a.Y - b.Y);

            int diag = Math.Min(dx, dy);
            int straight = Math.Max(dx, dy) - diag;

            return diag * 14 + straight * 10;
        }

        private void CreateGrids()
        {
            grid = new Node[grid_size, grid_size];
            for(int r=0;r<GridSize;r++)
            {
                for(int c=0;c<GridSize;c++)
                {
                    Node node = new Node
                    {
                        X = c * 30,
                        Y = r * 30,
                        ROW = r,
                        COL = c,
                        Color = Brushes.LightGray,
                        Fn = 0,
                        Hn = 0,
                        Gn = int.MaxValue
                    };
                    node.Weight = 1;
                    Nodes.Add(node);                    
                    grid[r, c] = node;
                }
            }            
        }
        private Node start = null;
        private Node end = null; 
        private PriorityQueue open = null;
        public void Init()
        {
            start.Type = NodeType.EMPTY;
            end.Type = NodeType.EMPTY;
            while(open.Count()!=0)
            {
                open.Dequeue();                
            }
            open = null;
            for (int r = 0; r < GridSize;r++)
            {
                for(int c=0;c<GridSize;c++)
                {
                    grid[r, c].Type = NodeType.EMPTY;
                    grid[r, c].Fn = 0;
                    grid[r, c].Hn = 0;
                    grid[r, c].Gn = 0;
                    grid[r, c] = null;
                    
                }
            }
            start = null;
            end = null;
            IsAstarInit = false;
            CreateGrids();
        }
        bool IsAstarInit = false;
        void AstarInitialize()
        {
            start = null;
            end = null;

            for (int r = 0; r < GridSize; r++)
            {
                for (int c = 0; c < GridSize; c++)
                {
                    if (grid[r, c].Type == NodeType.START)
                    {
                        start = grid[r, c];
                        start.Gn = 0;
                        start.Weight = 1;
                    }
                    if (grid[r, c].Type == NodeType.GOAL)
                    {
                        end = grid[r, c];
                    }
                }
            }

            open = new PriorityQueue();

            start.Hn = GetDist((start.COL, start.ROW), (end.COL, end.ROW), Octile);
            start.Fn = (start.Hn * start.Weight) + start.Gn;
            open.Enqueue(start);
            IsAstarInit = true;
        }

        public void FindPath_Astar()
        {
            if(IsAstarInit==false)
                AstarInitialize();

            //12시부터 반시계
            int[] dir_x = { 0, 1, 1, 1, 0, -1, -1, -1 };
            int[] dir_y = { -1, -1, 0, 1, 1, 1, 0, -1 };

            while (open.Count()!=0)
            {
                Node cur = open.Dequeue();
                cur.Open = false;
                if (cur==end)
                {
                    found = true;
                    break;
                }

                cur.Close = true;
                if(cur!=start)
                {
                    cur.Type = NodeType.CLOSE;
                }                
                for (int i=0;i<8;i++)
                {
                    int next_row = dir_y[i]+cur.ROW;
                    int next_col = dir_x[i]+ cur.COL;

                    int cost = (dir_x[i] * dir_y[i] != 0) ? Diagonal_Cost : Straight_Cost;

                    if (next_row < 0 || next_row >= grid_size || next_col < 0 || next_col >= grid_size)
                        continue;
                    if (grid[next_row, next_col].Type == NodeType.OBSTACLE)
                        continue;

                    if (dir_x[i] * dir_y[i]!=0)
                    {
                        int adj_row1 = cur.ROW+dir_y[i];
                        int adj_col1 = cur.COL;

                        int adj_row2 = cur.ROW;
                        int adj_col2=cur.COL+dir_x[i];

                        if (grid[adj_row1,adj_col1].Type== NodeType.OBSTACLE||
                            grid[adj_row2, adj_col2].Type == NodeType.OBSTACLE)
                        {
                            continue;
                        }                            
                    }

                    Node next_node = grid[next_row, next_col];                    
                    if (next_node.Close == true)
                    {
                        continue;
                    }
                        

                    float tentativeG = cur.Gn + cost;
                    if(tentativeG<next_node.Gn)
                    {
                        next_node.Gn= tentativeG;
                        float h = GetDist((next_col, next_row), (end.COL, end.ROW), Octile);
                        next_node.Hn = h;
                        next_node.Fn = (next_node.Hn * next_node.Weight) + next_node.Gn;
                        next_node.Parent = cur;
                        open.Enqueue(next_node);
                        if(next_node.Type!=NodeType.GOAL)
                            next_node.Type = NodeType.Open;
                    }
                }
            }
            if(found)
            {
                Node cur = end;               
                while (cur!=start)
                {
                    if (cur != end)
                        grid[cur.ROW, cur.COL].Type = NodeType.PATH;                    
                    cur = cur.Parent;
                }
            }            
        }
    }
}
