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
using System.Security.Cryptography;
using System.Threading;
using System.Windows;

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

        private bool isPaused = false;
        public bool IsPaused
        {
            get { return isPaused; }
            set { isPaused = value; OnPropertyChanged(nameof(IsPaused)); }
        }

        private int stepDelay = 20;//1/100second
        public int StepDelay
        {
            get { return stepDelay; }
            set { stepDelay=value; OnPropertyChanged(nameof(StepDelay));}
        }

        public ICommand RunAStarCommand { get;}
        public ICommand Initialize { get; }    
        public ICommand PauseResume { get; }

        private string hueristic_name;
        public string Hueristic_name
        {
            get { return hueristic_name; }
            set { hueristic_name = value; }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        public MainViewModel()
        {
            nodes = new ObservableCollection<Node>();
            CreateGrids();
            RunAStarCommand = new RelayCommand(FindPath_Astar);
            Initialize = new RelayCommand(Init);
            PauseResume = new RelayCommand(ToggleWait);
        }

        void ToggleWait()
        {
            IsPaused = !IsPaused;
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

        private CancellationTokenSource astarCts;


        delegate float Heuristic((float X, float Y) a, (float X, float Y) b);
        Heuristic H_Method;
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

        static public float Euclidean((float X, float Y) a, (float X, float Y) b)
        {
            Point p1=new Point(a.X, a.Y);
            Point p2 =new Point(b.X, b.Y);
            return (float)Math.Sqrt((Math.Pow(Math.Abs(p1.X - p2.X),2) + Math.Pow(Math.Abs(p1.Y - p2.Y),2)));
        }

        static public float Octile((float X, float Y) a, (float X, float Y) b)
        {
            float dx = Math.Abs(a.X - b.X);
            float dy = Math.Abs(a.Y - b.Y);
            
            float diag = Math.Min(dx, dy);
            float straight = Math.Max(dx, dy) - diag;

            return diag * 14 + straight * 10;
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
                    Point p = new Point(node.X,node.Y);
                    node.Pos = p;
                    node.Weight = 1;
                    Nodes.Add(node);                    
                    grid[r, c] = node;
                }
            }            
        }

        private Node start = null;
        private Node end = null; 
        private PriorityQueue open = null;

        bool LineOfSight(Node a, Node b)
        {
            int x0 = a.COL;
            int y0 = a.ROW;
            int x1 = b.COL;
            int y1 = b.ROW;

            int dx = Math.Abs(x1 - x0);
            int dy = Math.Abs(y1 - y0);

            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;

            int err = dx - dy;

            while (true)
            {
                if (grid[y0, x0].Type == NodeType.OBSTACLE)
                    return false;

                if (x0 == x1 && y0 == y1)
                    break;

                int e2 = 2 * err;
                if (e2 > -dy)
                {
                    err -= dy;
                    x0 += sx;
                }
                if (e2 < dx)
                {
                    err += dx;
                    y0 += sy;
                }
            }
            return true;
        }

        double Distance(Node a, Node b)
        {
            float dx = (float)(a.Pos.X - b.Pos.X);
            float dy = (float)(a.Pos.Y - b.Pos.Y);

            return Math.Sqrt(dx * dx + dy * dy);
        }

        public void Init()
        {
            if (astarCts != null)
            {
                astarCts.Cancel();
                astarCts = null;
            }
            IsPaused = false;
            IsAstarInit = false;
            found = false;

            if (start!=null)
                start.Type = NodeType.EMPTY;
            if(end!=null)
                end.Type = NodeType.EMPTY;
            if(open!=null)
            {
                while (open.Count() != 0)
                {
                    open.Dequeue();
                }
                open = null;
            }            
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
            Nodes.Clear();
            CreateGrids();
        }

        bool IsAstarInit = false;       
        async Task WaitIfPaused()
        {
            while(IsPaused)
            {
                await Task.Delay(50);
            }
        }

        void AstarInitialize()
        {
            
            start = null;
            end = null;
            int start_cnt = 0;
            int end_cnt = 0;
            for (int r = 0; r < GridSize; r++)
            {
                for (int c = 0; c < GridSize; c++)
                {
                    if (grid[r, c].Type == NodeType.START)
                    {
                        if(start_cnt == 0)
                        {
                            start = grid[r, c];
                            start.Gn = 0;
                            start.Weight = 1;
                            start_cnt++;
                        }
                        else if(start_cnt>0)
                        {
                            throw new ApplicationException("출발지점이 두 곳 이상 존재합니다");
                        }
                    }                    
                    if (grid[r, c].Type == NodeType.GOAL)
                    {
                        if(end_cnt == 0)
                        {
                            end = grid[r, c];
                            end_cnt++;
                        }
                        else if(end_cnt>0)
                        {
                            throw new ApplicationException("도착지점이 두 곳 이상 존재합니다");
                        }                        
                    }
                }
            }            
            if(start_cnt==0)
            {
                throw new ApplicationException("출발지점이 존재하지 않습니다");
            }
            if (end_cnt == 0)
            {
                throw new ApplicationException("도착지점이 존재하지 않습니다");
            }
            if(start_cnt+end_cnt==0)
            {
                throw new ApplicationException("출발지점과 도착지점이 존재하지 않습니다.");         
            }

            open = new PriorityQueue();
            H_Method = Octile;
            start.Hn = GetDist(((float)start.COL, (float)start.ROW), ((float)end.COL, (float)end.ROW), H_Method);
            
            if(H_Method!=null)
            {
                Hueristic_name= H_Method.Method.Name;
                if (Hueristic_name == "Octile"|| Hueristic_name == "Manha")
                {
                    start.Parent = null;
                }
                else if(Hueristic_name == "Euclidean")
                {
                    start.Parent = start;
                }
            }

            start.Fn = (start.Hn * start.Weight) + start.Gn;
            open.Enqueue(start);
            IsAstarInit = true;
        }
        
        public async void FindPath_Astar()
        {
            if(astarCts!=null)
            {
                astarCts.Cancel();
            }            
            astarCts = new CancellationTokenSource();
            CancellationToken token = astarCts.Token;

            if (IsAstarInit==false)
                AstarInitialize();            

            //12시부터 반시계
            int[] dir_x = { 0, 1, 1, 1, 0, -1, -1, -1 };
            int[] dir_y = { -1, -1, 0, 1, 1, 1, 0, -1 };

            try
            {
                while (open.Count() != 0)
                {
                    token.ThrowIfCancellationRequested();
                    
                    Node cur = open.Dequeue();
                    cur.Open = false;
                    if (cur == end)
                    {
                        found = true;
                        break;
                    }

                    cur.Close = true;
                    if (cur != start)
                    {
                        cur.Type = NodeType.CLOSE;
                    }

                    
                    await WaitIfPaused();
                    await Task.Delay(StepDelay, token);


                    for (int i = 0; i < 8; i++)
                    {
                        token.ThrowIfCancellationRequested();

                        int next_row = dir_y[i] + cur.ROW;
                        int next_col = dir_x[i] + cur.COL;

                        int cost = (dir_x[i] * dir_y[i] != 0) ? Diagonal_Cost : Straight_Cost;

                        if (next_row < 0 || next_row >= grid_size || next_col < 0 || next_col >= grid_size)
                            continue;
                        if (grid[next_row, next_col].Type == NodeType.OBSTACLE)
                            continue;

                        if (dir_x[i] * dir_y[i] != 0)
                        {
                            int adj_row1 = cur.ROW + dir_y[i];
                            int adj_col1 = cur.COL;

                            int adj_row2 = cur.ROW;
                            int adj_col2 = cur.COL + dir_x[i];

                            if (grid[adj_row1, adj_col1].Type == NodeType.OBSTACLE ||
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


                        float tentativeG; /*= cur.Gn + cost;*/

                        if (hueristic_name == "Euclidean")
                        {
                            if (cur.Parent != null && LineOfSight(cur.Parent, next_node))
                            {
                                tentativeG = cur.Parent.Gn + (float)Distance(cur.Parent, next_node);

                                if (tentativeG < next_node.Gn)
                                {
                                    next_node.Gn = tentativeG;
                                    next_node.Parent = cur.Parent;
                                }
                            }
                            else
                            {
                                tentativeG = cur.Gn + (float)Distance(cur.Parent, next_node);
                                if (tentativeG < next_node.Gn)
                                {
                                    next_node.Gn = tentativeG;
                                    next_node.Parent = cur;
                                }
                            }
                        }
                        else
                        {
                            tentativeG= cur.Gn + cost;
                            if (tentativeG < next_node.Gn)
                            {
                                next_node.Gn = tentativeG;                                                                                                                                                                
                            }
                        }

                        next_node.Hn = GetDist((next_node.COL, next_node.ROW),(end.COL,end.ROW),H_Method);
                        next_node.Fn = (next_node.Hn * next_node.Weight) + next_node.Gn;
                        next_node.Parent = cur;
                        open.Enqueue(next_node);
                        if (next_node.Type != NodeType.GOAL)
                            next_node.Type = NodeType.Open;
                    }
                }
            }
            catch(OperationCanceledException e)
            {
                Debug.WriteLine(e.Message);
                Debug.WriteLine("Astar Cancel");
                return;
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
