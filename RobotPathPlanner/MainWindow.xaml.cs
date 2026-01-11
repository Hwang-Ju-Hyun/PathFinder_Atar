using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobotPathPlanner.Models;
using System.Windows.Input;
using System.Windows.Shapes;
using RobotPathPlanner.ViewModels;
using RobotPathPlanner.Models;

namespace RobotPathPlanner.Views
{
    public partial class MainWindow
    {
        public MainWindow()
        {
            InitializeComponent();
        }
        private void OnNodeClicked(object sender, MouseButtonEventArgs e)
        {
            if (sender is Rectangle rect && rect.DataContext is Node node)
            {
                if (Keyboard.IsKeyDown(Key.LeftShift))
                    node.Type = NodeType.START;
                else if (Keyboard.IsKeyDown(Key.LeftCtrl))
                    node.Type = NodeType.GOAL;
                else
                    node.Type = node.Type == NodeType.OBSTACLE? NodeType.EMPTY: NodeType.OBSTACLE;
            }
        }

        private void Button_Click(object sender, System.Windows.RoutedEventArgs e)
        {

        }
    }
}
