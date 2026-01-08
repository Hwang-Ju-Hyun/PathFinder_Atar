using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xaml;
using RobotPathPlanner.Models;
using RobotPathPlanner.ViewModels;

namespace RobotPathPlanner.Util
{
    public class PriorityQueue
    {
        List<Node> heap;
        int count;
        public PriorityQueue()
        {
            heap = new List<Node>();
        }

        public int Count()
        {
            return heap.Count;
        }

        public Node Peek()
        {
            if (heap.Count == 0)
            {
                throw new ApplicationException("heap is empty");
            }       
            return heap[0];
        }

        public void Enqueue(Node node)
        {
            heap.Add(node);            
            int idx=heap.Count-1;
            while(idx>0)
            {
                int parent = (idx-1) / 2;
                int child = idx;
                if (heap[parent].Fn > heap[child].Fn)
                {
                    Swap( parent, child);
                    idx = parent;
                }
                else
                {
                    break;
                }
            }
        }

        public Node Dequeue()
        {
            if (heap.Count == 0)
                throw new InvalidOperationException("Heap is empty");

            Node data = heap[0];

            heap[0] = heap[heap.Count - 1];
            heap.RemoveAt(heap.Count - 1);

            int parent = 0;

            while (true)
            {
                int left = parent * 2 + 1;
                int right = left + 1;

                if (left >= heap.Count)
                    break;
                int child = left;

                if (right < heap.Count)
                {
                    if (heap[right].Fn < heap[left].Fn ||
                       (heap[right].Fn == heap[left].Fn && heap[right].Hn < heap[left].Hn))
                    {
                        child = right;
                    }
                }
                if (heap[parent].Fn > heap[child].Fn ||
                   (heap[parent].Fn == heap[child].Fn && heap[parent].Hn > heap[child].Hn))
                {
                    Swap(parent, child);
                    parent = child;
                }
                else
                {
                    break; //  이게 핵심
                }
            }
            return data;
        }
        public void Swap(int parent_idx,int child_idx)
        {
            Node temp;
            temp = heap[parent_idx];
            heap[parent_idx] = heap[child_idx];
            heap[child_idx] = temp;
        }        
    }
}